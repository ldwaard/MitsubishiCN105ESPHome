#include "cn105.h"
#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

using namespace esphome;

static const char* const PAC_TAG = "PAC";
static constexpr uint32_t HA_RETURN_DEBOUNCE_MS = 5000;
static constexpr uint32_t PAC_CACHE_MAX_AGE_MS = 30000;   // older than this => stay silent, don't lie to the controller
static constexpr uint32_t PAC_RELAY_MAX_WAIT_MS = 2000;

bool CN105Climate::processInputPac(void) {
    if (this->pac_uart_ == nullptr) {
        return false;
    }
    bool processed = false;
    while (this->get_pac_uart_()->available()) {
        processed = true;
        uint8_t inputData;
        if (this->get_pac_uart_()->read_byte(&inputData)) {
            this->pac_parser_.feed(inputData);
            if (this->pac_parser_.frame_complete()) {
                this->processPacFrame();
                this->pac_parser_.reset();
            }
        }
    }
    return processed;
}

void CN105Climate::processPacFrame() {
    if (!this->pac_parser_.checksum_valid() ||
        this->pac_parser_.raw()[2] != HEADER[2] ||
        this->pac_parser_.raw()[3] != HEADER[3]) {
        ESP_LOGD(PAC_TAG, "dropping invalid frame (cmd=0x%02X)", this->pac_parser_.command());
        return;
    }
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
    this->hpPacketDebug(this->pac_parser_.raw(), this->pac_parser_.frame_size(), "PAC_READ");
#endif

    switch (this->pac_parser_.command()) {
    case 0x5A: // connect (user)
    case 0x5B: // connect (installer)
        // always answered locally, never forwarded.
        // the ESP already owns the HP session
        this->replyPacConnect(this->pac_parser_.command());
        break;
    case 0x41: // set
    case 0x42: // info request
        if (this->passthrough_active_) {
            this->forwardPacFrameToHp();
        } else if (this->pac_parser_.command() == 0x42) {
            this->replyPacInfo(this->pac_parser_.data()[0]);
        } else {
            this->logPacSet(this->pac_parser_.data(), this->pac_parser_.data_length());
            this->replyPacSetAck(this->pac_parser_.data()[0]);
        }
        break;
    default:
        ESP_LOGD(PAC_TAG, "unhandled downstream cmd 0x%02X", this->pac_parser_.command());
        break;
    }
}

// ───────────── slave emulation (normal mode) ─────────────

void CN105Climate::sendPacFrame(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    if (this->pac_uart_ == nullptr || len > 32) return;
    uint8_t buf[5 + 32 + 1];
    const int n = cn105_protocol::build_frame(cmd, payload, len, buf);
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
    this->hpPacketDebug(buf, n, "PAC_WRITE");
#endif
    this->get_pac_uart_()->write_array(buf, n);
}

void CN105Climate::replyPacConnect(uint8_t cmd) {
    const uint8_t ok = 0x00;
    this->sendPacFrame(cmd + 0x20, &ok, 1);   // 0x5A -> 0x7A, 0x5B -> 0x7B
}

void CN105Climate::replyPacSetAck(uint8_t type) {
    uint8_t payload[16] = {};
    payload[0] = type;
    this->sendPacFrame(0x61, payload, sizeof(payload));
}

void CN105Climate::replyPacInfo(uint8_t code) {
    for (const auto& e : this->pac_cache_) {
        if (!e.valid || e.code != code) continue;
        if (CUSTOM_MILLIS - e.ms > PAC_CACHE_MAX_AGE_MS) {
            ESP_LOGD(PAC_TAG, "cache for 0x%02X is stale, not answering", code);
            return;
        }
        this->sendPacFrame(0x62, e.data, e.len);
        return;
    }
    ESP_LOGD(PAC_TAG, "no cached reply for 0x%02X, not answering", code);
}

void CN105Climate::cachePacResponse() {
    if (this->pac_uart_ == nullptr) return;
    const int len = this->parser_.data_length();
    if (len < 1 || len > (int)sizeof(PacCacheEntry::data)) return;
    const uint8_t* d = this->parser_.data();

    PacCacheEntry* slot = nullptr;
    for (auto& e : this->pac_cache_) {
        if (e.valid && e.code == d[0]) { slot = &e; break; }
        if (!e.valid && slot == nullptr) slot = &e;
    }
    if (slot == nullptr) return;
    slot->valid = true;
    slot->code = d[0];
    slot->len = (uint8_t)len;
    memcpy(slot->data, d, len);
    slot->ms = CUSTOM_MILLIS;
}

// ───────────── passthrough ─────────────

void CN105Climate::forwardPacFrameToHp() {
    this->get_hw_serial_()->write_array(this->pac_parser_.raw(), this->pac_parser_.frame_size());
    this->lastSend = CUSTOM_MILLIS;
    this->pac_awaiting_ = true;
    this->pac_forward_ms_ = CUSTOM_MILLIS;
}

void CN105Climate::relayHpFrameToPac() {
    if (!this->pac_awaiting_ || CUSTOM_MILLIS - this->pac_forward_ms_ > PAC_RELAY_MAX_WAIT_MS) {
        ESP_LOGD(PAC_TAG, "dropping HP frame with no pending downstream request");
        this->pac_awaiting_ = false;
        return;
    }
    this->get_pac_uart_()->write_array(this->parser_.raw(), this->parser_.frame_size());
    this->pac_awaiting_ = false;
}

void CN105Climate::updatePassthroughState() {
    if (this->pac_uart_ == nullptr) return;
    const uint32_t now = CUSTOM_MILLIS;

    bool ha_ok = false;
#ifdef USE_API
    ha_ok = api::global_api_server != nullptr && api::global_api_server->is_connected();
#endif
    if (ha_ok != this->ha_ok_) {
        this->ha_ok_ = ha_ok;
        this->ha_change_ms_ = now;
    }
    const uint32_t stable_ms = now - this->ha_change_ms_;
    const bool auto_enabled = this->passthrough_timeout_ms_ != 4294967295UL;

    bool want = this->passthrough_active_;
    if (this->passthrough_forced_) {
        want = true;
    } else if (!this->ha_ok_ && auto_enabled && stable_ms >= this->passthrough_timeout_ms_) {
        want = true;
    } else if (this->ha_ok_ && stable_ms >= HA_RETURN_DEBOUNCE_MS) {
        want = false;
    }

    if (want && !this->passthrough_active_) {
        // only switch at a quiet moment: HP connected, no poll cycle or write in flight
        if (!this->isHeatpumpConnected() || this->loopCycle.isCycleRunning() ||
            this->wantedSettings.hasChanged || this->wantedRunStates.hasChanged) return;
        this->enterPassthrough();
    } else if (!want && this->passthrough_active_) {
        this->exitPassthrough();
    }
}

void CN105Climate::enterPassthrough() {
    ESP_LOGW(PAC_TAG, "Entering PASSTHROUGH: downstream controller now talks to the heat pump");
    {
#ifdef USE_ESP32
        std::lock_guard<std::mutex> guard(this->wantedSettingsMutex);
#endif
        this->wantedSettings.resetSettings();       // drop stale HA commands
        this->wantedRunStates.resetSettings();
    }
    // HA's remote temperature is stale/irrelevant now; stop re-sending it.
    this->stopRemoteTempKeepAlive();
    this->cancel_timeout(SHEDULER_REMOTE_TEMP_TIMEOUT);
    this->shouldSendExternalTemperature_ = false;
    this->remoteTemperature_ = 0;

    this->pac_awaiting_ = false;
    this->passthrough_active_ = true;
    this->publishPassthroughState();
}

void CN105Climate::exitPassthrough() {
    ESP_LOGI(PAC_TAG, "Leaving PASSTHROUGH: resuming HA control and resyncing state");
    this->passthrough_active_ = false;
    this->pac_awaiting_ = false;
    this->currentSettings.resetSettings();          // force a full resync on the next poll
    this->currentRunStates.resetSettings();
    this->publishPassthroughState();
    this->lastResponseMs = CUSTOM_MILLIS;           // avoid a spurious "HP lost" reconnect
    this->loopCycle.lastCompleteCycleMs = CUSTOM_MILLIS;
}

void CN105Climate::publishPassthroughState() {
    if (this->passthrough_active_sensor_ != nullptr) {
        this->passthrough_active_sensor_->publish_state(this->passthrough_active_);
    }
}

// Decode a downstream SET (0x41) payload. data starts at the type byte (packet[5]),
// so data[1]/data[2] are the control masks (packet[6]/[7]) and data[n] is packet[5+n].
void CN105Climate::logPacSet(const uint8_t* data, int len) {
    const uint8_t type = data[0];

    if (len < 15) {
        ESP_LOGI(PAC_TAG, "SET 0x%02X from downstream ignored (HA in control): short payload (%d bytes)", type, len);
        return;
    }

    char desc[160];
    desc[0] = '\0';
    size_t used = 0;
    auto add = [&](const char* fmt, auto... args) {
        if (used >= sizeof(desc)) return;
        int n = snprintf(desc + used, sizeof(desc) - used, fmt, args...);
        if (n > 0) used += (size_t) n;
    };

    switch (type) {
    case 0x01: {  // settings
        const uint8_t m1 = data[1], m2 = data[2];
        if (m1 & 0x01) add("power=%s ", cn105_protocol::lookup_value_opt(POWER_MAP, POWER, 2, data[3]).value_or("?"));
        if (m1 & 0x02) add("mode=%s ", cn105_protocol::lookup_value_opt(MODE_MAP, MODE, 5, data[4]).value_or("?"));
        if (m1 & 0x04) {
            if (data[14] != 0) {                       // encoding B: packet[19]
                add("temp=%.1fC ", (data[14] - 128) / 2.0f);
            } else if (auto t = cn105_protocol::lookup_value_opt(TEMP_MAP, TEMP, 16, data[5])) {
                add("temp=%dC ", *t);               // encoding A: packet[10]
            } else {
                add("temp=? ");
            }
        }
        if (m1 & 0x08) add("fan=%s ", cn105_protocol::lookup_value_opt(FAN_MAP, FAN, 6, data[6]).value_or("?"));
        if (m1 & 0x10) add("vane=%s ", cn105_protocol::lookup_value_opt(VANE_MAP, VANE, 7, data[7]).value_or("?"));
        if (m2 & 0x01) {
            add("widevane=%s%s ",
                cn105_protocol::lookup_value_opt(WIDEVANE_MAP, WIDEVANE, 8, data[13] & 0x0F).value_or("?"),
                (data[13] & 0x80) ? "(adj)" : "");
        }
        if (m1 & ~0x1F) add("unknown_mask1=0x%02X ", m1 & ~0x1F);
        if (m2 & ~0x01) add("unknown_mask2=0x%02X ", m2 & ~0x01);
        break;
    }
    case 0x07:  // remote temperature
        if (data[1] & 0x01) add("remote_temp=%.1fC ", (data[3] - 128) / 2.0f);
        else             add("remote_temp=revert to internal sensor ");
        break;
    case 0x08: {  // run states
        const uint8_t m1 = data[1], m2 = data[2];
        if (m1 & 0x20) add("airflow=%s ", cn105_protocol::lookup_value_opt(AIRFLOW_CONTROL_MAP, AIRFLOW_CONTROL, 3, data[6]).value_or("?"));
        if (m2 & 0x04) add("air_purifier=%s ", data[12] ? "ON" : "OFF");
        if (m2 & 0x08) add("night_mode=%s ", data[13] ? "ON" : "OFF");
        if (m2 & 0x10) add("circulator=%s ", data[14] ? "ON" : "OFF");
        break;
    }
    case 0x1F:
    case 0x21:  // function settings: 15 bytes of ((code-100)<<2)|value
        for (int i = 1; i <= 15 && i < len; i++) {
            if (data[i] == 0) continue;
            add("fn%d=%d ", ((data[i] >> 2) & 0xFF) + 100, data[i] & 3);
        }
        break;
    default:
        add("unknown SET type, raw: ");
        for (int i = 1; i < len && used + 3 < sizeof(desc); i++) add("%02X ", data[i]);
        break;
    }

    if (desc[0] == '\0') add("(no fields flagged)");
    ESP_LOGI(PAC_TAG, "SET 0x%02X from downstream ignored (HA in control): %s", type, desc);
}
