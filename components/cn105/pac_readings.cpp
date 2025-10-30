#include "cn105.h"
#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

using namespace esphome;

static const char* const PAC_TAG = "PAC";
static constexpr uint32_t HA_RETURN_DEBOUNCE_MS = 5000;
static constexpr uint32_t PAC_CACHE_MAX_AGE_MS = 30000;   // older than this => stay silent, don't lie to the controller
static constexpr uint32_t PAC_RELAY_MAX_WAIT_MS = 2000;
static constexpr uint8_t PAC_MAX_UNANSWERED_FORWARDS = 3;
static constexpr uint32_t PAC_RECONNECT_MIN_INTERVAL_MS = 15000;   // longer than the 10 s handshake retry in sendFirstConnectionPacket()

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
    if (!this->pac_parser_.checksum_valid() || this->pac_parser_.raw()[2] != HEADER[2] || this->pac_parser_.raw()[3] != HEADER[3]) {
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
        if (this->is_pac_passthrough()) {
            this->forwardPacFrameToHp();
        } else if (this->pac_parser_.command() == 0x42) {
            this->replyCachedInfo(this->pac_parser_.data()[0]);
        } else {
            this->logPacSet(this->pac_parser_.data(), this->pac_parser_.data_length());
            this->replyPacSetAck(this->pac_parser_.data()[0]);
        }
        break;
    default:
        ESP_LOGD(PAC_TAG, "unhandled upstream cmd 0x%02X", this->pac_parser_.command());
        break;
    }
}

// ───────────── slave emulation (normal mode) ─────────────

void CN105Climate::sendPacFrame(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    if (this->pac_uart_ == nullptr || len > 32) {
        return;
    }
    uint8_t buf[5 + 32 + 1];
    const int frame_size = cn105_protocol::build_frame(cmd, payload, len, buf);
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
    this->hpPacketDebug(buf, frame_size, "PAC_WRITE");
#endif
    this->get_pac_uart_()->write_array(buf, frame_size);
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

void CN105Climate::replyCachedInfo(uint8_t code) {
    for (const auto& cache_entry : this->pac_cache_) {
        if (!cache_entry.valid || cache_entry.code != code) {
            continue;
        }
        if (CUSTOM_MILLIS - cache_entry.ms > PAC_CACHE_MAX_AGE_MS) {
            ESP_LOGD(PAC_TAG, "cache for 0x%02X is stale, not answering", code);
            return;
        }
        this->sendPacFrame(0x62, cache_entry.data, cache_entry.len);
        return;
    }
    ESP_LOGD(PAC_TAG, "no cached reply for 0x%02X, not answering", code);
}

void CN105Climate::cacheInfoResponse() {
    if (this->pac_uart_ == nullptr) {
        return;
    }
    const int len = this->parser_.data_length();
    if (len < 1 || len > (int)sizeof(PacCacheEntry::data)) {
        return;
    }
    const uint8_t* data = this->parser_.data();

    PacCacheEntry* slot = nullptr;
    for (auto& cache_entry : this->pac_cache_) {
        if (cache_entry.valid && cache_entry.code == data[0]) { 
            slot = &cache_entry;
            break;
        }
        if (!cache_entry.valid && slot == nullptr) {
            slot = &cache_entry;
        }
    }
    if (slot == nullptr) {
        return;
    }
    slot->valid = true;
    slot->code = data[0];
    slot->len = (uint8_t)len;
    memcpy(slot->data, data, len);
    slot->ms = CUSTOM_MILLIS;
}

// ───────────── PAC passthrough ─────────────

void CN105Climate::forwardPacFrameToHp() {
    if (!this->isHeatpumpConnected()) {
        ESP_LOGD(PAC_TAG, "dropping upstream frame (cmd=0x%02X): heat pump session is down",
            this->pac_parser_.command());
        return;
    }
    this->get_hw_serial_()->write_array(this->pac_parser_.raw(), this->pac_parser_.frame_size());
    this->lastSend = CUSTOM_MILLIS;
    this->pac_awaiting_ = true;
    this->pac_forward_ms_ = CUSTOM_MILLIS;
}

void CN105Climate::relayHpFrameToPac() {
    this->pac_unanswered_forward_count_ = 0;
    if (!this->pac_awaiting_ || CUSTOM_MILLIS - this->pac_forward_ms_ > PAC_RELAY_MAX_WAIT_MS) {
        ESP_LOGD(PAC_TAG, "dropping HP frame with no pending upstream request");
        this->pac_awaiting_ = false;
        return;
    }
    this->get_pac_uart_()->write_array(this->parser_.raw(), this->parser_.frame_size());
    this->pac_awaiting_ = false;
}

void CN105Climate::updatePacPassthroughState() {
    if (this->pac_uart_ == nullptr) {
        return;
    }
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
    const bool auto_enabled = this->pac_passthrough_timeout_ms_ != 4294967295UL;

    const bool timeout_elapsed = auto_enabled && !this->ha_ok_ && stable_ms >= this->pac_passthrough_timeout_ms_;
    const bool return_debounced = this->ha_ok_ && stable_ms >= HA_RETURN_DEBOUNCE_MS;
    // HA just came back: hold until the debounce passes
    const bool return_holdoff = this->is_pac_passthrough() && this->ha_ok_ && !return_debounced;
    const bool want_pac_passthrough = this->pac_passthrough_forced_ || timeout_elapsed || return_holdoff;

    if (want_pac_passthrough && !this->is_pac_passthrough()) {
        // only switch at a quiet moment: HP connected, no poll cycle or write in flight
        if (!this->isHeatpumpConnected() || this->loopCycle.isCycleRunning() ||
            this->wantedSettings.hasChanged || this->wantedRunStates.hasChanged) {
            return;
        }
        this->enterPacPassthrough();
    } else if (!want_pac_passthrough && this->is_pac_passthrough()) {
        this->exitPacPassthrough();
    }

    if (this->is_pac_passthrough()) {
        this->checkPacPassthroughHealth();
    }
}

void CN105Climate::enterPacPassthrough() {
    ESP_LOGW(PAC_TAG, "Entering PAC passthrough: upstream controller now talks to the heat pump");
#ifdef USE_ESP32
    std::lock_guard<std::mutex> guard(this->wantedSettingsMutex);
#endif
    this->wantedSettings.resetSettings();       // drop stale HA commands
    this->wantedRunStates.resetSettings();
    this->stopRemoteTempKeepAlive();
    this->pac_unanswered_forward_count_ = 0;
    this->pac_awaiting_ = false;
    this->pac_passthrough_active_ = true;
    this->publishPacPassthroughState();
}

void CN105Climate::exitPacPassthrough() {
    ESP_LOGI(PAC_TAG, "Leaving PAC passthrough: resuming HA control and resyncing state");
    this->pac_passthrough_active_ = false;
    this->currentSettings.resetSettings();          // force a full resync on the next poll
    this->currentRunStates.resetSettings();
    this->lastResponseMs = CUSTOM_MILLIS;           // avoid a spurious "HP lost" reconnect
    this->loopCycle.lastCompleteCycleMs = CUSTOM_MILLIS;
    this->startRemoteTempKeepAlive();
    this->publishPacPassthroughState();
}

void CN105Climate::publishPacPassthroughState() {
    if (this->pac_passthrough_sensor_ != nullptr) {
        this->pac_passthrough_sensor_->publish_state(this->is_pac_passthrough());
    }
}

// Decode an upstream SET (0x41) payload. data starts at the type byte (packet[5]),
// so data[1]/data[2] are the control masks (packet[6]/[7]) and data[n] is packet[5+n].
void CN105Climate::logPacSet(const uint8_t* data, int len) {
    const uint8_t type = data[0];

    if (len < 15) {
        ESP_LOGD(PAC_TAG, "SET 0x%02X from upstream ignored (HA in control): short payload (%d bytes)", type, len);
        return;
    }

    char desc[160];
    desc[0] = '\0';
    size_t used = 0;
    auto add = [&](const char* fmt, auto... args) {
        if (used >= sizeof(desc)) {
            return;
        }
        int frame_size = snprintf(desc + used, sizeof(desc) - used, fmt, args...);
        if (frame_size > 0) {
            used += (size_t) frame_size;
        }
    };

    switch (type) {
    case 0x01: {  // settings
        const uint8_t primary_mask = data[1];
        const uint8_t secondary_mask = data[2];
        if (primary_mask & 0x01) {
            add("power=%s ", cn105_protocol::lookup_value_opt(POWER_MAP, POWER, 2, data[3]).value_or("?"));
        }
        if (primary_mask & 0x02) {
            add("mode=%s ", cn105_protocol::lookup_value_opt(MODE_MAP, MODE, 5, data[4]).value_or("?"));
        }
        if (primary_mask & 0x04) {
            if (data[14] != 0) {                       // encoding B: packet[19]
                add("temp=%.1fC ", (data[14] - 128) / 2.0f);
            } else if (auto temp = cn105_protocol::lookup_value_opt(TEMP_MAP, TEMP, 16, data[5])) {
                add("temp=%dC ", *temp);               // encoding A: packet[10]
            } else {
                add("temp=? ");
            }
        }
        if (primary_mask & 0x08) {
            add("fan=%s ", cn105_protocol::lookup_value_opt(FAN_MAP, FAN, 6, data[6]).value_or("?"));
        }
        if (primary_mask & 0x10) {
            add("vane=%s ", cn105_protocol::lookup_value_opt(VANE_MAP, VANE, 7, data[7]).value_or("?"));
        }
        if (secondary_mask & 0x01) {
            add("widevane=%s%s ",
                cn105_protocol::lookup_value_opt(WIDEVANE_MAP, WIDEVANE, 8, data[13] & 0x0F).value_or("?"),
                (data[13] & 0x80) ? "(adj)" : "");
        }
        if (primary_mask & ~0x1F) {
            add("unknown_mask1=0x%02X ", primary_mask & ~0x1F);
        }
        if (secondary_mask & ~0x01) {
            add("unknown_mask2=0x%02X ", secondary_mask & ~0x01);
        }
        break;
    }
    case 0x07:  // remote temperature
        if (data[1] & 0x01) {
            add("remote_temp=%.1fC ", (data[3] - 128) / 2.0f);
        }
        else {
            add("remote_temp=revert to internal sensor ");
        }
        break;
    case 0x08: {  // run states
        const uint8_t primary_mask = data[1], secondary_mask = data[2];
        if (primary_mask & 0x20) {
            add("airflow=%s ", cn105_protocol::lookup_value_opt(AIRFLOW_CONTROL_MAP, AIRFLOW_CONTROL, 3, data[6]).value_or("?"));
        }
        if (secondary_mask & 0x04) {
            add("air_purifier=%s ", data[12] ? "ON" : "OFF");
        }
        if (secondary_mask & 0x08) {
            add("night_mode=%s ", data[13] ? "ON" : "OFF");
        }
        if (secondary_mask & 0x10) {
            add("circulator=%s ", data[14] ? "ON" : "OFF");
        }
        break;
    }
    case 0x1F:
    case 0x21:  // function settings: 15 bytes of ((code-100)<<2)|value
        for (int i = 1; i <= 15 && i < len; i++) {
            if (data[i] == 0) {
                continue;
            }
            add("fn%d=%d ", ((data[i] >> 2) & 0xFF) + 100, data[i] & 3);
        }
        break;
    default:
        add("unknown SET type, raw: ");
        for (int i = 1; i < len && used + 3 < sizeof(desc); i++) {
            add("%02X ", data[i]);
        }
        break;
    }

    if (desc[0] == '\0') {
        add("(no fields flagged)");
    }
    ESP_LOGD(PAC_TAG, "SET 0x%02X from upstream ignored (HA in control): %s", type, desc);
}

void CN105Climate::checkPacPassthroughHealth() {
    const uint32_t now_ms = CUSTOM_MILLIS;

    // A forwarded request that never got a reply counts as a failure
    if (this->pac_awaiting_ && now_ms - this->pac_forward_ms_ > PAC_RELAY_MAX_WAIT_MS) {
        this->pac_awaiting_ = false;
        this->pac_unanswered_forward_count_++;
        ESP_LOGW(PAC_TAG, "No heat pump reply to forwarded request (%u/%u)",
            this->pac_unanswered_forward_count_, PAC_MAX_UNANSWERED_FORWARDS);
    }

    // Handshake already in flight: sendFirstConnectionPacket() has its own 10 s retry
    if (this->driver_state() == DriverState::CONNECTING) {
        return;
    }

    const bool session_lost = !this->isHeatpumpConnected() ||
        this->pac_unanswered_forward_count_ >= PAC_MAX_UNANSWERED_FORWARDS;
    if (!session_lost) {
        return;
    }
    if (now_ms - this->lastReconnectTimeMs < PAC_RECONNECT_MIN_INTERVAL_MS) {
        return;
    }

    ESP_LOGW(PAC_TAG, "Heat pump session lost during PAC passthrough, reconnecting...");
    this->pac_unanswered_forward_count_ = 0;
    this->reconnectUART();      // PAC passthrough stays active; frames are dropped until 0x7A/0x7B arrives
}
