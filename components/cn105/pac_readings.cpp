#include "cn105.h"
#include <cstdarg>
#include <cstdio>
#include <algorithm>
#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif
#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_client.h"
#endif

using namespace esphome;

namespace {
    struct DescBuf {
        char buf[160] = {};
        size_t used = 0;

        __attribute__((format(printf, 2, 3)))
        void append(const char* fmt, ...) {
            if (used >= sizeof(buf) - 1) {
                // already full/truncated
                return;
            }
            va_list args;
            va_start(args, fmt);
            int n = vsnprintf(buf + used, sizeof(buf) - used, fmt, args);
            va_end(args);
            if (n > 0) {
                // vsnprintf returns the would-be length; clamp so `used` never passes the buffer
                used += std::min(static_cast<size_t>(n), sizeof(buf) - 1 - used);
            }
        }
        bool empty() const { return used == 0; }
    };
}

static const char* const PAC_TAG = "PAC";
static constexpr uint32_t HA_RETURN_DEBOUNCE_MS = 5000;
// older than this => stay silent, don't lie to the controller
static constexpr uint32_t PAC_CACHE_MAX_AGE_MS = 30000;
static constexpr uint32_t PAC_RELAY_MAX_WAIT_MS = 2000;
static constexpr uint8_t PAC_MAX_UNANSWERED_FORWARDS = 3;
// longer than the 10 s handshake retry in sendFirstConnectionPacket()
static constexpr uint32_t PAC_RECONNECT_MIN_INTERVAL_MS = 15000;
// longer than the heat pump's reply time
static constexpr uint32_t PAC_ENTRY_QUIET_MS = 500;

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
    if (!this->pac_parser_.checksum_valid() || !this->pac_parser_.header_valid()) {
        ESP_LOGD(PAC_TAG, "dropping invalid frame (cmd=0x%02X)", this->pac_parser_.command());
        return;
    }
    this->hpPacketDebug(this->pac_parser_.raw(), this->pac_parser_.frame_size(), "PAC_READ");

    switch (this->pac_parser_.command()) {
    case 0x5A: // connect (user)
        // always answered locally, never forwarded.
        // the ESP already owns the HP session
        this->replyPacConnect();
        break;
    case 0x5B: // connect (installer)
        // answered locally only when installer_mode is enabled and heat pump responsed to installer connect, never forwarded.
        if (this->installer_session_active_) {
            this->replyPacInstallerConnect();
        } else {
            ESP_LOGD(PAC_TAG, "installer connect requested but installer_mode is off, not answering");
        }
        break;
    case 0x41: // set
        if (this->is_pac_passthrough()) {
            const uint8_t requestType = this->pac_parser_.data()[0];
            const bool is_temperature_set_cmd = requestType == REMOTE_TEMP;
            const bool is_functions_set_cmd = requestType == FUNCTIONS_SET_PART1 || requestType == FUNCTIONS_SET_PART2;

            if (is_functions_set_cmd && this->pac_block_functions_set_) {
                this->replyPacSetAck();
                ESP_LOGD(PAC_TAG, "Upstream functions SET 0x%02X ignored: in passthrough", requestType);
            } else if (is_temperature_set_cmd && this->remoteTemperature_ > 0) {
                this->replyPacSetAck();
                ESP_LOGD(PAC_TAG, "Upstream remote temperature ignored: ESP remote temperature is active");
            } else {
                this->forwardPacFrameToHp();
            }
        } else {
            this->replyPacSetAck();
        }
        this->logPacSet(this->pac_parser_.data(), this->pac_parser_.data_length());
        break;
    case 0x42: // info request
        if (this->is_pac_passthrough()) {
            this->forwardPacFrameToHp();
        } else {
            this->replyCachedInfo(this->pac_parser_.data()[0]);
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
    uint8_t frame_buffer[5 + 32 + 1];
    const int frame_size = cn105_protocol::build_frame(cmd, payload, len, frame_buffer);
    this->hpPacketDebug(frame_buffer, frame_size, "PAC_WRITE");
    this->get_pac_uart_()->write_array(frame_buffer, frame_size);
}

void CN105Climate::replyPacConnect() {
    const uint8_t success_status = 0x00;
    this->sendPacFrame(0x7A, &success_status, 1);
}

void CN105Climate::replyPacInstallerConnect() {
    if (this->installer_session_active_) {
        ESP_LOGD(PAC_TAG, "installer connect requested, answering with cached heat pump reply");
        this->sendPacFrame(0x7B, this->installer_connect_reply_, this->installer_connect_reply_len_);
    } else {
        ESP_LOGD(PAC_TAG, "installer connect requested, installer mode not enabled, ignoring");
    }
}

// The heat pump acks every SET with an all-zero payload. Echoing the SET type
// made the PAC treat the ack as a failure and re-handshake.
void CN105Climate::replyPacSetAck() {
    uint8_t payload[16] = {};
    this->sendPacFrame(0x61, payload, sizeof(payload));
}

void CN105Climate::replyCachedInfo(uint8_t code) {
    const int slot_index = cn105_protocol::pac_cache_index_for_code(code);
    if (slot_index < 0) {
        ESP_LOGD(PAC_TAG, "0x%02X is not a cacheable code, not answering", code);
        return;
    }
    const PacCacheEntry& cache_entry = this->pac_cache_[slot_index];
    if (!cache_entry.valid) {
        ESP_LOGD(PAC_TAG, "no cached reply for 0x%02X, not answering", code);
        return;
    }
    if (CUSTOM_MILLIS - cache_entry.ms > PAC_CACHE_MAX_AGE_MS) {
        ESP_LOGD(PAC_TAG, "cache for 0x%02X is stale, not answering", code);
        return;
    }
    this->sendPacFrame(0x62, cache_entry.data, cache_entry.len);
}

void CN105Climate::cacheInfoResponse() {
    if (this->pac_uart_ == nullptr) {
        return;
    }
    const int payload_length = this->parser_.data_length();
    if (payload_length < 1 || payload_length > static_cast<int>(sizeof(PacCacheEntry::data))) {
        return;
    }
    const uint8_t* payload = this->parser_.data();

    const int slot_index = cn105_protocol::pac_cache_index_for_code(payload[0]);
    if (slot_index < 0) {
        return;   // not a code the scheduler polls; never evict a useful entry for it
    }
    PacCacheEntry& cache_entry = this->pac_cache_[slot_index];
    cache_entry.valid = true;
    cache_entry.code = payload[0];
    cache_entry.len = static_cast<uint8_t>(payload_length);
    memcpy(cache_entry.data, payload, payload_length);
    cache_entry.ms = CUSTOM_MILLIS;
}

// ───────────── PAC passthrough ─────────────

void CN105Climate::forwardPacFrameToHp() {
    if (!this->isHeatpumpConnected()) {
        ESP_LOGD(PAC_TAG, "dropping upstream frame (cmd=0x%02X): heat pump session is down",
            this->pac_parser_.command());
        return;
    }
    this->hpPacketDebug(this->pac_parser_.raw(), this->pac_parser_.frame_size(), "PAC_FWD");
    this->get_hw_serial_()->write_array(this->pac_parser_.raw(), this->pac_parser_.frame_size());
    this->lastSend = CUSTOM_MILLIS;
    this->pac_awaiting_ = true;
    this->pac_forward_ms_ = CUSTOM_MILLIS;
}

void CN105Climate::relayHpFrameToPac() {
    if (this->pac_esp_reply_pending_) {
        const bool within_wait = CUSTOM_MILLIS - this->pac_esp_request_ms_ <= PAC_RELAY_MAX_WAIT_MS;
        this->pac_esp_reply_pending_ = false;
        if (within_wait && this->parser_.command() == 0x61) {
            ESP_LOGD(PAC_TAG, "swallowing heat pump reply to ESP remote temperature packet");
            return;
        }
    }
    this->pac_unanswered_forward_count_ = 0;
    if (!this->pac_awaiting_ || CUSTOM_MILLIS - this->pac_forward_ms_ > PAC_RELAY_MAX_WAIT_MS) {
        ESP_LOGD(PAC_TAG, "dropping HP frame with no pending upstream request");
        this->pac_awaiting_ = false;
        return;
    }
    this->hpPacketDebug(this->parser_.raw(), this->parser_.frame_size(), "PAC_RLY");
    this->get_pac_uart_()->write_array(this->parser_.raw(), this->parser_.frame_size());
    this->pac_awaiting_ = false;
}

void CN105Climate::snoopPassthroughReply() {
    if (this->parser_.data_length() < 1) {
        return;
    }

    const uint8_t replyCode = this->data[0];
    switch (replyCode) {
    case 0x02:
    case 0x03:
    case 0x04:
    case 0x06:
    case 0x09:
    case 0x42:
        this->scheduler_.mark_response_seen(replyCode);
        break;
    default:
        break;
    }
}

void CN105Climate::ha_heartbeat() {
    this->last_ha_heartbeat_ms_ = CUSTOM_MILLIS;
}

bool CN105Climate::isHomeAssistantAvailable(uint32_t now_ms) const {
    bool transport_connected = false;
#if defined(USE_API)
    transport_connected = api::global_api_server != nullptr &&
                          api::global_api_server->is_connected();
#elif defined(USE_MQTT)
    transport_connected = mqtt::global_mqtt_client != nullptr &&
                          mqtt::global_mqtt_client->is_connected();
#endif
    return (transport_connected &&
        cn105_protocol::is_ha_heartbeat_fresh(this->last_ha_heartbeat_ms_, now_ms, this->ha_heartbeat_timeout_ms_));
}

void CN105Climate::updatePacPassthroughState() {
    if (this->pac_uart_ == nullptr) {
        return;
    }
    const uint32_t now_ms = CUSTOM_MILLIS;

    const bool home_assistant_available = this->isHomeAssistantAvailable(now_ms);
    if (home_assistant_available != this->ha_ok_) {
        ESP_LOGI(PAC_TAG, "Home Assistant is now %s", home_assistant_available ? "available" : "unavailable");
        this->ha_ok_ = home_assistant_available;
        this->ha_change_ms_ = now_ms;
    }

    const bool want_passthrough = cn105_protocol::should_pac_passthrough_be_active(
        this->is_pac_passthrough(),
        this->pac_passthrough_forced_,
        this->ha_ok_,
        now_ms - this->ha_change_ms_,
        this->pac_passthrough_timeout_ms_,
        HA_RETURN_DEBOUNCE_MS);

    if (want_passthrough && !this->is_pac_passthrough()) {
        // Only switch at a quiet moment (no cycle, no pending writes, no recent write)
        if (!this->isBusQuietForPassthroughEntry()) {
            return;
        }
        this->enterPacPassthrough();
    } else if (!want_passthrough && this->is_pac_passthrough()) {
        this->exitPacPassthrough();
    }

    if (this->is_pac_passthrough()) {
        this->checkPacPassthroughHealth();
        this->sendRemoteTemperatureDuringPassthrough();
    }
}

void CN105Climate::enterPacPassthrough() {
    ESP_LOGW(PAC_TAG, "Entering PAC passthrough: upstream controller now talks to the heat pump");
#ifdef USE_ESP32
    std::lock_guard<std::mutex> guard(this->wantedSettingsMutex);
#endif
    // drop stale HA commands
    this->wantedSettings.resetSettings();
    this->wantedSettings.last_user_vane = nullptr;
    this->wantedSettings.last_user_vane_ms = 0;
    this->wantedSettings.last_user_temperature = -1.0f;
    this->wantedSettings.last_user_temperature_ms = 0;
    this->wantedRunStates.resetSettings();
    this->pac_esp_reply_pending_ = false;
    this->pac_unanswered_forward_count_ = 0;
    this->pac_awaiting_ = false;
    this->pac_passthrough_active_ = true;
    this->publishPacPassthroughState();
}

void CN105Climate::exitPacPassthrough() {
    ESP_LOGI(PAC_TAG, "Leaving PAC passthrough: resuming HA control and resyncing state");
#ifdef USE_ESP32
    std::lock_guard<std::mutex> guard(this->wantedSettingsMutex);
#endif
    // drop anything queued during passthrough
    this->wantedSettings.resetSettings();
    // resetSettings() deliberately keeps these; stale values would be re-injected into the next SET
    this->wantedSettings.last_user_vane = nullptr;
    this->wantedSettings.last_user_vane_ms = 0;
    this->wantedSettings.last_user_temperature = -1.0f;
    this->wantedSettings.last_user_temperature_ms = 0;
    this->wantedRunStates.resetSettings();
    this->isSetFunctions_ = false;

    this->pac_awaiting_ = false;
    this->pac_passthrough_active_ = false;
    // force a full resync on the next poll
    this->currentSettings.resetSettings();
    this->currentRunStates.resetSettings();
    // avoid a spurious "HP lost" reconnect
    this->lastResponseMs = CUSTOM_MILLIS;
    this->loopCycle.lastCompleteCycleMs = CUSTOM_MILLIS;
    this->pac_esp_reply_pending_ = false;
    this->publishPacPassthroughState();
}

void CN105Climate::publishPacPassthroughState() {
    if (this->pac_passthrough_sensor_ != nullptr) {
        this->pac_passthrough_sensor_->publish_state(this->is_pac_passthrough());
    }
}

// Decode an upstream SET (0x41) payload. data starts at the type byte (packet[5]),
// so data[1]/data[2] are the control masks (packet[6]/[7]) and data[n] is packet[5+n].
void CN105Climate::logPacSet(const uint8_t* payload, int len) {
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
    const uint8_t type = payload[0];

    if (len < 15) {
        if (this->is_pac_passthrough()) {
            ESP_LOGD(PAC_TAG, "SET 0x%02X from upstream: short payload (%d bytes)", type, len);
        } else {
            ESP_LOGD(PAC_TAG, "SET 0x%02X from upstream ignored (HA in control): short payload (%d bytes)", type, len);
        }
        return;
    }

    DescBuf desc;

    switch (type) {
    case 0x01: {  // settings
        const uint8_t primary_mask = payload[1];
        const uint8_t secondary_mask = payload[2];
        if (primary_mask & 0x01) {
            desc.append("power=%s ", cn105_protocol::lookup_value_opt(POWER_MAP, POWER, 2, payload[3]).value_or("?"));
        }
        if (primary_mask & 0x02) {
            desc.append("mode=%s ", cn105_protocol::lookup_value_opt(MODE_MAP, MODE, 5, payload[4]).value_or("?"));
        }
        if (primary_mask & 0x04) {
            if (payload[14] != 0) {
                // encoding B: packet[19]
                desc.append("temp=%.1fC ", (payload[14] - 128) / 2.0f);
            } else if (auto temp = cn105_protocol::lookup_value_opt(TEMP_MAP, TEMP, 16, payload[5])) {
                // encoding A: packet[10]
                desc.append("temp=%dC ", *temp);
            } else {
                // no args: fine, the literal is checked at the call site
                desc.append("temp=? ");
            }
        }
        if (primary_mask & 0x08) {
            desc.append("fan=%s ", cn105_protocol::lookup_value_opt(FAN_MAP, FAN, 6, payload[6]).value_or("?"));
        }
        if (primary_mask & 0x10) {
            desc.append("vane=%s ", cn105_protocol::lookup_value_opt(VANE_MAP, VANE, 7, payload[7]).value_or("?"));
        }
        if (secondary_mask & 0x01) {
            desc.append("widevane=%s%s ",
                cn105_protocol::lookup_value_opt(WIDEVANE_MAP, WIDEVANE, 8, payload[13] & 0x0F).value_or("?"),
                (payload[13] & 0x80) ? "(adj)" : "");
        }
        if (primary_mask & ~0x1F) {
            desc.append("unknown_mask1=0x%02X ", primary_mask & ~0x1F);
        }
        if (secondary_mask & ~0x01) {
            desc.append("unknown_mask2=0x%02X ", secondary_mask & ~0x01);
        }
        break;
    }
    case 0x07:  // remote temperature
        if (payload[1] & 0x01) {
            desc.append("remote_temp=%.1fC ", (payload[3] - 128) / 2.0f);
        }
        else {
            desc.append("remote_temp=revert to internal sensor ");
        }
        break;
    case 0x08: {  // run states
        const uint8_t primary_mask = payload[1];
        const uint8_t secondary_mask = payload[2];
        if (primary_mask & 0x20) {
            desc.append("airflow=%s ", cn105_protocol::lookup_value_opt(AIRFLOW_CONTROL_MAP, AIRFLOW_CONTROL, 3, payload[6]).value_or("?"));
        }
        if (secondary_mask & 0x04) {
            desc.append("air_purifier=%s ", payload[12] ? "ON" : "OFF");
        }
        if (secondary_mask & 0x08) {
            desc.append("night_mode=%s ", payload[13] ? "ON" : "OFF");
        }
        if (secondary_mask & 0x10) {
            desc.append("circulator=%s ", payload[14] ? "ON" : "OFF");
        }
        break;
    }
    case 0x1F:
    case 0x21:  // function settings: 15 bytes of ((code-100)<<2)|value
        for (int i = 1; i <= 15 && i < len; i++) {
            if (payload[i] == 0) {
                continue;
            }
            desc.append("fn%d=%d ", ((payload[i] >> 2) & 0xFF) + 100, payload[i] & 3);
        }
        break;
    default:
        desc.append("unknown SET type, raw: ");
        for (int i = 1; i < len && desc.used + 3 < sizeof(desc.buf); i++) {
            desc.append("%02X ", payload[i]);
        }
        break;
    }

    if (desc.empty()) {
        desc.append("(no fields flagged)");
    }
    if (this->is_pac_passthrough()) {
        ESP_LOGD(PAC_TAG, "SET 0x%02X from upstream: %s", type, desc.buf);
    } else {
        ESP_LOGD(PAC_TAG, "SET 0x%02X from upstream ignored (HA in control): %s", type, desc.buf);
    }
#endif
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

// While the upstream controller owns the bus the ESP stops polling, but the remote
// temperature from Home Assistant must still reach the heat pump, including the
// "revert to internal sensor" packet after a timeout.
void CN105Climate::sendRemoteTemperatureDuringPassthrough() {
    if (!this->shouldSendExternalTemperature_) {
        return;
    }
    // Heat pump replies are matched to requests by order, so never interleave with a
    // request the controller is waiting on. The flag stays set and is retried next loop.
    if (this->pac_awaiting_ || !this->isHeatpumpConnected()) {
        return;
    }
    this->shouldSendExternalTemperature_ = false;
    if (this->sendRemoteTemperaturePacket()) {
        this->pac_esp_reply_pending_ = true;
        this->pac_esp_request_ms_ = CUSTOM_MILLIS;
    }
}

bool CN105Climate::isBusQuietForPassthroughEntry() {
    if (!this->isHeatpumpConnected() || this->loopCycle.isCycleRunning()) {
        return false;
    }
    if (this->wantedSettings.hasChanged || this->wantedRunStates.hasChanged) {
        return false;
    }
    // A reply to our last write (e.g. the remote temperature sent at the end of a cycle)
    // may still be on its way. Relaying it to the controller would shift every later reply by one.
    return (CUSTOM_MILLIS - this->lastSend) >= PAC_ENTRY_QUIET_MS;
}


void CN105Climate::cacheInstallerConnectReply() {
    const int payload_length = this->parser_.data_length();
    if (payload_length < 1 || payload_length > static_cast<int>(sizeof(this->installer_connect_reply_))) {
        ESP_LOGW(PAC_TAG, "installer connect reply has unexpected length %d, not caching", payload_length);
        return;
    }
    memcpy(this->installer_connect_reply_, this->parser_.data(), payload_length);
    this->installer_connect_reply_len_ = static_cast<uint8_t>(payload_length);
    ESP_LOGD(PAC_TAG, "cached installer connect reply (%d bytes)", payload_length);
}
