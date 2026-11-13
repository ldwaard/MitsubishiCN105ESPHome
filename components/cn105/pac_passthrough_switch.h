#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"
#include <functional>

namespace esphome {

    class PacPassthroughSwitch : public switch_::Switch, public Component {
    public:
        using CallbackFunction = std::function<void(bool state)>;

        void setCallbackFunction(CallbackFunction&& callback) {
            this->callBackFunction = std::move(callback);
        }

        // Apply restore_mode (ALWAYS_OFF by default). A restored "on" goes through
        // write_state(), so the callback forces passthrough before the first loop.
        // "Off" is already the forced state's default, so it is only published.
        void setup() override {
            if (this->get_initial_state_with_restore_mode().value_or(false)) {
                this->turn_on();
            } else {
                this->publish_state(false);
            }
        }

    protected:
        void write_state(bool state) override {
            if (callBackFunction) {
                callBackFunction(state);
            }
            this->publish_state(state);   // optimistic: no readback exists for this
        }

    private:
        CallbackFunction callBackFunction;
    };

}
