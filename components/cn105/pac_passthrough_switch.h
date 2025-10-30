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