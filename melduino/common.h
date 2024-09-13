#pragma once

namespace melty {

    // Global pin assignments
    namespace pins {
        // UART uses 0, 1
        static constexpr int left_motor = 9;
        static constexpr int right_motor = 10;
        static constexpr int status_led = 13; // aka LED_BUILTIN
        // I2C uses 14, 15 (shared with A4, A5)
    }

    // Debug tool for showing messages from interrupt context
    char * volatile debug_str = "ok";
    volatile bool debug_set = false;
    void log(char *str) {
        // Serial.println(str);
        if (!debug_set) {
            debug_str = str;
            debug_set = true;
        }
    }
    void printlog() {
        Serial.println(debug_str);
        debug_str = "ok";
        debug_set = false;
    }

}
