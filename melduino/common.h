#pragma once

namespace melty {

    // Global pin assignments
    namespace pins {
        // UART uses 0, 1

        // Radio receiver
        static constexpr int ch1_spin = 2;
        static constexpr int ch3_north_south = 3;
        static constexpr int ch4_east_west = 8;

        // Indicator LED
        static constexpr int led_blue = 7;
        static constexpr int led_green = 12;

        // ESC control
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

    /*
     * Arduino's map, with floats.
     * https://www.arduino.cc/reference/en/language/functions/math/map/
     */
    inline float map_float(
        const float x,
        const float in_min,
        const float in_max,
        const float out_min,
        const float out_max
    ) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /*
     * Dot product
     */
    inline float dot(const float ax, const float ay, const float bx, const float by) {
        return ax * bx + ay * by;
    }
}
