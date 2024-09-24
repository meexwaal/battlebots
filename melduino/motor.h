#pragma once

#include <Servo.h>

namespace melty {

    Servo left_motor, right_motor;

    bool motor_init() {
        // Found with guess and check
        constexpr size_t min_pulse_us = 1250;
        constexpr size_t max_pulse_us = 1650;

        // Flag failures in init so the main program can show them, but don't give
        // up trying to init.
        bool success = true;

        /* if (!left_motor.attach(pins::left_motor, min_pulse_us, max_pulse_us)) { */
        if (!left_motor.attach(pins::left_motor)) {
            Serial.println("Left motor failed to attach");
            success = false;
        }
        /* if (!right_motor.attach(pins::right_motor, min_pulse_us, max_pulse_us)) { */
        if (!right_motor.attach(pins::right_motor)) {
            Serial.println("Right motor failed to attach");
            success = false;
        }

        /* int hi = 180; */
        /* int lo = 0; */

        int hi = 110;
        int lo = 70;

        // ESC initialization sequence
        Serial.println("hi");
        left_motor.write(hi);
        right_motor.write(hi);
        delay(5000); // 7000 for more beeps, they just keep going

        Serial.println("lo");
        left_motor.write(lo);
        right_motor.write(lo);
        delay(2000);

        return success;
    }


    /*
     * Set the motor speeds. Each can be from 0 (off) to 180 (max speed).
     */
    void set_motors(const int left, const int right) {
        left_motor.write(left);
        right_motor.write(right);
    }
}
