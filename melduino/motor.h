#pragma once

#include <Servo.h>

namespace melty {

    // Found with guess and check
    static constexpr size_t min_pulse_us = 1266;
    static constexpr size_t max_pulse_us = 1678;

    Servo left_motor, right_motor;

    bool motor_init() {
        // Flag failures in init so the main program can show them, but don't give
        // up trying to init.
        bool success = true;

        if (!left_motor.attach(pins::left_motor)) {
            Serial.println("Left motor failed to attach");
            success = false;
        }
        if (!right_motor.attach(pins::right_motor)) {
            Serial.println("Right motor failed to attach");
            success = false;
        }

        // ESC initialization sequence
        Serial.println("Init motor: high duty cycle");
        left_motor.writeMicroseconds(max_pulse_us);
        right_motor.writeMicroseconds(max_pulse_us);
        delay(5000); // 7000 for more beeps, they just keep going

        Serial.println("Init motor: low duty cycle");
        left_motor.writeMicroseconds(min_pulse_us);
        right_motor.writeMicroseconds(min_pulse_us);
        delay(2000);

        Serial.println("Init motor: done");

        return success;
    }

    /*
     * Set the motor speeds. Each can be from 0 (off) to 1 (max speed).
     */
    void set_motors(float left, float right) {
        constexpr float speed_limit = 0.12f; // TODO: remove limit
        /* constexpr float speed_limit = 1.0f; */

        if (left < 0.0f || left > 1.0f) {
            Serial.print("Error: bad left motor speed: ");
            Serial.println(left);
            left = 0.0f;
        }
        left = constrain(left, 0.0f, speed_limit);

        if (right < 0.0f || right > 1.0f) {
            Serial.print("Error: bad right motor speed: ");
            Serial.println(right);
            right = 0.0f;
        }
        right = constrain(right, 0.0f, speed_limit);

        left_motor.writeMicroseconds(map_float(left, 0.0f, 1.0f, min_pulse_us, max_pulse_us));
        right_motor.writeMicroseconds(map_float(right, 0.0f, 1.0f, min_pulse_us, max_pulse_us));
    }
}
