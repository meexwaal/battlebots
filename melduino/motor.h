#pragma once

#include <Servo.h>

namespace melty {

    // Found with guess and check
    static constexpr size_t min_pulse_us = 1266;
    static constexpr size_t max_pulse_us = 1678;

    // TODO: refactor in terms of +Y and -Y motor
    Servo py_motor, ny_motor;

    bool motor_init() {
        // Flag failures in init so the main program can show them, but don't give
        // up trying to init.
        bool success = true;

        if (!py_motor.attach(pins::py_motor)) {
            Serial.println("+Y motor failed to attach");
            success = false;
        }
        if (!ny_motor.attach(pins::ny_motor)) {
            Serial.println("-Y motor failed to attach");
            success = false;
        }

        // ESC initialization sequence
        Serial.println("Init motor: high duty cycle");
        py_motor.writeMicroseconds(max_pulse_us);
        ny_motor.writeMicroseconds(max_pulse_us);
        delay(5000); // 7000 for more beeps, they just keep going

        Serial.println("Init motor: low duty cycle");
        py_motor.writeMicroseconds(min_pulse_us);
        ny_motor.writeMicroseconds(min_pulse_us);
        delay(2000);

        Serial.println("Init motor: done");

        return success;
    }

    /*
     * Set the motor speeds. Each can be from 0 (off) to 1 (max speed).
     */
    template <bool verbose = false>
    void set_motors(float py_speed, float ny_speed) {
        constexpr float speed_limit = 0.12f; // TODO: remove limit
        /* constexpr float speed_limit = 1.0f; */

        if (py_speed < 0.0f || py_speed > 1.0f) {
            if (verbose) {
                Serial.print("Error: bad +Y motor speed: ");
                Serial.println(py_speed);
            }
            py_speed = 0.0f;
        }
        py_speed = constrain(py_speed, 0.0f, speed_limit);

        if (ny_speed < 0.0f || ny_speed > 1.0f) {
            if (verbose) {
                Serial.print("Error: bad -Y motor speed: ");
                Serial.println(ny_speed);
            }
            ny_speed = 0.0f;
        }
        ny_speed = constrain(ny_speed, 0.0f, speed_limit);

        py_motor.writeMicroseconds(map_float(py_speed, 0.0f, 1.0f, min_pulse_us, max_pulse_us));
        ny_motor.writeMicroseconds(map_float(ny_speed, 0.0f, 1.0f, min_pulse_us, max_pulse_us));
    }
}
