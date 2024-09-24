#pragma once

#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/*
 * IMU sensors:
 *  - ICM20649 6-DoF accelerometer + gyroscope
 *    https://learn.adafruit.com/adafruit-icm20649-wide-range-6-dof-imu-accelerometer-and-gyro
 *  - H3LIS331 400-G accelerometer
 *    https://learn.adafruit.com/adafruit-h3lis331-and-lis331hh-high-g-3-axis-accelerometers
 */

namespace melty {

    Adafruit_ICM20649 icm;

    bool icm_init() {
        if (!icm.begin_I2C()) {
            // todo: retry?
            Serial.println("Failed to find ICM20649 (gyro)");
            return false;
        }

        // icm.setAccelRange(ICM20649_ACCEL_RANGE_4_G);
        Serial.print("Accelerometer range set to: ");
        switch (icm.getAccelRange()) {
        case ICM20649_ACCEL_RANGE_4_G:
            Serial.println("+-4G");
            break;
        case ICM20649_ACCEL_RANGE_8_G:
            Serial.println("+-8G");
            break;
        case ICM20649_ACCEL_RANGE_16_G:
            Serial.println("+-16G");
            break;
        case ICM20649_ACCEL_RANGE_30_G:
            Serial.println("+-30G");
            break;
        }

        // icm.setGyroRange(ICM20649_GYRO_RANGE_500_DPS);
        Serial.print("Gyro range set to: ");
        switch (icm.getGyroRange()) {
        case ICM20649_GYRO_RANGE_500_DPS:
            Serial.println("500 degrees/s");
            break;
        case ICM20649_GYRO_RANGE_1000_DPS:
            Serial.println("1000 degrees/s");
            break;
        case ICM20649_GYRO_RANGE_2000_DPS:
            Serial.println("2000 degrees/s");
            break;
        case ICM20649_GYRO_RANGE_4000_DPS:
            Serial.println("4000 degrees/s");
            break;
        }

        //  icm.setAccelRateDivisor(4095);
        uint16_t accel_divisor = icm.getAccelRateDivisor();
        float accel_rate = 1125 / (1.0 + accel_divisor);

        Serial.print("Accelerometer data rate divisor set to: ");
        Serial.println(accel_divisor);
        Serial.print("Accelerometer data rate (Hz) is approximately: ");
        Serial.println(accel_rate);

        //  icm.setGyroRateDivisor(255);
        uint8_t gyro_divisor = icm.getGyroRateDivisor();
        float gyro_rate = 1100 / (1.0 + gyro_divisor);

        Serial.print("Gyro data rate divisor set to: ");
        Serial.println(gyro_divisor);
        Serial.print("Gyro data rate (Hz) is approximately: ");
        Serial.println(gyro_rate);
        Serial.println();
    }


    class NavState {
    public:
        NavState() :
            theta(0.0f),
            prev_t(0)
        {
        }

        bool init() {
            return icm_init();
        }

        void step() {
            icm_measure();

            const uint32_t t = micros();
            const uint32_t dt = t - prev_t;

            theta += (dt * 1e-6) * icm_gyro.gyro.z;

            prev_t = t;
        }


        /*
         * Get a measurement from the ICM20649 gyro sensor.
         * Return true when the sensor measurement is updated since last ... todo
         *
         * TODO: this may be slow. One option for a faster driver:
         * https://forum.arduino.cc/t/icm-20649-data-aquisition-is-slow/1275971/2
         */
        bool icm_measure() {
            /* Get a new sensor reading */
            icm.getEvent(&icm_accel, &icm_gyro, &icm_temp);

            /* Serial.print("\t\tTemperature "); */
            /* Serial.print(temp.temperature); */
            /* Serial.println(" deg C"); */

            /* /\* Display the results (acceleration is measured in m/s^2) *\/ */
            /* Serial.print("\t\tAccel X: "); */
            /* Serial.print(accel.acceleration.x); */
            /* Serial.print(" \tY: "); */
            /* Serial.print(accel.acceleration.y); */
            /* Serial.print(" \tZ: "); */
            /* Serial.print(accel.acceleration.z); */
            /* Serial.println(" m/s^2 "); */

            /* /\* Display the results (acceleration is measured in m/s^2) *\/ */
            /* Serial.print("\t\tGyro X: "); */
            /* Serial.print(gyro.gyro.x); */
            /* Serial.print(" \tY: "); */
            /* Serial.print(gyro.gyro.y); */
            /* Serial.print(" \tZ: "); */
            /* Serial.print(gyro.gyro.z); */
            /* Serial.println(" radians/s "); */
            /* Serial.println(); */
        }

        /* Orientation, in radians */
        float theta;
        uint32_t prev_t;

        sensors_event_t icm_accel;
        sensors_event_t icm_gyro;
        sensors_event_t icm_temp;

    };
}
