#pragma once

#include <Adafruit_H3LIS331.h>
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

    /* Distance from center of rotation to accel sensor, in m */
    static constexpr float accel_radius = 0.0254;

    Adafruit_ICM20649 icm;
    Adafruit_H3LIS331 lis;

    bool icm_init() {
        if (!icm.begin_I2C()) {
            // todo: retry?
            Serial.println("Failed to find ICM20649 (gyro)");
            return false;
        }

        // icm.setAccelRange(ICM20649_ACCEL_RANGE_4_G);
        Serial.print("ICM accelerometer range set to: ");
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

    bool lis_init() {
        if (!lis.begin_I2C()) {
            // todo: retry?
            Serial.println("Failed to find H3LIS331 (accel)");
            return false;
        }

        // lis.setRange(H3LIS331_RANGE_100_G);   // 100, 200, or 400 G!
        Serial.print("Accelerometer range set to: ");
        switch (lis.getRange()) {
        case H3LIS331_RANGE_100_G: Serial.println("100 g"); break;
        case H3LIS331_RANGE_200_G: Serial.println("200 g"); break;
        case H3LIS331_RANGE_400_G: Serial.println("400 g"); break;
        }

        // lis.setDataRate(LIS331_DATARATE_1000_HZ);
        Serial.print("Accelerometer data rate set to: ");
        switch (lis.getDataRate()) {

        case LIS331_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
        case LIS331_DATARATE_50_HZ: Serial.println("50 Hz"); break;
        case LIS331_DATARATE_100_HZ: Serial.println("100 Hz"); break;
        case LIS331_DATARATE_400_HZ: Serial.println("400 Hz"); break;
        case LIS331_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
        case LIS331_DATARATE_LOWPOWER_0_5_HZ: Serial.println("0.5 Hz Low Power"); break;
        case LIS331_DATARATE_LOWPOWER_1_HZ: Serial.println("1 Hz Low Power"); break;
        case LIS331_DATARATE_LOWPOWER_2_HZ: Serial.println("2 Hz Low Power"); break;
        case LIS331_DATARATE_LOWPOWER_5_HZ: Serial.println("5 Hz Low Power"); break;
        case LIS331_DATARATE_LOWPOWER_10_HZ: Serial.println("10 Hz Low Power"); break;

        }
    }


    class NavState {
    public:
        NavState() :
            theta(0.0f),
            prev_t(0)
        {
        }

        bool init() {
            const bool icm_ok = icm_init();
            const bool lis_ok = lis_init();
            return icm_ok && lis_ok;
        }

        void step_theta() {
            const uint32_t t = micros();
            const uint32_t dt = t - prev_t;

            theta += (dt * 1e-6) * gyro_w;
            theta = std::fmod(theta, 2.0 * M_PI);

            prev_t = t;
        }

        void step_sensors() {
            icm_measure();
            delay(1);
            lis_measure();
            delay(1);

            gyro_w = icm_gyro.gyro.z;
            accel_w = std::sqrt(lis_accel.acceleration.y / accel_radius);
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

        bool lis_measure() {
            lis.getEvent(&lis_accel);

            /* /\* Display the results (acceleration is measured in m/s^2) *\/ */
            /* Serial.print("\t\tX: "); Serial.print(event.acceleration.x); */
            /* Serial.print(" \tY: "); Serial.print(event.acceleration.y); */
            /* Serial.print(" \tZ: "); Serial.print(event.acceleration.z); */
            /* Serial.println(" m/s^2 "); */

            /* Alternately, given the range of the H3LIS331, display the results measured in g */
            // Serial.print("\t\tX:"); Serial.print(event.acceleration.x / SENSORS_GRAVITY_STANDARD);
            // Serial.print(" \tY: "); Serial.print(event.acceleration.y / SENSORS_GRAVITY_STANDARD);
            // Serial.print(" \tZ: "); Serial.print(event.acceleration.z / SENSORS_GRAVITY_STANDARD);
            // Serial.println(" g");
        }

        /* Orientation, in radians */
        float theta;
        float gyro_w;
        float accel_w;
        uint32_t prev_t;

        sensors_event_t icm_accel;
        sensors_event_t icm_gyro;
        sensors_event_t icm_temp;
        sensors_event_t lis_accel;
    };
}
