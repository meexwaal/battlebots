/*
 * Notes on setup:
 *
 * Install libraries:
 *  - Adafruit ICM20X (2.0.7)
 *  - Adafruit LIS331 (1.0.6)
 *
 * I also had to modify
 * ~/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/cores/arduino/IRQManager.cpp
 * to set UART_SCI_PRIORITY = 6, less than TIMER_PRIORITY, so that Serial can interrupt our
 * timer-driver fast loop.
 * TODO: set up a timer interrupt with lower priority in a less hacky way.
 */

#include "FspTimer.h"
#include "pwm.h"

#include "common.h"

#include "imu.h"
#include "led_matrix.h"
#include "lidar.h"
#include "motor.h"
#include "protocol.h"
#include "wifi.h"

namespace melty {

    FspTimer isr_timer;
    PwmOut pwm(D2);
    NavState nav_state{};

    volatile bool has_lidar = false;
    volatile bool has_accel = false;
    volatile bool has_gyro = false;
    volatile bool has_wifi = false;

    volatile size_t isr_count = 0;

    volatile size_t last_start = 0;
    volatile size_t max_period = 0;
    volatile size_t max_duration = 0;

    volatile size_t lidar_idx = 0;
    std::array<volatile uint16_t, 1024> dists;

    size_t start_millis = 0;

    template <typename unused_t>
    void isr(unused_t unused) {
        const size_t start = micros();
        const size_t period = start - last_start;
        last_start = start;
        if (period > max_period) {
            max_period = period;
        }

        // Blink the light to show that ISR is running
        digitalWrite(pins::status_led, (isr_count & (1 << 6)) == 0);

        isr_count++;
        // delayMicroseconds(100);

        int max_reads = 5;
        if (has_lidar) {
            while (max_reads > 0 && Serial1.available() >= 9) {
                max_reads--;
                uint16_t m = -1;
                const bool success = lidarMeasure(m);
                if (success) {
                    dists[lidar_idx] = m;
                    lidar_idx++;
                    if (lidar_idx >= dists.size()) {
                        lidar_idx = 0;
                    }
                }
            }
        }

        const size_t duration = micros() - start;
        if (duration > max_duration) {
            max_duration = duration;
        }
    }

    bool beginTimer(float rate) {
      uint8_t timer_type = GPT_TIMER;
      int8_t tindex = FspTimer::get_available_timer(timer_type);
      if (tindex < 0){
        tindex = FspTimer::get_available_timer(timer_type, true);
      }
      if (tindex < 0){
        return false;
      }

      FspTimer::force_use_of_pwm_reserved_timer();

      if(!isr_timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, rate, 0.0f, isr)){
        return false;
      }

      if (!isr_timer.setup_overflow_irq()){
        return false;
      }

      if (!isr_timer.open()){
        return false;
      }

      if (!isr_timer.start()){
        return false;
      }
      return true;
    }

    volatile float pwm_duty = 0;
    volatile size_t last_pwm_rise_us = 0;
    volatile size_t last_pwm_fall_us = 0;
    void measure_pwm() {
        const size_t now = micros();
        if (digitalRead(D3) == HIGH) {
            const size_t last_period = now - last_pwm_rise_us;
            pwm_duty = static_cast<float>(last_pwm_fall_us - last_pwm_rise_us) / last_period;
            last_pwm_rise_us = now;
        } else {
            last_pwm_fall_us = now;
        }
    }

    constexpr unsigned int FAST_CYCLE_HZ = 500;

    // constexpr unsigned long SLOW_CYCLE_DT_US = 20000; // 50 Hz
    constexpr unsigned long SLOW_CYCLE_DT_US = 50000; // 20 Hz
    // constexpr unsigned long SLOW_CYCLE_DT_US = 100000; // 10 Hz
    // constexpr unsigned long SLOW_CYCLE_DT_US = 200000; // 5 Hz


    unsigned long next_wakeup = 0;

    size_t cycle_count = 0;
    size_t late_wakeup_count = 0;
    size_t skipped_cycle_count = 0;

    telem_packet_t telem_packet;

    IPAddress dest_addr{192,168,4,2};

    int packet_count = 0;
    void telemeter() {
        telem_packet.packet_count = packet_count;
        telem_packet.late_wakeup_count = late_wakeup_count;
        telem_packet.skipped_cycle_count = skipped_cycle_count;

        Udp.beginPacket(dest_addr, localPort);
        Udp.write(reinterpret_cast<uint8_t *>(&telem_packet), sizeof(telem_packet));
        Udp.endPacket();

        packet_count++;
    }

    void sleep_until_cycle_start() {
        const unsigned long now = micros();
        long sleep_us = next_wakeup - now;
        if (sleep_us < 0) {
            late_wakeup_count++;

            const size_t skipped_cycles = (-sleep_us) / SLOW_CYCLE_DT_US;
            skipped_cycle_count += skipped_cycles;

            sleep_us = 0;
            next_wakeup += skipped_cycles * SLOW_CYCLE_DT_US;
        }
        delayMicroseconds(sleep_us);
        next_wakeup += SLOW_CYCLE_DT_US;
    }
}

using namespace melty;

void setup() {
    pinMode(pins::status_led, OUTPUT);
    digitalWrite(pins::status_led, HIGH);

    Serial.begin(460800);
    Serial.println("Starting init");

    led_matrix_init();

    // WiFi
    led_print(led_msg::init | led_msg::wifi);
    has_wifi = wifiSetup();
    if (!has_wifi) {
        Serial.println("WiFi failed to init");
    }

    // Motors
    led_print(led_msg::init | led_msg::motor);
    const bool has_good_motors = motor_init();
    if (!has_good_motors) {
        Serial.println("Motor init failure");
    }

    /* pwm.begin(1000.0f, 0.0f); */
    /* pwm.pulse_perc(42.7f); */
    /* /\* */
    /*  * also: period(int ms), pulseWidth(int ms), same with _us suffix */
    /*  *\/ */

    /* attachInterrupt(digitalPinToInterrupt(D3), measure_pwm, CHANGE); */

    // Lidar
    led_print(led_msg::init | led_msg::lidar);
    Serial1.begin(460800);
    for (int i = 0; i < 5; i++)
    {
        // Lidar must be in idle before it accepts commands. Attempt a few times.
        if (lidarIdle()) {
            break;
        }

        Serial.println("Lidar failed to idle, trying again");
        delay(50);
    }

    if (!(
            lidarSelfTest() &&
            lidarSetFreq() &&
            lidarSetFiltering(false) &&
            lidarStartScan()
            )) {
        Serial.println("Lidar failed to init");
        has_lidar = false;
    } else {
        has_lidar = true;
    }

    // Nav sensors, IMU
    // led_print(led_msg::init | led_msg::nav); todo
    const bool has_nav = nav_state.init();
    if (!has_nav) {
        Serial.println("Nav init failure");
    }

    // Timer interrupt for fast loop
    led_print(led_msg::init | led_msg::timer);
    if (!beginTimer(FAST_CYCLE_HZ)) {
        Serial.println("Timer failed init");

        // We need the timer to run, so give up if it fails.
        while (true);
    };

    // Put some status on the LED matrix to show what failed init
    Frame end_msg(led_msg::empty);
    if (!has_wifi) {
        end_msg |= led_msg::no | led_msg::wifi;
    }
    if (!has_good_motors) {
        end_msg |= led_msg::no | led_msg::motor;
    }
    if (!has_lidar) {
        end_msg |= led_msg::no | led_msg::lidar;
    }
    if (!has_nav) {
        /* end_msg |= led_msg::no | led_msg::nav; */ // todo
    }
    led_print(end_msg);

    start_millis = millis();
}

void loop() {

    cycle_count++;
    sleep_until_cycle_start();

    /* int max_reads = 1000; */
    /* if (has_lidar) { */
    /*     while (max_reads > 0 && Serial1.available() >= 9) { */
    /*         max_reads--; */
    /*         uint16_t m = -1; */
    /*         const bool success = lidarMeasure(m); */
    /*         if (success) { */
    /*             dists[lidar_idx] = m; */
    /*             lidar_idx++; */
    /*             if (lidar_idx >= dists.size()) { */
    /*                 lidar_idx = 0; */
    /*             } */
    /*         } */
    /*     } */
    /* } */


    /* Serial.println(""); */
    /* Serial.print("Cycle #        : "); */
    /* Serial.println(cycle_count); */
    /* Serial.print("Late wakeups   : "); */
    /* Serial.println(late_wakeup_count); */
    /* Serial.print("Skipped cycles : "); */
    /* Serial.println(skipped_cycle_count); */

    /* Serial.print("Max ISR period : "); */
    /* Serial.println(max_period); */
    /* max_period = 0; */
    /* Serial.print("Max ISR dur.   : "); */
    /* Serial.println(max_duration); */
    /* max_duration = 0; */

    /* for (int i = 0; i < 8; i++) { */
    /*     Serial.println(telem_packet.lidar_mm[i]); */
    /* } */

    uint32_t sum = 0;

    for (size_t i = 0; i < dists.size(); i++) {
        sum += dists[i];
    }

    float mean = sum / dists.size();
    float std_sum = 0;
    for (const auto d : dists) {
        std_sum += (d - mean) * (d - mean);
    }
    float std = std::sqrt(std_sum / dists.size());

    /* Serial.print("mean "); */
    /* Serial.println(mean); */
    /* Serial.print("std  "); */
    /* Serial.println(std); */

    /* Serial.print("lidar_idx "); */
    /* Serial.println(lidar_idx); */
    /* for (int i = 0; i < lidar_idx; i++) { */
    /*     Serial.println(dists[i]); */
    /* } */

    /* printlog(); */

    /* Serial.print("Lidar success rate "); */
    /* Serial.println(9.0 * lidar_success / lidar_bytes); */

    /* Serial.print("Lidar rate Hz "); */
    /* Serial.println(1000.0 * lidar_success / (millis() - start_millis)); */

    /* Serial.print("ISR rate Hz "); */
    /* Serial.println(1000.0 * isr_count / (millis() - start_millis)); */

    /* Serial.print("PWM duty "); */
    /* Serial.println(pwm_duty); */

    if (has_wifi) {
        telemeter();
    }

    nav_state.step();
    const float theta = -nav_state.theta;
    led_vector(cos(theta), sin(theta));

    const size_t uptime = millis() - start_millis;
    if (uptime > 4000) {
        set_motors(70, 70);
    } else if (uptime > 1000) {
        set_motors(73, 73);
    }
}
