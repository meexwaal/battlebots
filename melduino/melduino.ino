/*
 * Notes on setup:
 *
 * Install libraries:
 *  - Adafruit ICM20X (2.0.7)
 *  - Adafruit LIS331 (1.0.6)
 *  - ArduinoGraphics (1.1.3)
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

    volatile size_t lidar_count = 0;
    size_t prev_lidar_count;
    size_t prev_millis = 0;
    size_t prev_lidar_idx = 0;

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
                if (!success) {
                    m = -1;
                }
                lidar_count++;

                dists[lidar_idx] = m;
                lidar_idx++;
                if (lidar_idx >= dists.size()) {
                    lidar_idx = 0;
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

    #define SPIN_CH1_PIN 2
    #define NS_CH3_PIN 3
    #define EW_CH4_PIN 8

    volatile long start_times[3] = {};
    volatile long current_times[3] = {};
    volatile long pulses[3] = {};
    volatile int pulse_widths[3] = {};

    template <int idx>
    void measure_pwm() {
        current_times[idx] = micros();

        if (current_times[idx] > start_times[idx]) {
          pulses[idx] = current_times[idx] - start_times[idx];
          start_times[idx] = current_times[idx];
        }

        if (pulses[idx] < 2000) {
          pulse_widths[idx] = pulses[idx];
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
        /* constexpr size_t n_points = 16; */
        /* constexpr size_t downsample = 2; */
        /* int next_idx = lidar_idx - (n_points - 1) * downsample; */
        int next_idx = lidar_idx - 1;
        if (next_idx < 0) {
            next_idx += dists.size();
        }
        telem_packet.lidar_mm[0] = dists[next_idx];
        /* for (size_t i = 0; i < n_points; i++) { */
        /*     telem_packet.lidar_mm[i] = dists[next_idx]; */
        /*     next_idx += downsample; */
        /*     if (next_idx >= dists.size()) { */
        /*         next_idx -= dists.size(); */
        /*     } */
        /* } */

        telem_packet.theta = nav_state.theta;
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
    pinMode(SPIN_CH1_PIN, INPUT_PULLUP);
    pinMode(NS_CH3_PIN, INPUT_PULLUP);
    pinMode(EW_CH4_PIN, INPUT_PULLUP);

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

    attachInterrupt(digitalPinToInterrupt(SPIN_CH1_PIN), measure_pwm<0>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(NS_CH3_PIN), measure_pwm<1>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EW_CH4_PIN), measure_pwm<2>, CHANGE);

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

    for (size_t i = 0; i < sizeof(telem_packet.lidar_mm) / sizeof(telem_packet.lidar_mm[0]); i++) {
        telem_packet.lidar_mm[i] = -1;
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

    Serial.print("SPIN_PIN:");
    Serial.print(pulse_widths[0]);
    Serial.print(",");
    Serial.print("NS_PIN:");
    Serial.print(pulse_widths[1]);
    Serial.print(",");
    Serial.print("EW_PIN:");
    Serial.print(pulse_widths[2]);
    Serial.println();

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

    /*
    Serial.print("Lidar success rate ");
    Serial.println(9.0 * lidar_success / lidar_bytes);

    Serial.print("Lidar rate Hz ");
    const size_t new_lidar_count = lidar_count;
    const size_t new_millis = millis();
    Serial.println(1000.0 * (new_lidar_count - prev_lidar_count) / (new_millis - prev_millis));
    prev_lidar_count = new_lidar_count;
    prev_millis = new_millis;

    const size_t new_lidar_idx = lidar_idx;
    Serial.print("Lidar samples ");
    Serial.println((new_lidar_idx - prev_lidar_idx) % dists.size());

    uint16_t min_mm = -1;
    uint16_t max_mm = 0;
    size_t scan_idx = prev_lidar_idx;
    while (scan_idx != new_lidar_idx) {
        min_mm = min(min_mm, dists[scan_idx]);
        max_mm = max(max_mm, dists[scan_idx]);

        scan_idx++;
        if (scan_idx >= dists.size()) {
            scan_idx = 0;
        }
    }
    Serial.print("Min ");
    Serial.print(min_mm / 1000.0);
    Serial.print(", max ");
    Serial.println(max_mm / 1000.0);
    prev_lidar_idx = new_lidar_idx;

    Serial.print("ISR rate Hz ");
    Serial.println(1000.0 * isr_count / (millis() - start_millis));
    */

    nav_state.step();
    const float theta = -nav_state.theta;
    led_vector(cos(theta), sin(theta));

    const size_t uptime = millis() - start_millis;

    if (uptime > 4000 && uptime < 30000) {
        const float spin_speed = map_float(pulse_widths[0], 1000, 2000, -0.08, 0.08);
        set_motors(spin_speed, spin_speed);
    } else {
        set_motors(0, 0);
    }

    if (has_wifi) {
        telemeter();
    }
}
