/*
 * Notes on setup:
 *
 * Install libraries:
 *  - Adafruit ICM20X (2.0.7)
 *  - Adafruit LIS331 (1.0.6)
 *  - ArduinoGraphics (1.1.3)
 *
 * I also modified:
 * ~/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/cores/arduino/IRQManager.cpp
 *   Set UART_SCI_PRIORITY = 6, less than TIMER_PRIORITY, so that Serial can interrupt our
 *   timer-driver fast loop.
 *   Set I2C_MASTER/SLAVE_PRIORITY, less than UART or TIMER, so that the I2C gyro doesn't
 *   crash.
 * ~/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/platform.txt
 *   Set compiler.optimization_flags=-O2 and compiler.optimization_flags.release=-O2
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

    struct {
        volatile bool has_lidar = false;
        bool has_wifi = false;

        volatile size_t isr_count = 0;

        volatile size_t last_start = 0;
        volatile size_t max_period = 0;
        volatile size_t max_duration = 0;

        volatile size_t write_lidar_idx = 0;
        volatile size_t read_lidar_idx = 0;

        volatile float px = 0;
        volatile float py = 0;

        volatile float spin_speed = 0;
        volatile float east_speed = 0;
        volatile float north_speed = 0;
        volatile float total_speed = 0;
        volatile bool motor_en = false;
    } state;

    // Position of the LED in the bot frame
    constexpr float led_x = -9.8 / 10.51; // std::sqrt(9.8 * 9.8 + 3.8 * 3.8);
    constexpr float led_y = -3.8 / 10.51;

    std::array<volatile uint16_t, 1024> dists;

    size_t start_millis = 0;

    volatile size_t lidar_count = 0;
    size_t prev_lidar_count;
    size_t prev_millis = 0;
    size_t prev_lidar_idx = 0;

    template <typename unused_t>
    void isr(unused_t unused) {
        const size_t start = micros();
        const size_t period = start - state.last_start;
        state.last_start = start;
        if (period > state.max_period) {
            state.max_period = period;
        }

        // Blink the light to show that ISR is running
        digitalWrite(pins::status_led, (state.isr_count & (1 << 6)) == 0);

        state.isr_count++;
        // delayMicroseconds(100);

        nav_state.step_theta();
        const float theta = nav_state.theta;
        const float px = cos(theta);
        const float py = sin(theta);
        state.px = px;
        state.py = py;

        constexpr float threshold = 0.996194698; // cos(5 deg)

        // Rotate the LED vector into the arena frame (see bot_to_arena in README.md)
        const float led_x_arena = dot(led_x, led_y, px, -py);
        const float led_y_arena = dot(led_x, led_y, py, px);

        // Light one LED color when the LED points north, the other when the LED
        // points towards the commanded direction.
        const float led_north_dot = led_y_arena; // = dot(led_x_arena, led_y_arena, 0, 1)
        const float led_cmd_dot =
            dot(led_x_arena, led_y_arena, state.east_speed, state.north_speed);

        digitalWrite(pins::led_green, led_north_dot > threshold);
        digitalWrite(pins::led_blue,
                     state.total_speed > 0.05 &&
                     led_cmd_dot > threshold * state.total_speed);

        const float motor_dot = dot(px, py, state.east_speed, state.north_speed);

        float left_bias = state.total_speed * 0.5;
        if (motor_dot > 0) {
            left_bias *= -1;
        }

        if (state.motor_en) {
            set_motors(state.spin_speed * (1 + left_bias), state.spin_speed * (1 - left_bias));
        } else {
            set_motors(0, 0);
        }

        int max_reads = 5;
        if (state.has_lidar) {
            while (max_reads > 0 && Serial1.available() >= 9) {
                max_reads--;
                uint16_t m = -1;
                const bool success = lidarMeasure(m);
                if (!success) {
                    m = -1;
                }
                lidar_count++;

                dists[state.write_lidar_idx] = m;
                state.write_lidar_idx++;
                if (state.write_lidar_idx >= dists.size()) {
                    state.write_lidar_idx = 0;
                }
            }
        }

        const size_t duration = micros() - start;
        if (duration > state.max_duration) {
            state.max_duration = duration;
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

    // PWM measurements for each receiver channel
    volatile long start_times[3] = {};
    volatile long current_times[3] = {};
    volatile long pulses[3] = {};
    volatile int pulse_widths[3] = {};

    // After this much time without a pulse, consider the channel invalid
    // TODO: log when this happens
    constexpr long pwm_stale_time_us = 40000; // 25 Hz, some margin on the expected 50 Hz

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

    IPAddress dest_addr{192,168,4,255};

    int packet_count = 0;
    void telemeter() {
        constexpr size_t n_points = 16;
        /* telem_packet.lidar_mm[0] = dists[next_idx]; */

        bool no_more_samples = false;
        for (size_t i = 0; i < n_points; i++) {
            if (no_more_samples || state.read_lidar_idx == state.write_lidar_idx) {
                no_more_samples = true;
                telem_packet.lidar_mm[i] = -2;
            } else {
                telem_packet.lidar_mm[i] = dists[state.read_lidar_idx];
                state.read_lidar_idx++;
                if (state.read_lidar_idx >= dists.size()) {
                    state.read_lidar_idx -= dists.size();
                }
            }
        }

        telem_packet.theta = nav_state.theta;
        telem_packet.gyro_w = nav_state.gyro_w;
        telem_packet.accel_w = nav_state.accel_w;
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
    pinMode(pins::ch1_spin, INPUT_PULLUP);
    pinMode(pins::ch3_north_south, INPUT_PULLUP);
    pinMode(pins::ch4_east_west, INPUT_PULLUP);

    pinMode(pins::led_blue, OUTPUT);
    pinMode(pins::led_green, OUTPUT);
    pinMode(pins::status_led, OUTPUT);
    digitalWrite(pins::status_led, HIGH);

    Serial.begin(460800);
    Serial.println("Starting init");

    led_matrix_init();

    // WiFi
    led_print(led_msg::init | led_msg::wifi);
    state.has_wifi = wifiSetup();
    if (!state.has_wifi) {
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

    attachInterrupt(digitalPinToInterrupt(pins::ch1_spin), measure_pwm<0>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins::ch3_north_south), measure_pwm<1>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins::ch4_east_west), measure_pwm<2>, CHANGE);

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
        state.has_lidar = false;
    } else {
        state.has_lidar = true;
    }

    for (size_t i = 0; i < sizeof(telem_packet.lidar_mm) / sizeof(telem_packet.lidar_mm[0]); i++) {
        telem_packet.lidar_mm[i] = -1;
    }


    // Nav sensors, IMU
    led_print(led_msg::init | led_msg::nav);
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
    if (!state.has_wifi) {
        end_msg |= led_msg::no | led_msg::wifi;
    }
    if (!has_good_motors) {
        end_msg |= led_msg::no | led_msg::motor;
    }
    if (!state.has_lidar) {
        end_msg |= led_msg::no | led_msg::lidar;
    }
    if (!has_nav) {
        end_msg |= led_msg::no | led_msg::nav;
    }
    led_print(end_msg);

    start_millis = millis();
}

void loop() {

    cycle_count++;
    sleep_until_cycle_start();

    /* Serial.print("SPIN_PIN:"); */
    /* Serial.print(pulse_widths[0]); */
    /* Serial.print(","); */
    /* Serial.print("NS_PIN:"); */
    /* Serial.print(pulse_widths[1]); */
    /* Serial.print(","); */
    /* Serial.print("EW_PIN:"); */
    /* Serial.print(pulse_widths[2]); */
    /* Serial.println(); */


    /* Serial.println(""); */
    /* Serial.print("Cycle #        : "); */
    /* Serial.println(cycle_count); */
    /* Serial.print("Late wakeups   : "); */
    /* Serial.println(late_wakeup_count); */
    /* Serial.print("Skipped cycles : "); */
    /* Serial.println(skipped_cycle_count); */

    /* Serial.print("Max ISR period : "); */
    /* Serial.println(state.max_period); */
    /* state.max_period = 0; */
    /* Serial.print("Max ISR dur.   : "); */
    /* Serial.println(state.max_duration); */
    /* state.max_duration = 0; */

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
    Serial.println(1000.0 * state.isr_count / (millis() - start_millis));
    */

    nav_state.step_sensors();

    // Draw a vector that points east in the arena, which is the vector at
    // -theta in the bot frame.
    // v = (cos(-theta), sin(-theta)) = (cos(theta), -sin(theta)) = (px, -py)
    led_vector(state.px, -state.py);

    const size_t uptime = millis() - start_millis;
    state.motor_en = uptime < 60000;

    const long now_us = micros();

    constexpr float full_speed = 0.08;
    /* constexpr float full_speed = 1.0f; */
    float spin_speed = map_float(pulse_widths[0], 1000, 2000, -full_speed, full_speed);
    float east_speed = map_float(pulse_widths[2], 1000, 2000, 1, -1);
    float north_speed = map_float(pulse_widths[1], 1000, 2000, 1, -1);

    bool stale = false;
    static bool prev_stale = true;
    // TODO: when receiver stops getting signal, it sends 1500.
    // This should use the same logic, so a failure anywhere in the chain has the same response.
    // Also, add sanity check on the value, because even invalid values persist for short times.

    if (now_us - current_times[0] > pwm_stale_time_us) {
        spin_speed = 0;
        stale = true;
    }
    if (now_us - current_times[1] > pwm_stale_time_us) {
        north_speed = 0;
        stale = true;
    }
    if (now_us - current_times[2] > pwm_stale_time_us) {
        east_speed = 0;
        stale = true;
    }
    if (stale && !prev_stale) {
        Serial.println("PWM went stale");
    }
    prev_stale = stale;

    state.spin_speed = spin_speed;
    state.total_speed = std::sqrt(east_speed * east_speed + north_speed * north_speed);
    state.east_speed = east_speed;
    state.north_speed = north_speed;

    if (state.has_wifi) {
        telemeter();
    }

    // TODO: atomic state updates, because fast loop can interrupt slow loop
}
