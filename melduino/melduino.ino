/*
 * Note: I had to modify
 * ~/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/cores/arduino/IRQManager.cpp
 * to set UART_SCI_PRIORITY = 6, less than TIMER_PRIORITY, so that Serial can interrupt our
 * timer-driver fast loop.
 * TODO: set up a timer interrupt with lower priority in a less hacky way. Some notes below.
 */

#include "ArduinoGraphics.h"
#include <WiFiS3.h>
#include "FspTimer.h"
#include "pwm.h"

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

#include "led_matrix.h"
#include "lidar.h"
#include "wifi.h"
#include "protocol.h"

FspTimer isr_timer;
PwmOut pwm(D2);

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
    digitalWrite(LED_BUILTIN, (isr_count & (1 << 6)) == 0);

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

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(460800);
    Serial.println("Starting init");

    led_matrix_init();

    // WiFi
    led_print(led_msg::init | led_msg::wifi);
    has_wifi = wifiSetup();
    if (!has_wifi) {
        Serial.println("WiFi failed to init");
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
    if (!has_lidar) {
        end_msg |= led_msg::no | led_msg::lidar;
    }
    led_print(end_msg);

    start_millis = millis();
}

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


    Serial.println("");
    Serial.print("Cycle #        : ");
    Serial.println(cycle_count);
    Serial.print("Late wakeups   : ");
    Serial.println(late_wakeup_count);
    Serial.print("Skipped cycles : ");
    Serial.println(skipped_cycle_count);

    Serial.print("Max ISR period : ");
    Serial.println(max_period);
    max_period = 0;
    Serial.print("Max ISR dur.   : ");
    Serial.println(max_duration);
    max_duration = 0;

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

    Serial.print("mean ");
    Serial.println(mean);
    Serial.print("std  ");
    Serial.println(std);

    Serial.print("lidar_idx ");
    /* Serial.println(lidar_idx); */
    /* for (int i = 0; i < lidar_idx; i++) { */
    /*     Serial.println(dists[i]); */
    /* } */

    printlog();

    Serial.print("Lidar success rate ");
    Serial.println(9.0 * lidar_success / lidar_bytes);

    Serial.print("Lidar rate Hz ");
    Serial.println(1000.0 * lidar_success / (millis() - start_millis));

    Serial.print("ISR rate Hz ");
    Serial.println(1000.0 * isr_count / (millis() - start_millis));

    Serial.print("PWM duty ");
    Serial.println(pwm_duty);

    if (has_wifi) {
        telemeter();
    }
}

/* Initialize and drive a motor
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
int val;    // variable to read the value from the analog pin

void setup() {
Serial.begin(115200);
Serial.println("Enter to start");
while (!Serial.available());
while (Serial.available()) Serial.read();

Serial.println("hi");
myservo.attach(9);  // attaches the servo on pin 9 to the servo object
myservo.write(110);
while (!Serial.available());
while (Serial.available()) Serial.read();
Serial.println("lo");
myservo.write(70);
while (!Serial.available());
while (Serial.available()) Serial.read();
Serial.println("start");
}

void loop() {
val = Serial.parseInt();
Serial.println(val);
if (70 <= val && val <= 110) {
Serial.println("ok");
myservo.write(val);
}
}
*/

/*
 * TODO: follow up on how to properly set the priority of the fast loop interrupt.
 * Originally why Serial.print doesn't work in the interrupt.
 * Useful reading:
 * https://forum.arduino.cc/t/the-renesas-users-manual-and-fsp-the-deep-magic-for-r4/1179100/4
 * https://www.renesas.com/us/en/document/mah/renesas-ra4m1-group-users-manual-hardware?r=1054146
 * https://forum.arduino.cc/t/uno-r4-attachinterrupt/1148214/9
 * https://www.pschatzmann.ch/home/2023/07/01/under-the-hood-arduino-uno-r4-timers/
 * https://github.com/arduino/ArduinoCore-renesas/blob/149f78b6490ccbafeb420f68919c381a5bdb6e21/cores/arduino/FspTimer.cpp
 * https://forum.arduino.cc/t/ra4m1-interrupts/1179222
 *
 * This answer clarified things for me, it's about priority
 * https://stackoverflow.com/questions/21637541/how-to-re-enable-interrupts-from-within-an-interrupt-handler-on-arm-cortex-m3
 * How do I set the priority?
 * Aha! This post solves exactly my problem, but in a hacky way:
 * https://forum.arduino.cc/t/serial-print-from-isr-will-hang/1145292/5
 *
 * Some thoughts on faster serial comms:
 * https://github.com/arduino/ArduinoCore-renesas/issues/44
 */
