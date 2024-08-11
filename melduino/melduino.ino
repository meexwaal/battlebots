#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include <WiFiS3.h>
#include "FspTimer.h"

char * volatile debug_str = "ok";
void log(char *str) {
    // Serial.println(str);
    debug_str = str;
}

#include "lidar.h"
#include "wifi.h"
#include "protocol.h"

ArduinoLEDMatrix matrix;
FspTimer isr_timer;

volatile size_t isr_count = 0;

volatile size_t last_start = 0;
volatile size_t max_period = 0;
volatile size_t max_duration = 0;

volatile size_t lidar_idx = 0;
std::array<volatile uint16_t, 1024> dists;

static constexpr bool lidar_en = true;

template <typename unused_t>
void isr(unused_t unused) {
    const size_t start = micros();
    const size_t period = start - last_start;
    last_start = start;
    if (period > max_period) {
        max_period = period;
    }

    // Blink the light to show that ISR is running
    digitalWrite(13, (isr_count & (1 << 6)) == 0);

    isr_count++;
    // delayMicroseconds(100);

    int max_reads = 10;
    if (lidar_en) {
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

void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    Serial.begin(460800);
    Serial1.begin(460800);

    matrix.begin();

    wifiSetup();

    if (lidar_en) {
        while (!lidarIdle()) {
            Serial.println("Lidar failed to idle, trying again");
            delay(1000);
        };

        if (!(
                lidarSelfTest() &&
                lidarSetFreq() &&
                lidarSetFiltering(false) &&
                lidarStartScan()
                )) {
            Serial.println("Lidar failed to init, giving up");
            while (true);
        }

        Serial.println("Lidar ok");
    }

    if (!beginTimer(500)) {
        Serial.println("Timer failed init");
    };
}

// constexpr unsigned long CYCLE_DT_US = 2000; // 500 Hz
// constexpr unsigned long CYCLE_DT_US = 10000; // 100 Hz
// constexpr unsigned long CYCLE_DT_US = 20000; // 50 Hz
constexpr unsigned long CYCLE_DT_US = 50000; // 20 Hz
// constexpr unsigned long CYCLE_DT_US = 100000; // 10 Hz
// constexpr unsigned long CYCLE_DT_US = 200000; // 5 Hz
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

        const size_t skipped_cycles = (-sleep_us) / CYCLE_DT_US;
        skipped_cycle_count += skipped_cycles;

        sleep_us = 0;
        next_wakeup += skipped_cycles * CYCLE_DT_US;
    }
    delayMicroseconds(sleep_us);
    next_wakeup += CYCLE_DT_US;
}

void loop() {
    cycle_count++;
    sleep_until_cycle_start();

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
        /* Serial.println(telem_packet.lidar_mm[i]); */
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

    Serial.println(debug_str);
    debug_str = "ok";

    Serial.print("Lidar rate ");
    Serial.println(9.0 * lidar_success / lidar_bytes);

    // telemeter();
}

/*


  matrix.beginDraw();
  matrix.stroke(0xFFFFFFFF);
  char text[5];
  snprintf(text, sizeof(text), "%f", std);
  matrix.textFont(Font_4x6);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(text);
  matrix.endText();
  matrix.endDraw();
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
 * TODO: follow up on why Serial.print doesn't work in the interrupt.
 * Useful reading:
 * https://forum.arduino.cc/t/the-renesas-users-manual-and-fsp-the-deep-magic-for-r4/1179100/4
 * https://www.renesas.com/us/en/document/mah/renesas-ra4m1-group-users-manual-hardware?r=1054146
 * https://forum.arduino.cc/t/uno-r4-attachinterrupt/1148214/9
 * https://www.pschatzmann.ch/home/2023/07/01/under-the-hood-arduino-uno-r4-timers/
 * https://github.com/arduino/ArduinoCore-renesas/blob/149f78b6490ccbafeb420f68919c381a5bdb6e21/cores/arduino/FspTimer.cpp
 *
 * Some observations:
 *
 * noInterrupts() in ISR will prevent it from firing again
 * but interrupts() will not let it be re-entered
 * delay() works in an isr, but it hangs if I call noInterrupts() first
 * takeaway:
 *   noInterrupts() will block interrupts from running, but there's something else Also blocking?
 *   like an "I'm in an ISR" register
 *   explains why print fails and why function can't reenter
 *   doesn't totally explain why delay works. maybe it's a higher priority?
 *
 *
 * wifi send interruptability:
 *  at 500 Hz interrupts, if ISR delays for 200 us, seeing some long delays between packets
 *  even with low wifi period, 5 Hz
 *  delay is ~10 s. I wonder if it's hitting some retry logic
 *  not dropping packets though
 *  at 100 us delay in the ISR (500 Hz), wifi is ok
 */
