#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include <WiFiS3.h>

#include "lidar.h"
#include "wifi.h"
#include "protocol.h"

ArduinoLEDMatrix matrix;

char packetBuffer[256]; //buffer to hold incoming packet

void setup() {
    Serial.begin(460800);
    //Serial1.begin(230400);
    Serial1.begin(460800); // 230400

    matrix.begin();

    wifiSetup();

    Serial.println("Hello");

    /*
    while (!lidarIdle()) {
        delay(1000);
    };

    if (!(
            lidarSelfTest() &&
            lidarSetFreq() &&
            lidarSetFiltering(false) &&
            lidarStartScan()
            )) {
        while (true);
    }

    Serial.println("Lidar ok");
    */
}

// constexpr unsigned long CYCLE_DT_US = 2000; // 500 Hz
constexpr unsigned long CYCLE_DT_US = 10000; // 100 Hz
// constexpr unsigned long CYCLE_DT_US = 20000; // 50 Hz
// constexpr unsigned long CYCLE_DT_US = 100000; // 10 Hz
// constexpr unsigned long CYCLE_DT_US = 200000; // 5 Hz
unsigned long next_wakeup = 0;

size_t cycle_count = 0;
size_t late_wakeup_count = 0;
size_t skipped_cycle_count = 0;

telem_packet_t telem_packet;

IPAddress dest_addr{192,168,4,2};

int count = 0;
void telemeter() {
    //telem_packet.lidar_mm[0] = 11;
    telem_packet.lidar_mm[7] = cycle_count;
    telem_packet.late_wakeup_count = late_wakeup_count;
    telem_packet.skipped_cycle_count = skipped_cycle_count;

    if (count == 0) {
        Udp.beginPacket(dest_addr, localPort);
    }
    if (count == 1) {
        Udp.write(reinterpret_cast<uint8_t *>(&telem_packet), sizeof(telem_packet));
    }
    if (count == 2) {
        Udp.endPacket();
    }

    count++;
    if (count >= 10) {
        count = 0;
    }
}

void sleep_until_cycle_start() {
    const unsigned long now = micros();
    long sleep_us = next_wakeup - now;
    if (sleep_us < 0) {
        late_wakeup_count++;
        //telem_packet.lidar_mm[count]++;

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

    Serial.println("Running a cycle");
    Serial.println(next_wakeup);
    Serial.println(late_wakeup_count);
    Serial.println(skipped_cycle_count);

    telemeter();
}

/*

  std::array<uint16_t, 1024> dists;
  uint32_t sum = 0;

  for (size_t i = 0; i < dists.size(); i++) {
  bool success = lidarMeasure(dists[i]);
  //Serial.println(dist);
  if (!success) {
  while (true);
  }

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
pinMode(13, OUTPUT);
digitalWrite(13, HIGH);
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
