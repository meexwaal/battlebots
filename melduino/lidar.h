#pragma once

/*
 * Receive from the lidar.
 * If commandType != 0, check that the command is the given type.
 */
bool lidarReceive(
    byte response[], const size_t responseLen,
    const byte commandType = 0
    ) {
    bool success = true;

    if (Serial1.available() == 512) {
        Serial.println("Serial1 likely overflowed");
    }

    size_t num_read = Serial1.readBytes(response, responseLen);
    if (num_read != responseLen) {
        Serial.println("Lidar didn't respond with all bytes");
        success = false;
    }

    if (response[0] != 0xAA || response[1] != 0x55) {
        Serial.println("Lidar response bad packet header");
        success = false;
    }

    if (commandType != 0 && response[2] != commandType) {
        Serial.println("Lidar response bad command type");
        success = false;
    }

    if (response[3] != responseLen - 5) {
        Serial.print("Lidar response bad length. Got ");
        Serial.print(response[3]);
        Serial.print(", expected ");
        Serial.println(responseLen - 5);
        success = false;
    }

    byte checksum = 0;
    for (size_t i = 0; i < responseLen - 1; i++) {
        checksum += response[i];
    }
    if (checksum != response[responseLen-1]) {
        Serial.print("Lidar checksum mismatch. Maybe serial buffer overflow? Got ");
        Serial.print(response[responseLen-1]);
        Serial.print(", expected ");
        Serial.println(checksum);
        for (size_t i = 0; i < responseLen; i++) {
            Serial.println(response[i], HEX);
        }
        success = false;
    }

    return success;
}

/*
 * Send a buffer, not including checksum. Receive a response.
 */
bool lidarSend(
    const byte buf[], const size_t bufLen,
    byte response[], const size_t responseLen,
    bool checkAvailable = true
    ) {
    while (checkAvailable && Serial1.available()) {
        Serial.println("Lidar sent unexpected data");
        Serial1.read();
    }
    byte checksum = 0;
    for (size_t i = 0; i < bufLen; i++) {
        checksum += buf[i];
    }
    Serial1.write(buf, bufLen);
    Serial1.write(checksum);

    if (responseLen == 0) {
        return true;
    }

    bool success = lidarReceive(response, responseLen, buf[2]);

    if (Serial1.available()) {
        Serial.println("Lidar has more data available");
    }

    return success;
}

bool lidarSelfTest() {
    Serial.println("Starting self test");
    byte cmd[4] = {0xAA, 0x55, 0x63, 0x00};
    byte response[39];
    bool success = lidarSend(cmd, sizeof(cmd), response, sizeof(response));
    if (response[4] != 0x01) {
        Serial.println("Lidar self-test failed");
        success = false;
    }
    return success;
}

/*
 * Set lidar output frequency to 1800 Hz
 */
bool lidarSetFreq() {
    Serial.println("Setting freq");
    byte cmd[5] = {0xAA, 0x55, 0x64, 0x01, 0x05};
    byte response[6];
    return lidarSend(cmd, sizeof(cmd), response, sizeof(response));
}

bool lidarSetFiltering(bool enable) {
    Serial.println("Setting filtering state");
    byte cmd[5] = {0xAA, 0x55, 0x65, 0x01, enable};
    byte response[6];
    return lidarSend(cmd, sizeof(cmd), response, sizeof(response));
}

bool lidarStartScan() {
    Serial.println("Starting scan");
    byte cmd[4] = {0xAA, 0x55, 0x60, 0x00};
    byte response[5];
    return lidarSend(cmd, sizeof(cmd), response, sizeof(response));
}

/*
 * Read a value from the lidar (mm) to out. Requires lidar to be in scan mode.
 */
bool lidarMeasure(uint16_t &out) {
    const size_t response_bytes = 9;
    uint16_t response[(response_bytes + 1) / 2];
    bool success = lidarReceive(reinterpret_cast<byte*>(response), response_bytes, 0x60);

    out = response[2];

    return success;
}

/*
 * Return lidar to idle by sending stop command and clearing the buffer
 */
bool lidarIdle() {
    // Send Stop Scan command and ignore check for available data before send
    byte cmd[4] = {0xAA, 0x55, 0x61, 0x00};
    bool success = lidarSend(cmd, sizeof(cmd), NULL, 0, false);
    delay(100);
    while (Serial1.available()) {
        Serial1.read();
    }
    delay(100);
    if (Serial1.available()) {
        Serial.println("Lidar did not idle cleanly");
        success = false;
    }
    return success;
}
