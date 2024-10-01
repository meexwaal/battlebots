#pragma once

namespace melty {
    volatile size_t lidar_success = 0;
    volatile size_t lidar_bytes = 0;

    /*
     * Receive from the lidar.
     * If commandType != 0, check that the command is the given type.
     */
    bool lidarReceive(
        byte response[], const size_t responseLen,
        const byte commandType = 0
        ) {
        bool success = true;

        if (responseLen < 3 || response == NULL) {
            return false;
        }

        if (Serial1.available() == 512) {
            log("Serial1 likely overflowed");
        }

        size_t num_read = Serial1.readBytes(response, responseLen);
        lidar_bytes += num_read;
        if (num_read != responseLen) {
            log("Lidar didn't respond with all bytes");
            success = false;
        }

        if (response[0] != 0xAA || response[1] != 0x55) {
            log("Lidar response bad packet header");
            success = false;
        }

        if (commandType != 0 && response[2] != commandType) {
            log("Lidar response bad command type");
            success = false;
        }

        if (response[3] != responseLen - 5) {
            log("Lidar response bad length. Got ");
            /* Serial.print(response[3]); */ // todo
            /* Serial.print(", expected "); */
            /* Serial.println(responseLen - 5); */
            success = false;
        }

        byte checksum = 0;
        for (size_t i = 0; i < responseLen - 1; i++) {
            checksum += response[i];
        }
        if (checksum != response[responseLen-1]) {
            log("Lidar checksum mismatch. Maybe serial buffer overflow? Got ");
            /* Serial.print(response[responseLen-1]); */ // todo
            /* Serial.print(", expected "); */
            /* Serial.println(checksum); */
            /* for (size_t i = 0; i < responseLen; i++) { */
            /*     Serial.println(response[i], HEX); */
            /* } */
            success = false;
        }

        return success;
    }

    /*
     * More robust receive from the lidar.
     */
    bool lidarReceiveRobust(
        byte response[], const size_t responseLen,
        const byte commandType
        ) {
        bool success = true;

        if (responseLen < 3 || response == NULL) {
            return false;
        }

        if (Serial1.available() == 512) {
            log("Serial1 likely overflowed");
        }

        // Find header
        while (true) {
            if (Serial1.available() < responseLen) {
                return false;
            }

            /*
             * Check if the next 2 bytes are the header.
             * Note: making use of boolean short-circuiting to avoid missing the
             * header in the case where the buffer is "<bad byte> 0xAA 0x55 ...".
             */
            // todo: if (Serial1.read() == 0xAA && Serial1.read() == 0x55) {
            if ((lidar_bytes++,Serial1.read()) == 0xAA && (lidar_bytes++,Serial1.read()) == 0x55) {
                break;
            } else {
                log("Warning: had to drop data"); // todo don't spam
            }
        }

        // Fill in the header of the response
        response[0] = 0xAA;
        response[1] = 0x55;
        byte *body_buf = response + 2;
        const size_t bodyLen = responseLen - 2;
        size_t num_read = Serial1.readBytes(body_buf, bodyLen);
        lidar_bytes += num_read;
        if (num_read != bodyLen) {
            log("Lidar didn't respond with all bytes");
            success = false;
        }

        if (body_buf[0] != commandType) {
            log("Lidar response bad command type");
            success = false;
        }

        if (body_buf[1] != responseLen - 5) {
            log("Lidar response bad length. Got ");
            /* Serial.print(body_buf[1]); */ // todo
            /* Serial.print(", expected "); */
            /* Serial.println(responseLen - 5); */
            success = false;
        }

        byte checksum = 0xAA + 0x55;
        for (size_t i = 0; i < bodyLen - 1; i++) {
            checksum += body_buf[i];
        }
        if (checksum != body_buf[bodyLen - 1]) {
            log("Lidar checksum mismatch. Maybe serial buffer overflow? Got ");
            /* Serial.print(body_buf[bodyLen - 1]); */ // todo
            /* Serial.print(", expected "); */
            /* Serial.println(checksum); */
            /* for (size_t i = 0; i < bodyLen; i++) { */
            /*     Serial.println(body_buf[i], HEX); */
            /* } */
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
        byte cmd[5] = {0xAA, 0x55, 0x64, 0x01, 0x02};
        /*
         *            0x00  0x01  0x02  0x03   0x04   0x05
         * Data sheet 10Hz 100Hz 200Hz 500Hz 1000Hz 1800Hz
         * Measured   50Hz 400Hz 800Hz 2000 ~3950Hz 2200Hz
         */
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
        bool success = lidarReceiveRobust(reinterpret_cast<byte*>(response), response_bytes, 0x60);

        out = response[2];

        if (success) {
            lidar_success++;
        }

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

    bool lidarFactoryReset() {
        Serial.println("Lidar factory reset");
        byte cmd[4] = {0xAA, 0x55, 0x68, 0x00};
        byte response[5];
        return lidarSend(cmd, sizeof(cmd), response, sizeof(response));
    }
}
