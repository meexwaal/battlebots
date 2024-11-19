#pragma once

typedef struct {
    uint16_t lidar_mm[16];
    float theta;
    float gyro_w;
    float accel_w;
    uint8_t packet_count;
    uint8_t late_wakeup_count;
    uint8_t skipped_cycle_count;
} telem_packet_t;
