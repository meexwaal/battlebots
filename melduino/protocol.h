#pragma once

typedef struct {
    uint16_t lidar_mm[8];
    uint8_t late_wakeup_count;
    uint8_t skipped_cycle_count;
} telem_packet_t;
