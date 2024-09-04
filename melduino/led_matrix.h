#pragma once

#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;

/*
 * Pack a sparse array into a dense format.
 */
using sparse_frame_t = uint8_t[8][12];
struct Frame {
    static constexpr size_t num_words = 3;
    uint32_t f[num_words];

    constexpr Frame(const uint32_t a, const uint32_t b, const uint32_t c) :
        f{a, b, c}
    {}

    constexpr Frame(const uint32_t f_[num_words]) :
        f{0,0,0}
    {
        for (size_t i = 0; i < num_words; i++) {
            f[i] = f_[i];
        }
    }

    constexpr Frame(const sparse_frame_t &in) :
        f{0,0,0}
    {
        size_t out_row = 0;
        size_t out_col = 0;
        for (int row = 0; row < 8; row++) {
            for (int col = 0; col < 12; col++) {
                f[out_row] <<= 1;
                f[out_row] |= in[row][col];
                out_col++;
                if (out_col >= 32) {
                    out_row++;
                    out_col = 0;
                }
            }
        }
    }

    constexpr Frame& operator|=(const Frame& rhs)
    {
        for (size_t i = 0; i < num_words; i++) {
            this->f[i] |= rhs.f[i];
        }
        return *this;
    }
};

constexpr inline Frame operator|(const Frame& lhs, const Frame& rhs)
{
    Frame result = lhs;
    result |= rhs;
    return result;
}

namespace led_msg {
    constexpr Frame hi({
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
        { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0 },
        { 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    });

    constexpr Frame init({
        { 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    });

    constexpr Frame no({
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    });

    constexpr Frame wifi({
        { 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 },
        { 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 },
        { 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    });

    constexpr Frame lidar({
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    });

    constexpr Frame timer({
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    });

    constexpr Frame smile(LEDMATRIX_EMOJI_HAPPY);
    constexpr Frame empty(0, 0, 0);
}

void led_print(const Frame &msg) {
    matrix.loadFrame(msg.f);
}

void led_matrix_init() {
    matrix.begin();
    led_print(led_msg::hi);
    delay(5);
}
