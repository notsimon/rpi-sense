#pragma once

#include <cstdio>
#include <cstdint>

class LedMatrix {
    int fd_;
    uint16_t* buffer_;

    static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
        r = static_cast<uint16_t>(r) * (1u << 5) / (UINT8_MAX + 1);
        g = static_cast<uint16_t>(g) * (1u << 6) / (UINT8_MAX + 1);
        b = static_cast<uint16_t>(b) * (1u << 5) / (UINT8_MAX + 1);
        return r << 11 | g << 5 | b;
    }

public:
    static const size_t Width = 8;
    static const size_t Height = 8;

    LedMatrix();
    ~LedMatrix();

    inline void set(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
        buffer_[x*Height + y] = rgb565(r, g, b);
    }

    inline void set(uint8_t r, uint8_t g, uint8_t b) {
        const uint16_t color = rgb565(r, g, b);
        for (size_t i = 0; i < Width * Height; i++)
            buffer_[i] = color;
    }
};
