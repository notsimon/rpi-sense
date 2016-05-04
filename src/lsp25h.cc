#include "lsp25h.h"
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <cstdio>

#define DEV_PATH "/dev/i2c-1"

enum {
    DEV_ID = 0x5c,
    WHO_AM_I = 0x0f,
    RES_CONF = 0x10,
    CTRL_REG1 = 0x20,
    CTRL_REG1__PD = 0x80,
    CTRL_REG1__ODR_1HZ = 0x10,
    CTRL_REG1__ODR_12_5HZ = 0x30,
    CTRL_REG1__ODR_25HZ = 0x40,
    CTRL_REG1__BDU = 0x04,
    CTRL_REG2 = 0x21,
    CTRL_REG2__SW_RESET = 0x04,
    CTRL_REG2__FIFO_EN = 0x40,
    CTRL_REG2__AUTOZERO = 0x02,
    PRESS_OUT_XL = 0x28,
    PRESS_OUT_L = 0x29,
    PRESS_OUT_H = 0x2a,
    TEMP_OUT_L = 0x2b,
    TEMP_OUT_H = 0x2c,
    FIFO_CTRL = 0x2e,
};

Lsp25h::Lsp25h() {
    if ((fd_ = open(DEV_PATH, O_RDWR)) < 0) {
        perror("Unable to open i2c device");
        exit(1);
    }

    if (ioctl(fd_, I2C_SLAVE, DEV_ID) < 0) {
        perror("Unable to configure i2c slave device");
        close(fd_);
        exit(1);
    }

    if (i2c_smbus_read_byte_data(fd_, WHO_AM_I) != 0xBD) {
        printf("%s\n", "who_am_i error");
        close(fd_);
        exit(1);
    }

    // Power down the device
    i2c_smbus_write_byte_data(fd_, CTRL_REG1, 0x00);
    i2c_smbus_write_byte_data(fd_, CTRL_REG2, CTRL_REG2__SW_RESET); // sw reset
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    // Setup averaging
    //i2c_smbus_write_byte_data(fd_, RES_CONF, 0x0f); // max ADC HW average
    i2c_smbus_write_byte_data(fd_, FIFO_CTRL, 0xcf); // FIFO moving mean
    i2c_smbus_write_byte_data(fd_, CTRL_REG2, CTRL_REG2__FIFO_EN);

    // Turn on the pressure sensor at 25Hz ODR
    i2c_smbus_write_byte_data(fd_, CTRL_REG1, CTRL_REG1__PD|CTRL_REG1__ODR_25HZ);
}

Lsp25h::~Lsp25h() {
    i2c_smbus_write_byte_data(fd_, CTRL_REG1, 0x00);
    close(fd_);
}

float Lsp25h::read_pressure() const {
    // TODO handle endianness ;)
    int32_t p = i2c_smbus_read_byte_data(fd_, PRESS_OUT_H) << 16
              | i2c_smbus_read_byte_data(fd_, PRESS_OUT_L) << 8
              | i2c_smbus_read_byte_data(fd_, PRESS_OUT_XL);

    return p / 4096.0; // in hPa
}

float Lsp25h::read_temp() const {
    // TODO handle endianness ;)
    int16_t t = i2c_smbus_read_byte_data(fd_, TEMP_OUT_H) << 8
              | i2c_smbus_read_byte_data(fd_, TEMP_OUT_L);

    return 42.5 + t / 480.0; // in Celsius
}
