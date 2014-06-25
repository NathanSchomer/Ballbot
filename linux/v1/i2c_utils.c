/* 
 * File:   i2c_utils.c
 * Author: Rich
 *
 * Created on August 31, 2013, 10:39 PM
 */

#include "i2c_utils.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <unistd.h>

int _i2c_handle;

void i2cOpen(void) {
    _i2c_handle = open("/dev/i2c-1", O_RDWR);
    if (_i2c_handle < 0) {
        perror("i2cOpen error");
    }
}

void i2cClose(void) {
    close(_i2c_handle);
}

inline void i2cSetAddress(char address) {
    if (ioctl(_i2c_handle, I2C_SLAVE, address) < 0) {
        perror("i2cSetAddress error");
    }
}

inline void i2cBulkRW(struct i2c_rdwr_ioctl_data* i2c_data) {
    if (ioctl(_i2c_handle, I2C_RDWR, i2c_data) < 0) {
        perror("i2cBulkRW error");
    }
}

inline void i2cWriteSingle(char address, char data) {

    char buffer[2];
    buffer[0] = address;
    buffer[1] = data;

    if (write(_i2c_handle, buffer, 2) != 2) {
        perror("i2cWriteSingle error");
    }
}

inline void i2cWriteBlock(char address, char *data, char len) {

}

inline char i2cReadSingle(char address) {

}

void i2cReadBlock(char address, char *data, char len) {

    if (write(_i2c_handle, &address, 1) != 1) {
        perror("i2cReadBlock error (setting address)");
    }
    if (read(_i2c_handle, data, len) != len) {
        perror("i2cReadBlock error (reading data)");
    }

}