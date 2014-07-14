/* 
 * File:   main.c
 * Author: Nathan Schomer
 *
 * Created on June 26, 2014
 * 
 * This program is written to run on a Beaglebone Black board
 * to display data pulled from an intertia measurement unit via I2c
 */

//#include "freeIMU.h"
//#include "i2c_utils.h"
//#include "l3gd20.h"
#include "lsm303dlhc.h"

void sensorRead(imu_data *pimu_data);

typedef struct {
        float accel[3];
        float mag[3];
        float gyro[3];
        float rpy[3];
        float q[4];
    } imu_data;

int IMU_test(int argc, char** argv) {
    
    imu_data curr_data;
    
    while(1) {
        sensorRead(&curr_data);
     
        printf("RPY: %.2f, %.2f, %.2f\n\n", curr_data.rpy[0], curr_data.rpy[1], curr_data.rpy[2]);
    }
    
    return(EXIT_SUCCESS);
}

void sensorRead(imu_data *pimu_data) {

    union {
        char as_char[6];
        int16_t as_int16[3];
    } raw;

    // read and scale accelerometer values
    i2cSetAddress(ACCEL_I2C_ADDR);
    i2cReadBlock(ACCEL_DATA_BASE_ADD, raw.as_char, 6);

    pimu_data->accel[0] = raw.as_int16[0] * ACCEL_RES;
    pimu_data->accel[1] = raw.as_int16[1] * ACCEL_RES;
    pimu_data->accel[2] = raw.as_int16[2] * ACCEL_RES;

    // read and scale magnetometer values - returned as big-endien
    i2cSetAddress(MAG_I2C_ADDR);
    i2cReadBlock(MAG_DATA_BASE_ADD, raw.as_char, 6);

    pimu_data->mag[0] = (int16_t) (raw.as_char[0] << 8 | raw.as_char[1]) / MAG_SEN;
    pimu_data->mag[1] = (int16_t) (raw.as_char[4] << 8 | raw.as_char[5]) / MAG_SEN;
    pimu_data->mag[2] = (int16_t) (raw.as_char[2] << 8 | raw.as_char[3]) / MAG_SENZ;

    // read and scale gyroscope values
    i2cSetAddress(GYRO_I2C_ADDR);
    i2cReadBlock(GYRO_DATA_BASE_ADD, raw.as_char, 6);

    pimu_data->gyro[0] = raw.as_int16[0] * GYRO_RES;
    pimu_data->gyro[1] = raw.as_int16[1] * GYRO_RES;
    pimu_data->gyro[2] = raw.as_int16[2] * GYRO_RES;
}