/* 
 * File:   imu.c
 * Author: Rich
 *
 * Created on August 31, 2013, 10:40 PM
 */

#include "imu.h"

#include "freeIMU.h"
#include "i2c_utils.h"
#include "l3gd20.h"
#include "lsm303dlhc.h"

#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <sys/prctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

sem_t sem_imu_trigger; // to trigger the next run of the loop
pthread_t _thread_imu;
pthread_mutex_t _imu_lock = PTHREAD_MUTEX_INITIALIZER;

imu_data _imu_working_reg = {.q[0] = 1};

void imuConfig(void) {

    i2cOpen();                                                  // TODO: catch error

    // configure accelerometer
    i2cSetAddress(ACCEL_I2C_ADDR);
    i2cWriteSingle(CTRL_REG1_A, 0b01100111);    // 200Hz update, XYZ enabled
    i2cWriteSingle(CTRL_REG2_A, 0x00);
    i2cWriteSingle(CTRL_REG3_A, 0x00);
    i2cWriteSingle(CTRL_REG4_A, 0b00111000);    // block update, little-endian, FS = 2G, high-res mode
    i2cWriteSingle(CTRL_REG5_A, 0x00);
    i2cWriteSingle(CTRL_REG6_A, 0x00);

    // configure magnetometer
    i2cSetAddress(MAG_I2C_ADDR);
    i2cWriteSingle(CRA_REG_M, 0b00011100);      // temp sensor off, 220Hz update rate
    i2cWriteSingle(CRB_REG_M, 0b00100000);      // set full-scale range, 1.3 gauss
    i2cWriteSingle(MR_REG_M, 0b00000000);       // continuous conversion mode

    // configure gyroscope
    i2cSetAddress(GYRO_I2C_ADDR);
    i2cWriteSingle(CTRL_REG1_G, 0b10001111);    // 380Hz update, 100Hz LPF cutoff, XYZ enabled
    //i2cWriteSingle(CTRL_REG2_G, 0b00001000);    // normal mode, 0.045 Hz HPF cutoff
    i2cWriteSingle(CTRL_REG4_G, 0b00000000);    // continuous update, little-endian, 250dps
    i2cWriteSingle(CTRL_REG5_G, 0b00000010);    // high-pass filter disabled, output LPFed

    //sem_init(&sem_imu_trigger, 0, 0);
    //pthread_create(&_thread_imu, NULL, imuUpdate, NULL);

}

void imuGetData(imu_data *pimu_data) {

    int imu_update_missed;
    static int imu_missed_counter;

    sem_getvalue(&sem_imu_trigger, &imu_update_missed);

    if (imu_update_missed) {
        imu_missed_counter++;
        printf("Missed IMU update %d\n", imu_missed_counter);
    } else {
        pthread_mutex_lock(&_imu_lock);
        memcpy(pimu_data, &_imu_working_reg, sizeof _imu_working_reg);
        pthread_mutex_unlock(&_imu_lock);

        sem_post(&sem_imu_trigger);
    }

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

    //printf("IMU data: \t%.3f\t%.3f\t%.3f", pimu_data->accel[0], pimu_data->accel[1], pimu_data->accel[2]);
    //printf("\t\t%.3f\t%.3f\t%.3f", pimu_data->mag[0], pimu_data->mag[1], pimu_data->mag[2]);
    //printf("\t\t%.3f\t%.3f\t%.3f\n", pimu_data->gyro[0], pimu_data->gyro[1], pimu_data->gyro[2]);

}

void *imuUpdate(void *arg) {
    
    //struct timeval t0, t1;
    //float elapsed;
    
    struct sched_param param = {.sched_priority = 15};
    sched_setscheduler(0, SCHED_RR, &param);
    
    prctl(PR_SET_NAME,"imu",0,0,0);
    
    while (1) {
        sem_wait(&sem_imu_trigger);
        
        //gettimeofday(&t0, NULL);
        
        pthread_mutex_lock(&_imu_lock);
        sensorRead(&_imu_working_reg);          // read 9-dof sensor
        
        //gettimeofday(&t1, NULL);
        //elapsed = (t1.tv_sec - t0.tv_sec)*1000 + (t1.tv_usec - t0.tv_usec) / 1000.0f;
        //printf("Sensor read time: %f ms\n", elapsed);
        
        AHRSupdate(&_imu_working_reg);          // call AHRS update routine
        getYawPitchRoll(&_imu_working_reg);     // return RPY representation
        pthread_mutex_unlock(&_imu_lock);

    }

}

//void sensorReadFast(imu_data *pimu_data) {
//    
//    uint8_t accel_data[6];
//    uint8_t mag_data[6];
//    uint8_t gyro_data[6];
//    
//    uint8_t accel_base = ACCEL_DATA_BASE_ADD;
//    uint8_t mag_base = MAG_DATA_BASE_ADD;
//    uint8_t gyro_base = GYRO_DATA_BASE_ADD;
//    
//    struct i2c_rdwr_ioctl_data i2c_data;
//    struct i2c_msg packet[6];
//    
//    i2c_data.msgs = packet;
//    i2c_data.nmsgs = 6;
//    
//    // 
//    packet[0].addr = ACCEL_I2C_ADDR;
//    packet[0].len = 1;
//    packet[0].buf = &accel_base;
//    
//    packet[1].addr = ACCEL_I2C_ADDR;
//    packet[1].flags = I2C_M_RD | I2C_M_NOSTART;
//    packet[1].len = 6;
//    packet[1].buf = accel_data;
//    
//    packet[2].addr = MAG_I2C_ADDR;
//    packet[2].len = 1;
//    packet[2].buf = &mag_base;
//    
//    packet[3].addr = MAG_I2C_ADDR;
//    packet[3].flags = I2C_M_RD | I2C_M_NOSTART;
//    packet[3].len = 6;
//    packet[3].buf = mag_data;
//    
//    packet[4].addr = GYRO_I2C_ADDR;
//    packet[4].len = 1;
//    packet[4].buf = &gyro_base;
//    
//    packet[5].addr = GYRO_I2C_ADDR;
//    packet[5].flags = I2C_M_RD | I2C_M_NOSTART;
//    packet[5].len = 6;
//    packet[5].buf = gyro_data;
//    
//    i2cBulkRW(&i2c_data);
//
////    // read and scale accelerometer values
////    i2cSetAddress(ACCEL_I2C_ADDR);
////    i2cReadBlock(ACCEL_DATA_BASE_ADD, raw.as_char, 6);
////    
////     // read and scale magnetometer values - returned as big-endien
////    i2cSetAddress(MAG_I2C_ADDR);
////    i2cReadBlock(MAG_DATA_BASE_ADD, raw.as_char, 6);
////    
////    // read and scale gyroscope values
////    i2cSetAddress(GYRO_I2C_ADDR);
////    i2cReadBlock(GYRO_DATA_BASE_ADD, raw.as_char, 6);
////    
////
//    pimu_data->accel[0] = *(int16_t*) accel_data * ACCEL_RES;
//    pimu_data->accel[1] = *(int16_t*) (accel_data+2) * ACCEL_RES;
//    pimu_data->accel[2] = *(int16_t*) (accel_data+4) * ACCEL_RES;
//
//    pimu_data->mag[0] = (int16_t) (mag_data[0] << 8 | mag_data[1]) / MAG_SEN;
//    pimu_data->mag[1] = (int16_t) (mag_data[4] << 8 | mag_data[5]) / MAG_SEN;
//    pimu_data->mag[2] = (int16_t) (mag_data[2] << 8 | mag_data[3]) / MAG_SENZ;
//
//    pimu_data->gyro[0] = *(int16_t*) gyro_data * GYRO_RES;
//    pimu_data->gyro[1] = *(int16_t*) (gyro_data+2) * GYRO_RES;
//    pimu_data->gyro[2] = *(int16_t*) (gyro_data+4) * GYRO_RES;
//    
//    printf("IMU data: \t%.3f\t%.3f\t%.3f", pimu_data->accel[0], pimu_data->accel[1], pimu_data->accel[2]);
//    printf("\t\t%.3f\t%.3f\t%.3f", pimu_data->mag[0], pimu_data->mag[1], pimu_data->mag[2]);
//    printf("\t\t%.3f\t%.3f\t%.3f\n", pimu_data->gyro[0], pimu_data->gyro[1], pimu_data->gyro[2]);
//}