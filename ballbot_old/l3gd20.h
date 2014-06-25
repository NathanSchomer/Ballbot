/* 
 * File:   l3gd20.h
 * Author: Rich Primerano
 *
 * Created on September 2, 2013, 1:23 PM
 */

#ifndef L3GD20_H
#define	L3GD20_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "robot.h"

// Accelerometer defines
#define GYRO_FS_CODE            0               // full-scale: 0=250dps, 1=500dps, 2=2000dps
#define GYRO_I2C_ADDR           0x6b            // I2C address of device
#define WHO_AM_I_G              0x0f
#define CTRL_REG1_G             0x20            // control register 1
#define CTRL_REG2_G             0x21            // control register 2
#define CTRL_REG3_G             0x22            // control register 3
#define CTRL_REG4_G             0x23            // control register 4
#define CTRL_REG5_G             0x24            // control register 5
#define STATUS_REG_G            0x27            // status register
#define GYRO_DATA_BASE_ADD      0xa8            // base address of sensor registers


//Gyroscope sensitivity in (rad/sec/LSB)
#if GYRO_FS_CODE == 0
    #define GYRO_RES 0.00875f / 180.0f * PI
#elif GYRO_FS_CODE == 1
    #define GYRO_RES 0.01750f / 180.0f * PI
#elif GYRO_FS_CODE == 2
    #define GYRO_RES 0.07000f / 180.0f * PI
#endif
    

#ifdef	__cplusplus
}
#endif

#endif	/* L3GD20_H */

