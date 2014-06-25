/* 
 * File:   lsm303dlhc.h
 * Author: Rich Primerano
 *
 * Created on September 2, 2013, 12:24 PM
 */

#ifndef LSM303DLHC_H
#define	LSM303DLHC_H

#ifdef	__cplusplus
extern "C" {
#endif

// Accelerometer defines
#define ACCEL_FS_CODE           0               // full-scale: 0=2g, 1=4g, 2=8g, 3=16g
#define ACCEL_I2C_ADDR          0x19            // I2C address of device
#define CTRL_REG1_A             0x20            // control register 1
#define CTRL_REG2_A             0x21            // control register 2
#define CTRL_REG3_A             0x22            // control register 3
#define CTRL_REG4_A             0x23            // control register 4
#define CTRL_REG5_A             0x24            // control register 5
#define CTRL_REG6_A             0x25            // control register 6
#define STATUS_REG_A            0x27            // status register
#define ACCEL_DATA_BASE_ADD     0xa8            // base address of sensor registers
    
// Magnetometer defines
#define MAG_FS_CODE             1
#define MAG_I2C_ADDR            0x1e
#define CRA_REG_M               0x00
#define CRB_REG_M               0x01
#define MR_REG_M                0x02
#define MAG_DATA_BASE_ADD       0x03

    
    
//Accelerometer sensitivity in (g/LSB)
//Note: the 12-bit reading is left aligned requiring an additional divide-by-16
#if ACCEL_FS_CODE == 0                  // 2g full scale
    #define ACCEL_RES           0.001f/16
#elif ACCEL_FS_CODE == 1                // 4g full scale
    #define ACCEL_RES           0.002f/16
#elif ACCEL_FS_CODE == 2                // 8g full scale
    #define ACCEL_RES           0.004f/16
#elif ACCEL_FS_CODE == 3                // 16g full scale
    #define ACCEL_RES           0.012f/16
#endif

//Magnetometer full scale (guass) and sensitivity (LSB/guass)
#if MAG_FS_CODE == 1 
    #define MAG_FS              1.3f
    #define MAG_SEN             1100.0f
    #define MAG_SENZ            980.0f
#elif MAG_FS_CODE== 2
    #define MAG_FS              1.9f
    #define MAG_SEN             855
    #define MAG_SENZ            760
#elif MAG_FS_CODE == 3
    #define MAG_FS              2.5f
    #define MAG_SEN             670.0f
    #define MAG_SENZ            6000.f
#elif MAG_FS_CODE == 4
    #define MAG_FS              4.0f
    #define MAG_SEN             450.0f
    #define MAG_SENZ            400.0f
#elif MAG_FS_CODE == 5
    #define MAG_FS              4.7f
    #define MAG_SEN             400.0f
    #define MAG_SENZ            355.0f
#elif MAG_FS_CODE == 6
    #define MAG_FS              5.6f
    #define MAG_SEN             330.0f
    #define MAG_SENZ            295.0f
#elif MAG_FS_CODE == 7
    #define MAG_FS              8.1f
    #define MAG_SEN             230.0f
    #define MAG_SENZ            205.0f
#endif



#ifdef	__cplusplus
}
#endif

#endif	/* LSM303DLHC_H */

