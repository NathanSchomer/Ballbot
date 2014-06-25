/* 
 * File:   i2c_utils.h
 * Author: Rich
 *
 * Created on August 31, 2013, 10:39 PM
 */

#ifndef I2C_UTILS_H
#define	I2C_UTILS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
    
    void i2cOpen(void);
    void i2cClose(void);

    inline void i2cSetAddress(char);

    inline void i2cWriteSingle(char, char);
    inline void i2cWriteBlock(char, char*, char);

    inline char i2cReadSingle(char);
    inline void i2cReadBlock(char, char*, char);
    
    inline void i2cBulkRW(struct i2c_rdwr_ioctl_data*);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_UTILS_H */

