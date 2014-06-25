/* 
 * File:   imu.h
 * Author: Rich
 *
 * Created on August 31, 2013, 10:40 PM
 */

#ifndef IMU_H
#define	IMU_H

#ifdef	__cplusplus
extern "C" {
#endif

    typedef struct {
        float accel[3];
        float mag[3];
        float gyro[3];
        float rpy[3];
        float q[4];
    } imu_data;
    
    void imuConfig (void);
    void imuGetData(imu_data*);
    void sensorRead (imu_data*);
    void sensorReadFast(imu_data*);
    void *imuUpdate(void*);
    
#ifdef	__cplusplus
}
#endif

#endif	/* IMU_H */

