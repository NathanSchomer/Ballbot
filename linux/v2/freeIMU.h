/* 
 * File:   can_utils.h
 * Author: Rich
 *
 * Created on August 31, 2013, 10:40 PM
 */
// ported from freeIMU for arduino

#ifndef FREEIMU_H
#define	FREEIMU_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include "imu.h"

    void AHRSupdate(imu_data*);
    void getYawPitchRoll(imu_data*);
    float invSqrt(float);

#ifdef	__cplusplus
}
#endif

#endif	/* FREEIMU_H */

