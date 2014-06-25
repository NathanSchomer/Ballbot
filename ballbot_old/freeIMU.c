/* 
 * File:   can_utils.h
 * Author: Rich
 *
 * Created on August 31, 2013, 10:40 PM
 */
// ported from freeIMU for arduino

#include "freeIMU.h"

#include "imu.h"
#include "robot.h"

#include <math.h> 

#define halfT           LOOP_PERIOD/2.0f                // half the sample period

void AHRSupdate(imu_data *imu) {
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    
    float ex, ey, ez;                                   // proportional error terms
    static float exInt, eyInt, ezInt;                   // integral error terms
    
    float accel[3], mag[3], gyro[3];                    // intermediate values

    // auxiliary variables to reduce number of repeated operations
    float q0q0 = imu->q[0] * imu->q[0];
    float q0q1 = imu->q[0] * imu->q[1];
    float q0q2 = imu->q[0] * imu->q[2];
    float q0q3 = imu->q[0] * imu->q[3];
    float q1q1 = imu->q[1] * imu->q[1];
    float q1q2 = imu->q[1] * imu->q[2];
    float q1q3 = imu->q[1] * imu->q[3];
    float q2q2 = imu->q[2] * imu->q[2];
    float q2q3 = imu->q[2] * imu->q[3];
    float q3q3 = imu->q[3] * imu->q[3];

    // normalize accelerometer
    norm = invSqrt(imu->accel[0] * imu->accel[0] + imu->accel[1] * imu->accel[1] + imu->accel[2] * imu->accel[2]);
    accel[0] = imu->accel[0] * norm;
    accel[1] = imu->accel[1] * norm;
    accel[2] = imu->accel[2] * norm;

    // normalize magnetometer
    norm = invSqrt(imu->mag[0] * imu->mag[0] + imu->mag[1] * imu->mag[1] + imu->mag[2] * imu->mag[2]);
    mag[0] = imu->mag[0] * norm;
    mag[1] = imu->mag[1] * norm;
    mag[2] = imu->mag[2] * norm;

    // compute reference direction of flux
    hx = 2 * mag[0]*(0.5 - q2q2 - q3q3)    + 2 * mag[1]*(q1q2 - q0q3)           + 2 * mag[2]*(q1q3 + q0q2);
    hy = 2 * mag[0]*(q1q2 + q0q3)          + 2 * mag[1]*(0.5 - q1q1 - q3q3)     + 2 * mag[2]*(q2q3 - q0q1);
    hz = 2 * mag[0]*(q1q3 - q0q2)          + 2 * mag[1]*(q2q3 + q0q1)           + 2 * mag[2]*(0.5 - q1q1 - q2q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;

    // estimated direction of gravity and flux (v and w)
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (accel[1] * vz - accel[2] * vy) + (mag[1] * wz - mag[2] * wy);
    ey = (accel[2] * vx - accel[0] * vz) + (mag[2] * wx - mag[0] * wz);
    ez = (accel[0] * vy - accel[1] * vx) + (mag[0] * wy - mag[1] * wx);

    // integral error scaled integral gain
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;

    // adjusted gyroscope measurements
    gyro[0] = imu->gyro[0] + Kp * ex + exInt;
    gyro[1] = imu->gyro[1] + Kp * ey + eyInt;
    gyro[2] = imu->gyro[2] + Kp * ez + ezInt;

    // integrate quaternion rate and normalize
    imu->q[0] = imu->q[0] + (-imu->q[1] * gyro[0] - imu->q[2] * gyro[1] - imu->q[3] * gyro[2]) * halfT;
    imu->q[1] = imu->q[1] + (imu->q[0] * gyro[0] + imu->q[2] * gyro[2] - imu->q[3] * gyro[1]) * halfT;
    imu->q[2] = imu->q[2] + (imu->q[0] * gyro[1] - imu->q[1] * gyro[2] + imu->q[3] * gyro[0]) * halfT;
    imu->q[3] = imu->q[3] + (imu->q[0] * gyro[2] + imu->q[1] * gyro[1] - imu->q[2] * gyro[0]) * halfT;

    // normalize quaternion
    norm = invSqrt(imu->q[0] * imu->q[0] + imu->q[1] * imu->q[1] + imu->q[2] * imu->q[2] + imu->q[3] * imu->q[3]);
    imu->q[0] = imu->q[0] * norm;
    imu->q[1] = imu->q[1] * norm;
    imu->q[2] = imu->q[2] * norm;
    imu->q[3] = imu->q[3] * norm;
}

// Returns the yaw, pitch, roll angles in degrees

void getYawPitchRoll(imu_data *imu) {
    
    
    // auxiliary variables to reduce number of repeated operations
    float q0q0 = imu->q[0] * imu->q[0];
    float q0q1 = imu->q[0] * imu->q[1];
    float q0q2 = imu->q[0] * imu->q[2];
    float q0q3 = imu->q[0] * imu->q[3];
    float q1q1 = imu->q[1] * imu->q[1];
    float q1q2 = imu->q[1] * imu->q[2];
    float q1q3 = imu->q[1] * imu->q[3];
    float q2q2 = imu->q[2] * imu->q[2];
    float q2q3 = imu->q[2] * imu->q[3];
    float q3q3 = imu->q[3] * imu->q[3];
    
    imu->rpy[0] = atan2(2*(q0q1+q2q3), q0q0-q1q1-q2q2+q3q3) * RAD_2_DEG;
    imu->rpy[1] = -asin(2*(q1q3-q0q2)) * RAD_2_DEG;
    imu->rpy[2] = atan2(2*(q0q3+q1q2), q0q0+q1q1-q2q2-q3q3) * RAD_2_DEG;
    
    //imu->rpy[0] = atan2(2 * imu->q[2] * imu->q[3] - 2 * imu->q[0] * imu->q[1], 2 * imu->q[0] * imu->q[0] + 2 * imu->q[3] * imu->q[3] - 1) * RAD_2_DEG;
    //imu->rpy[1] = -asin(2 * imu->q[1] * imu->q[3] + 2 * imu->q[0] * imu->q[2]) * RAD_2_DEG;
    //imu->rpy[2] = atan2(2 * imu->q[1] * imu->q[2] - 2 * imu->q[0] * imu->q[3], 2 * imu->q[0] * imu->q[0] + 2 * imu->q[1] * imu->q[1] - 1) * RAD_2_DEG;
    
}

float invSqrt(float number) {
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = *(long *) &y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *) &i;
    y = y * (f - (x * y * y));
    return y;
}
