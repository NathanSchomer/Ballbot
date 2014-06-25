/* 
 * File:   main.c
 * Author: Rich Primerano
 * Modified By: Nathan Schomer
 *
 * Created on June 25, 2014
 */

#include "freeIMU.h"
#include "i2c_utils.h"
#include "l3gd20.h"
#include "lsm303dlhc.h"


void forceIK(float*, const float*);
void velocityFK(float*, const float*);
void velocityIK(float*, const float*);
void sensorRead(imu_data *pimu_data);

int main2(int argc, char** argv) { // ??argc and argv??
    
    imu_data heading;                           // raw and processed IMU data
    float ball_vel[3], motor_omega[3];
    state_vector s[2];
    
    while(1) {
        sensorRead(&heading); //why is parameter needed?
        
        heading.rpy[0] -= con.x_trim;
        heading.rpy[1] -= con.y_trim;
        
        for (i = 0; i < 2; i++){
            s[i].angle = heading.rpy[i];

            s[i].angle_int += heading.rpy[i] * LOOP_PERIOD;
            if (s[i].angle_int > con.int_sat)
                s[i].angle_int = con.int_sat;
            else if (s[i].angle_int < -con.int_sat)
                s[i].angle_int = -con.int_sat;

            s[i].angle_dot = heading.gyro[i] * 180 / PI;

            s[i].pos += (ball_vel[i] * LOOP_PERIOD);
            s[i].pos_dot = ball_vel[i];
        }
        
        s[2].angle = heading.rpy[2];
        s[2].angle_dot = heading.gyro[2] * 180 / PI;
        
        // compute control force
        for (i = 0; i < 2; i++){
            ball_vel[i] = con.p * s[i].angle
                + con.i * s[i].angle_int
                + con.d * s[i].angle_dot
                + con.p_pos * s[i].pos
                + con.d_pos * s[i].pos_dot;
        }
        
        ball_vel[2] = 0;
        
        velocityIK(motor_omega, ball_vel);
        motSetVel(motor_omega);
        
        printf("RPY: %.2f, %.2f, %.2f\n\n", heading.rpy[0], heading.rpy[1], heading.rpy[2]);
    }
    
    return(EXIT_SUCCESS);
}

//WHAT IS THE 3rd LOOP?????

/* Map X,Y forces and theta torque of ball into motor torques. 
 * - ball_force[0]      =>      force in X direction (Newtons)
 * - ball_force[1]      =>      force in Y direction (Newtons)
 * - ball_force[2]      =>      torque about Z axis (Newton-meters)
 * - motor_torque       =>      motor torque (Newton-meters)*/
void forceIK(float* motor_torque, const float* ball_force) {
    float ball_tau[3];
    
    // Translate ball's X and Y tangent forces into torques (Z is already a torque)
    // Scale down by gear ratio to represent required motor torque
    ball_tau[0] = ball_force[1] / GEAR_RATIO * (D_BALL/2);      // torque in X dir a result of force in Y
    ball_tau[1] = ball_force[0] / GEAR_RATIO * (D_BALL/2);      // torque in Y dir a result of force in X
    ball_tau[2] = ball_force[2] / GEAR_RATIO;                   // this is already a torque

    // Transform torque from Cartesian frame to motor axis aligned frame
    motor_torque[0] =   0.9428 * ball_tau[0]                           - 0.4714 * ball_tau[2];
    motor_torque[1] =  -0.4714 * ball_tau[0]   + 0.8165 * ball_tau[1]  - 0.4714 * ball_tau[2];
    motor_torque[2] =  -0.4714 * ball_tau[0]   - 0.8165 * ball_tau[1]  - 0.4714 * ball_tau[2];
    
}

void velocityFK(float* ball_vel, const float* motor_omega) {
    float ball_omega[3];
    
    // Transform from motor axis coordinates to ball's Cartesian coordinates
    ball_omega[0] =   0.9428 * motor_omega[0]  - 0.4714 * motor_omega[1]  - 0.4714 * motor_omega[2];
    ball_omega[1] =                            - 0.8165 * motor_omega[1]  + 0.8165 * motor_omega[2];
    ball_omega[2] =  -0.4714 * motor_omega[0]  - 0.4714 * motor_omega[1]  - 0.4714 * motor_omega[2];
    
    // Scale angular velcoity (in rev/) to rad/ reflected through gear train
    // Convert angular velocities about Y and X axes into linear velocities in X and Y
    ball_vel[0] = ball_omega[1] * (2*PI / GEAR_RATIO) * (D_BALL/2);
    ball_vel[1] = ball_omega[0] * (2*PI / GEAR_RATIO) * (D_BALL/2);
    ball_vel[2] = ball_omega[2] * (2*PI / GEAR_RATIO);
    
}

void velocityIK(float* motor_omega, const float* ball_vel) {
    float ball_omega[3];
    
    ball_omega[0] = ball_vel[1] * GEAR_RATIO / PI / D_BALL;
    ball_omega[1] = -ball_vel[0] * GEAR_RATIO / PI / D_BALL;
    ball_omega[2] = ball_vel[2] * GEAR_RATIO;
    
    
    // Transform from motor axis coordinates to ball's Cartesian coordinates
    motor_omega[0] =   0.7071 * ball_omega[0]                                   - 0.7071 * ball_omega[2];
    motor_omega[1] =  -0.3536 * ball_omega[0]   - 0.6124 * ball_omega[1]        - 0.7071 * ball_omega[2];
    motor_omega[2] =  -0.3536 * ball_omega[0]   + 0.6124 * ball_omega[1]        - 0.7071 * ball_omega[2];
    
    // Scale angular velcoity (in rev/) to rad/ reflected through gear train
    // Convert angular velocities about Y and X axes into linear velocities in X and Y
//    motor_omega[0] = motor_omega[0] * GEAR_RATIO / (2*PI);
//    motor_omega[1] = motor_omega[1] * GEAR_RATIO / (2*PI);
//    motor_omega[2] = motor_omega[2] * GEAR_RATIO / (2*PI);
    
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

//    printf("IMU data: \t%.3f\t%.3f\t%.3f", pimu_data->accel[0], pimu_data->accel[1], pimu_data->accel[2]);
//    printf("\t\t%.3f\t%.3f\t%.3f", pimu_data->mag[0], pimu_data->mag[1], pimu_data->mag[2]);
//    printf("\t\t%.3f\t%.3f\t%.3f\n", pimu_data->gyro[0], pimu_data->gyro[1], pimu_data->gyro[2]);

}
