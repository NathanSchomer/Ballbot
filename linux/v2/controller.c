/* 
 * File:   controller.c
 * Author: Rich
 *
 * Created on September 4, 2013, 12:44 AM
 */

#include "controller.h"
#include "robot.h"
#include "motors.h"

#include <pthread.h>

struct control_params con = {.lock = PTHREAD_MUTEX_INITIALIZER};

///* Map X,Y forces and theta torque of ball into motor torques. 
// * - ball_force[0]      =>      force in X direction (Newtons)
// * - ball_force[1]      =>      force in Y direction (Newtons)
// * - ball_force[2]      =>      torque about Z axis (Newton-meters)
// * - motor_torque       =>      motor torque (Newton-meters)*/
//void forceIK(float* motor_torque, const float* ball_force) {
//    float ball_tau[3];
//    
//    // Translate ball's X and Y tangent forces into torques (Z is already a torque)
//    // Scale down by gear ratio to represent required motor torque
//    ball_tau[0] = ball_force[1] / GEAR_RATIO * (D_BALL/2);      // torque in X dir a result of force in Y
//    ball_tau[1] = ball_force[0] / GEAR_RATIO * (D_BALL/2);      // torque in Y dir a result of force in X
//    ball_tau[2] = ball_force[2] / GEAR_RATIO;                   // this is already a torque
//
//    // Transform torque from Cartesian frame to motor axis aligned frame
//    motor_torque[0] =   0.9428 * ball_tau[0]                           - 0.4714 * ball_tau[2];
//    motor_torque[1] =  -0.4714 * ball_tau[0]   + 0.8165 * ball_tau[1]  - 0.4714 * ball_tau[2];
//    motor_torque[2] =  -0.4714 * ball_tau[0]   - 0.8165 * ball_tau[1]  - 0.4714 * ball_tau[2];
//    
//}
//
//void velocityFK(float* ball_vel, const float* motor_omega) {
//    float ball_omega[3];
//    
//    // Transform from motor axis coordinates to ball's Cartesian coordinates
//    ball_omega[0] =   0.9428 * motor_omega[0]  - 0.4714 * motor_omega[1]  - 0.4714 * motor_omega[2];
//    ball_omega[1] =                            - 0.8165 * motor_omega[1]  + 0.8165 * motor_omega[2];
//    ball_omega[2] =  -0.4714 * motor_omega[0]  - 0.4714 * motor_omega[1]  - 0.4714 * motor_omega[2];
//    
//    // Scale angular velcoity (in rev/) to rad/ reflected through gear train
//    // Convert angular velocities about Y and X axes into linear velocities in X and Y
//    ball_vel[0] = ball_omega[1] * (2*PI / GEAR_RATIO) * (D_BALL/2);
//    ball_vel[1] = ball_omega[0] * (2*PI / GEAR_RATIO) * (D_BALL/2);
//    ball_vel[2] = ball_omega[2] * (2*PI / GEAR_RATIO);
//    
//}

//void controlLaw (float* ball_force, const state_vector* state) {
//    pthread_mutex_lock(&(gain.lock));                          // lock the gain vector
//    ball_force[0] = gain.k[0]*state[0].angle + gain.k[1]*state[0].angle_dot + gain.k[2]*state[0].pos + gain.k[3]*state[0].pos_dot;
//    ball_force[1] = gain.k[0]*state[1].angle + gain.k[1]*state[1].angle_dot + gain.k[2]*state[1].pos + gain.k[3]*state[1].pos_dot;
//    pthread_mutex_unlock(&(gain.lock));                        // unlock the gain vector
//}
//
//void stateUpdate(state_vector* state, const imu_data* heading, const float* ball_vel) {
//    /* State variable for X component; angle and angular rate of robot, position
//     * and velocity of ball.*/
//    state[0].angle = heading.rpy[0];
//    state[0].angle_int += (heading.rpy[0]/LOOP_FREQ);
//    state[0].angle_dot = heading.gyro[0] * 180 / PI;
//    state[0].pos += (ball_vel[0] * LOOP_PERIOD);
//    state[0].pos_dot = ball_vel[0];
//
//    /* State variable for Y component; angle and angular rate of robot, position
//     * and velocity of ball.*/
//    state[1].angle = heading.rpy[1];
//    state[1].angle_dot = heading.gyro[1] * 180 / PI;
//    state[1].pos += (ball_vel[1] * LOOP_PERIOD);
//    state[1].pos_dot = ball_vel[1];
//
//
//    /* State variable for theta component; angle and angular rate of robot
//     * about its Z axis.*/
//    state[2].angle = heading.rpy[2];
//    state[2].angle_dot = heading.gyro[2] * 180 / PI;
//}

void chaseLEDs (int dwell_time) {
    static int board_num = 1;
    static int on_off = 0;
    static int dwell_count = 0;
    
    if (++dwell_count == dwell_time) {
        
        motSetLed(board_num, on_off);
        
        dwell_count = 0;
        
        if (++board_num == 4) {
            board_num = 1;
            on_off = !on_off;
        }
        
    }
}