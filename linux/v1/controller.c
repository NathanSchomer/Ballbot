/* 
 * File:   controller.c
 * Author: Rich
 *
 * Created on September 4, 2013, 12:44 AM
 */

#include "controller.h"
#include "robot.h"

#include <pthread.h>

struct state_feedback_gain gain = {.k = {0, 0, 0, 0}, .lock = PTHREAD_MUTEX_INITIALIZER};

void ballIK(float* ball_vel, int32_t* motor_cps) {
    int i;
    float omega_x, omega_y, omega_z;                    // Cartesian frame
    float psi_a, psi_b, psi_c;                          // motor axis frame
    
    // Translate ball's X and Y velocities into X and Y angular rates of
    omega_x = 2 * ball_vel[1] / D_BALL;
    omega_y = 2 * ball_vel[0] / D_BALL;
    omega_z = (ball_vel[2] * PI) / 180;

    // psi = T(3x3) * omega
    psi_a =     0.9428 * omega_x                                + -0.4714 * omega_z;
    psi_b =     -0.4714 * omega_x       + 0.8165 * omega_y      - 0.4714 * omega_z;
    psi_c =     -0.4714 * omega_x       - 0.8165 * omega_y      - 0.4714 * omega_z;

    //convert motor velocities to counts per second
    const float counts_per_rad = GEAR_RATIO * ENCODER_CPR / (2 * PI);
    motor_cps[0] = psi_a * counts_per_rad;
    motor_cps[1] = psi_b * counts_per_rad;
    motor_cps[2] = psi_c * counts_per_rad;
    
    //limit check
    for (i = 0; i < 3; i++) {
        if (motor_cps[i] > MOTOR_MAX_CPS)
            motor_cps[i] = MOTOR_MAX_CPS;
        else if (motor_cps[i] < -MOTOR_MAX_CPS)
            motor_cps[i] = -MOTOR_MAX_CPS;
    }
}