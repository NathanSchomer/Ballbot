/* 
 * File:   main.c
 * Author: Rich
 *
 * Created on August 31, 2013, 10:10 PM
 */

#include "controller.h"
#include "interface.h"
#include "imu.h"
#include "messege.h"
#include "motors.h"
#include "tick.h"
#include "robot.h"

#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/timerfd.h>

void forceIK(float*, const float*);
void velocityFK(float*, const float*);
void velocityIK(float*, const float*);

int main(int argc, char** argv) {
    
    struct sched_param param = {.sched_priority = 10};
    sched_setscheduler(0, SCHED_RR, &param);
    
    int loop_timer;                             // timer file handler ID
    
    imu_data heading;                           // raw and processed IMU data
    float ball_vel[3], motor_omega[3];
    state_vector s[2];
    
    int i;
    
//    struct timeval t0, t1;
//    float elapsed;
    
    loop_timer = makeTimer(LOOP_FREQ);          // create main loop timer
    imuConfig();                                // create IMU read thread
    socketConfig();                             // create message socket thread
    motorConfig();                              // create motor update thread
    
//    motSetMode(7, MODE_DUTY);
    motSetMode(7, MODE_VEL);
    motSetLed(7, 1);

    while(1) {
        
        // gettimeofday(&t0, NULL);
        
        /* Read latest heading values. A call to imuGetData() returns the 
         * currently held IMU data and triggers another IMU update in a 
         * background thread.*/
        imuGetData(&heading);
        
        /* Read latest motor speeds. A call to motGetVel() returns the most
         * recently reported motor velocity values. Velocity values are reported
         * automatically when a motSetVolt() command is issued (below).*/
        //motGetVel(motor_vel);
                
        /* Map the motor velocities (given in counts per second) into robot 
         * velocity. This velocity is described in X and Y translational components,
         * and theta rotational component.*/
        //velocityFK(ball_vel, motor_vel);

        pthread_mutex_lock(&(con.lock));                          // lock the control parameter vector
        
        // trim roll and pitch
        heading.rpy[0] -= con.x_trim;
        heading.rpy[1] -= con.y_trim;
        
        /**/
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
        
        //ball_vel[2] = con.p_pos * s[2].pos + con.d_pos * s[2].pos_dot;
        ball_vel[2] = 0;
        
        pthread_mutex_unlock(&(con.lock));                        // unlock the control parameter vector
        
        /* Motors are operating in voltage control mode. The current (and therefore
         * the torque) of the motor can be approximated as the applied voltage times
         * the phase resistance. This is valid at zero motor velocity and neglects
         * the motor's back-EMF. TODO: add current control to motor.*/
//        forceIK(motor_torque, ball_force);    
//        motor_duty[0] = motor_torque[0]* (PHASE_OHMS / (TORQUE_CONST * VOLTAGE_FS));
//        motor_duty[1] = motor_torque[1]* (PHASE_OHMS / (TORQUE_CONST * VOLTAGE_FS));
//        motor_duty[2] = motor_torque[2]* (PHASE_OHMS / (TORQUE_CONST * VOLTAGE_FS));
//        motSetDuty(motor_duty);

        
//        // this is funny business
//        ball_vel[0] = ball_force[0];
//        ball_vel[1] = ball_force[1];
//        ball_vel[2] = ball_force[2];
        
        velocityIK(motor_omega, ball_vel);
        motSetVel(motor_omega);
        
        // calculate the required ball force
        //forceIK(float* motor_torque, const float* ball_force);
        // convert motor torque into required voltage
        //motSetDuty(float* duty);
        
        // log variables
        putLog ((char*) &s[0], sizeof(s[0]));
//        printf("Values: %.2f, %.2f, %.2f, %.2f, %.2f\n", s[0].angle, s[0].angle_dot, s[0].angle_int, s[0].pos, s[0].pos_dot);
//        printf("Ball velocities: %.2f, %.2f, %.2f\n", ball_vel[0], ball_vel[1], ball_vel[2]);
//        printf("Motor velocities: %.2f, %.2f, %.2f\n\n", motor_omega[0], motor_omega[1], motor_omega[2]);
        printf("RPY: %.2f, %.2f, %.2f\n\n", heading.rpy[0], heading.rpy[1], heading.rpy[2]);

        
        //gettimeofday(&t1, NULL);
        //elapsed = (t1.tv_sec - t0.tv_sec)*1000 + (t1.tv_usec - t0.tv_usec) / 1000.0f;
        //printf("Loop time: %f ms\n", elapsed);
        
//        chaseLEDs(20);
        waitOnTimer(loop_timer);                // wait for next timer tick
        
    }
    
    return (EXIT_SUCCESS);
}

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