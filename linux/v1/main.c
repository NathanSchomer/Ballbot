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

int main(int argc, char** argv) {
    
    struct sched_param param = {.sched_priority = 10};
    sched_setscheduler(0, SCHED_RR, &param);
    
    int loop_timer;                             // timer file handler ID
    imu_data heading;                           // raw and processed IMU data
    
    int32_t mot_vel[3];                         // motor velocity registers (encoder ticks/sec)
    float ball_vel[3];                          // x, y, and theta velocities of ball
    
    state_vector state_x, state_y;
    float force_x, force_y;
    float mass = 1;
    
    struct timeval t0, t1;
    float elapsed;
    
    loop_timer = makeTimer(LOOP_FREQ);          // create main loop timer
    imuConfig();                                // create IMU read thread
    socketConfig();                             // create message socket thread
    motorConfig();                              // create motor update thread
    
    
    mot_vel[0] = 0;
    mot_vel[1] = 0;
    mot_vel[2] = 0;
    
    motorEnable(7, 1);
    //motorLED(7, 1);
    
    while(1) {
        
        //gettimeofday(&t0, NULL);
        
        imuGetData(&heading);                   // trigger another IMU update
        
        // update state estimate
        state_x.angle = heading.rpy[0];
        state_x.angle_dot = heading.gyro[0] * 180 / PI;
        state_x.pos += (ball_vel[0] * LOOP_PERIOD);
        state_x.pos_dot = ball_vel[0];
        
        state_y.angle = heading.rpy[1];
        state_y.angle_dot = heading.gyro[1] * 180 / PI;
        state_y.pos += (ball_vel[1] * LOOP_PERIOD);
        state_y.pos_dot = ball_vel[1];
        
        // compute control force (map into acceleration of ball)
        pthread_mutex_lock(&(gain.lock));                          // lock the gain vector
        force_x = gain.k[0]*state_x.angle + gain.k[1]*state_x.angle_dot + gain.k[2]*state_x.pos + gain.k[3]*state_x.pos_dot;
        force_y = gain.k[0]*state_y.angle + gain.k[1]*state_y.angle_dot + gain.k[2]*state_y.pos + gain.k[3]*state_y.pos_dot;
        pthread_mutex_unlock(&(gain.lock));                        // unlock the gain vector
        
        
        ball_vel[0] += (force_x / mass) * LOOP_PERIOD;
        ball_vel[1] += (force_y / mass) * LOOP_PERIOD;
        ball_vel[2] = 0;
        
        ballIK(ball_vel, mot_vel);       // calculate motor velocities from IK function
        
//        printf("Ball velocities: X - %f, Y - %f\n", ball_vel[0], ball_vel[1]);
        
        motorVelocitySet(mot_vel);             // write new motor velocities
        
        // log variables
//        putLog ((char*) heading.rpy, 12);
//        putLog ((char*) heading.accel, 12);
//        putLog ((char*) heading.mag, 12);
//        putLog ((char*) heading.gyro, 12);
        putLog ((char*) &state_x, sizeof(state_x));
        
        //gettimeofday(&t1, NULL);
        //elapsed = (t1.tv_sec - t0.tv_sec)*1000 + (t1.tv_usec - t0.tv_usec) / 1000.0f;
        //printf("Loop time: %f ms\n", elapsed);
        
        waitOnTimer(loop_timer);                // wait for next timer tick
        
        //printf("Main loop %i \n", i);
        
    }
    
    return (EXIT_SUCCESS);
}