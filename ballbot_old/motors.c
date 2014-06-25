/* 
 * File:   motors.h
 * Author: Rich
 *
 * Created on August 31, 2013, 10:40 PM
 */

// Provides interface to motor controller

#include "motors.h"

#include "can_utils.h"

#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <string.h>
#include <sys/prctl.h>

sem_t sem_motor_trigger; // to trigger the next run of the loop
pthread_t thread_motor;
pthread_mutex_t _motor_lock;

void motorConfig(void) {
    canOpen();
    pthread_create(&thread_motor, NULL, motorReadBackground, NULL);
}

void motorVelocitySet(int32_t* motors) {
    
    can_buffer can_msg;
       
    // common to all motor commands
    can_msg.IID = SET_VEL;
    can_msg.source = BB_ID;
    can_msg.frame.can_dlc = 4;
    
    int i;
    for (i = 0; i < 3; i++) {
        can_msg.dest = i+1;
        memcpy(can_msg.frame.data, (char*) &(motors[i]), 4);
        //printf("Bytes: %i, %i, %i, %i\n", can_msg.frame.data[0], can_msg.frame.data[1], can_msg.frame.data[2], can_msg.frame.data[3]);
        canWrite(&can_msg);  
    }   
}

void motorEnable(int motor, int en) {
    
    can_buffer can_msg;
    
    can_msg.IID = MOTOR_EN;
    can_msg.source = BB_ID;
    can_msg.dest = motor;
    
    can_msg.frame.can_dlc = 1;
    can_msg.frame.data[0] = en;
    
    canWrite(&can_msg);
    
}

void motorLED(int motor, int en) {
    
    can_buffer can_msg;
    
    can_msg.IID = LED;
    can_msg.source = BB_ID;
    can_msg.dest = motor;
    
    can_msg.frame.can_dlc = 1;
    can_msg.frame.data[0] = en;
    
    canWrite(&can_msg); 
    
}

void *motorReadBackground(void *arg) {
    can_buffer can_msg;
    int MID ;
    int16_t data;
    
    prctl(PR_SET_NAME,"motor",0,0,0);
    
    while (1) {
        
        canRead(&can_msg);
        
        MID = can_msg.IID;
        data = *(int16_t*) &can_msg.frame.data[0];
        
        printf("Received a CAN message ID %i, value %i\n", MID, data);
        
    }
}
