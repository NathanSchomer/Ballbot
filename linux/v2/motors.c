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
pthread_mutex_t _motor_lock = PTHREAD_MUTEX_INITIALIZER;

struct motor_info {
    float velocity;
} _mot_get[3];

void motorConfig(void) {
    canOpen();
    pthread_create(&thread_motor, NULL, motProcRxMsg, NULL);
}

void *motProcRxMsg(void *arg) {
    can_buffer can_msg;
    uint source_add;
    
    prctl(PR_SET_NAME,"motor",0,0,0);
    
    while (1) {
        canRead(&can_msg);                              // block on CAN RX message
        
        source_add = (can_msg.source - 1);
        
        if (source_add < 3) {
            
            pthread_mutex_lock(&_motor_lock);
            
            if (can_msg.IID == REPLY_VEL) {
                _mot_get[source_add].velocity = *(float*) &can_msg.frame.data[0];
            }
            else {
                printf("Unknown CAN message received! IID %i.\n", can_msg.IID);
            }
            
            pthread_mutex_unlock(&_motor_lock);
        }
    }
}

void motSetVel(float* vel) {
    
    can_buffer can_msg;
       
    // common to all motor commands
    can_msg.IID = SET_VEL;
    can_msg.source = BB_ID;
    can_msg.frame.can_dlc = 4;
    
    int i;
    for (i = 0; i < 3; i++) {
        can_msg.dest = i+1;
        memcpy(can_msg.frame.data, (char*) &(vel[i]), 4);
        canWrite(&can_msg);  
    }   
}

void motGetVel(float* vel) {
    
    int i;
    
    pthread_mutex_lock(&_motor_lock);
    
    for (i = 0; i < 3; i++) {
        vel[i] = _mot_get[i].velocity;
    }
    
    pthread_mutex_unlock(&_motor_lock);
    
}

void motSetDuty(float* duty) {
    
    int i;
    can_buffer can_msg;
       
    // common to all motor commands
    can_msg.IID = SET_DUTY;
    can_msg.source = BB_ID;
    can_msg.frame.can_dlc = 4;
    
    for (i = 0; i < 3; i++) {
        can_msg.dest = i+1;
        memcpy(can_msg.frame.data, (char*) &(duty[i]), sizeof(float));
        canWrite(&can_msg);  
    }
}

void motSetMode(int motor, int mode) {
    
    can_buffer can_msg;
    
    can_msg.IID = SET_MODE;
    can_msg.source = BB_ID;
    can_msg.dest = motor;
    
    can_msg.frame.can_dlc = 4;
    can_msg.frame.data[0] = mode;
    
    canWrite(&can_msg);
    
}

void motSetLed(int motor, int on_off) {
    
    can_buffer can_msg;
    
    can_msg.IID = SET_LED;
    can_msg.source = BB_ID;
    can_msg.dest = motor;
    
    can_msg.frame.can_dlc = 4;
    can_msg.frame.data[0] = on_off;
    
    canWrite(&can_msg); 
    
}

