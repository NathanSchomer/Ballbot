/* 
 * File:   motors.c
 * Author: Nathan Schomer
 *
 * Created on July 18, 2014, 3:40 PM
 */

// Provides interface to motor controller

#include "motors.h"

#include "can_utils.h"

void motorConfig(void) {
    canOpen();
}

void motorVelocitySet(motorMsg* motor) {
    can_buffer can_msg;
    
    if(motor->velocity < 0){
        can_msg.cmd = ROL;
    }
    else if(motor->velocity >= 0){
        can_msg.cmd = ROR;
    }
    
    can_msg.motor = motor->motor;
    can_msg.val = motor->velocity;
    canWrite(&can_msg);
}

/*void motorEnable(int motor){
    can_buffer can_msg;
    
    can_msg.motor = motor;
    //HOW TO ENABLE MOTORS??
}*/

//void motorLED(){}
//void motorRead(){}