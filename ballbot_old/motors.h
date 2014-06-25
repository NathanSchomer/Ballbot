/* 
 * File:   motors.h
 * Author: Rich
 *
 * Created on August 31, 2013, 10:40 PM
 */

#ifndef MOTORS_H
#define	MOTORS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <stdint.h>
    
//Device IDs
#define BB_ID 0
#define MOTOR1_ID 1
#define MOTOR2_ID 2
#define MOTOR3_ID 3

//COMMAND INTERFACE
#define ECHO 0
#define LED 1
#define MOTOR_EN 2
#define SET_VEL 3
#define SET_GAIN 4
#define MODE 5

//PID param IDs
#define SET_P 1
#define SET_I 2
#define SET_D 3

    void motorConfig(void);
    void *motorReadBackground(void *);
    
    void motorVelocitySet(int32_t*);
    void motorEnable(int, int);
    void motorLED(int, int);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTORS_H */

