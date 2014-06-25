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
    
// Device IDs
#define BB_ID           0
#define MOTOR1_ID       1
#define MOTOR2_ID       2
#define MOTOR3_ID       3

// Motor Command CAN Message IDs
#define ECHO            0
#define SET_LED         1
#define SET_MODE        2
#define SET_VEL         3
#define SET_GAIN        4
#define SET_DUTY        5

// Motor Reply CAN Message IDs
#define REPLY_VEL       12

// Motor Modes
#define MODE_OFF        0
#define MODE_POS        1
#define MODE_VEL        2
#define MODE_DUTY       3
#define MODE_CURR       4
    
// PID param IDs
#define SET_P 1
#define SET_I 2
#define SET_D 3

    void motorConfig(void);
    void *motProcRxMsg(void*);
    
    void motSetVel(float*);
    void motGetVel(float*);
    
    void motSetDuty(float*);
    void motSetMode(int, int);
    void motSetLed(int, int);
    
#ifdef	__cplusplus
}
#endif

#endif	/* MOTORS_H */

