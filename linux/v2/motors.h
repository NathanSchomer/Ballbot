/* 
 * File:   motors.h
 * Author: NathanSchomer
 *
 * Created on July 17, 2014, 1:39 PM
 */

#ifndef MOTORS_H
#define	MOTORS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <stdint.h>
    
#define BB_ID 0
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3

    //MOTION COMMANDS
#define ROR 1   //rotate right
#define ROL 2   //rotate left
#define MST 3   //motors stop
#define MVP 4   //move to position
    //PARAMETER COMMANDS
#define SAP 5   //set axis parameter
#define GAP 6   //get axis parameter
#define STAP 7  //store axis parameter into EEPROM
#define RSAP 8  //restore axis parameter from EEPROM
#define SGP 9   //set global parameter
#define GGP 10  //get global parameter
#define STGP 11 //store global parameter into EEPROM
#define RSGP 12 //restore global parameter from EEPROM
    //CONTROL COMMANDS
#define JA 22   //jump always
#define JC 21   //jump conditional
#define COMP 20 //compare accumulator with constant value
#define CSUB 23 //call subroutine
#define RSUB 24 //return from subroutine
#define WAIT 27 //wait for a specified event
#define STOP 28 //end of a TMCL program
    // I/O PORT COMMANDS
#define SIO 14  //set output
#define GIO 15  //get input
    //CALCULATION COMMANDS (For standalone mode ONLY!!)
#define CALC 19     //calculate using the accumulator and a constant value
#define CALCX 33    //calculate using the accumulator and the X register
#define AAP 34      //copy accumulator to an axis parameter
#define AGP 35      //copy accumulator to a global parameter
    
    typedef struct {
        int32_t motor;
        float velocity;
    } motorMsg;
    
    void motorConfig(void);
    void *motorReadBackground(void *);
    
    void motorVelocitySet(motorMsg* motor);
    void motorEnable(int, int);
    void motorLED(int, int);
    
#ifdef	__cpluspluss
}
#endif

#endif	/* MOTORS_H */

