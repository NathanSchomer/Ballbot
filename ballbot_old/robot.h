/* 
 * File:   robot.h
 * Author: Rich Primerano
 *
 * Created on September 3, 2013, 3:01 PM
 */

#ifndef ROBOT_H
#define	ROBOT_H

#ifdef	__cplusplus
extern "C" {
#endif

    //control loop update rate
#define LOOP_FREQ               200
#define LOOP_PERIOD             1.0f/LOOP_FREQ

    //IMU filter loop constants
#define Kp                      2.0f            // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki                      0.005f          // integral gain governs rate of convergence of gyroscope biases

    //gear train constants
#define MOTOR_MAX_CPS 50000                     //max motor velocity

//#define GEAR_RATIO = (CLUSTER_GEAR / PINION) * (OUTPUT_PULLEY/CLUSTER_PULLEY) * (D_BALL/D_WHEEL);

#define D_WHEEL         2.5f       //diameter of omniwheel
#define D_BALL         10.5f       //diameter of basketball
#define THETA          45.0f       //included angle between omniwheel's axis and vertical

#define PINION         16.0f       //motor pinion teeth
#define CLUSTER_GEAR   50.0f       //cluster gear teeth
#define CLUSTER_PULLEY 18.0f       //cluster pully teeth
#define OUTPUT_PULLEY  32.0f       //output pully teeth
#define GEAR_RATIO     (CLUSTER_GEAR / PINION) * (OUTPUT_PULLEY / CLUSTER_PULLEY) * (D_BALL / D_WHEEL)

#define ENCODER_CPR    2048.0f     //motor enconder counts per revolutions

    // server constants
#define INTERFACE_PORT          2002            // listening port
#define MESSAGE_LEN             1024            // max message size
#define SMALL_MESSAGE           128
#define TIMEOUT_SEC             0               // use for debugging
#define TIMEOUT_US              50000           // wait this long for a complete message
#define MAX_LINE                1000

    // Misc
#define PI                      3.141592653
#define RAD_2_DEG               180.0f / PI
    
#define CAN_INTERFACE           "can0"

#ifdef	__cplusplus
}
#endif

#endif	/* ROBOT_H */

