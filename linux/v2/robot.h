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

#define D_WHEEL         0.070f          //diameter of omniwheel
#define D_BALL          0.254f          //diameter of basketball
#define THETA           45.0f           //included angle between omniwheel's axis and vertical

#define PINION          16.0f           //motor pinion teeth
#define CLUSTER_GEAR    50.0f           //cluster gear teeth
#define CLUSTER_PULLEY  18.0f           //cluster pully teeth
#define OUTPUT_PULLEY   32.0f           //output pully teeth
#define GEAR_RATIO      (CLUSTER_GEAR / PINION) * (OUTPUT_PULLEY / CLUSTER_PULLEY) * (D_BALL / D_WHEEL)

    // Motor properties
#define ENCODER_CPR     2048.0f         //motor enconder counts per revolutions
#define TORQUE_CONST    3.40e-2         //Nm/amp, (4.81 oz-in/A)
#define BACK_EMF_CONST  6.15e-5         //volts/RPS (3.69 V/kRPM)
#define VOLTAGE_FS      24.0f           //motor driver supply voltage, volts
#define PHASE_OHMS      1.50            //resistance of motor coil, ohms
#define PHASE_IND       2.10e-3         //inductance of motor coil, Henries

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

