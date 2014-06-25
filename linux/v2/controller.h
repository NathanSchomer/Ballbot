/* 
 * File:   controller.h
 * Author: Rich
 *
 * Created on September 4, 2013, 12:44 AM
 */

#ifndef CONTROLLER_H
#define	CONTROLLER_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "imu.h"
    
#include <pthread.h>
#include <stdint.h>

    typedef struct {
        float angle;
        float angle_int;
        float angle_dot;
        float pos;
        float pos_dot;
    } state_vector; // state vector for one axis

    struct control_params {
        union {
            float k[6]; // state feedback gains
            struct {
                float p;
                float i;
                float d;
                float p_pos;
                float d_pos;
                float int_sat;
            };
        };
        float x_trim, y_trim;
        pthread_mutex_t lock; // used to control access to struct
    };

    extern struct control_params con;

//    void controlLaw(float*, const state_vector*);
//    void stateUpdate(state_vector*, const imu_data*, const float*);
//    void forceIK(float*, const float*);
//    void velocityFK(float*, const float*);
    
    void chaseLEDs (int);

#ifdef	__cplusplus
}
#endif

#endif	/* CONTROLLER_H */

