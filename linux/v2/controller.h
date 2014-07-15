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

#include <pthread.h>
#include <stdint.h>

    typedef struct {
        float angle;
        float angle_dot;
        float pos;
        float pos_dot;
    } state_vector; // state vector for one axis

    struct state_feedback_gain {
        float k[4]; // state feedback gains
        pthread_mutex_t lock; // used to control access to struct
    };

    extern struct state_feedback_gain gain;

    void controlLaw(void);
    void stateUpdate(void);
    void ballIK(float*, int32_t*);

#ifdef	__cplusplus
}
#endif

#endif	/* CONTROLLER_H */

