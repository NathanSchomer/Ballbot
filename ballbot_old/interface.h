/* 
 * File:   interface.h
 * Author: Rich
 *
 * Created on August 31, 2013, 10:41 PM
 */

#ifndef INTERFACE_H
#define	INTERFACE_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "robot.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define         SOCK_ERR_OK             0
#define         SOCK_ERR_HEADER         1      //timed out or errored during header read
#define         SOCK_ERR_DATA           2
#define         SOCK_ERR_EXTRA          3
    
#define         SOCK_BUF_LENGTH         1024
       
    typedef struct {
        uint16_t MID;                           // message ID
        uint16_t length;
        uint8_t* data;
    } sockMsg;
   
    void socketConfig(void);
    void *messageServer(void*);
    
    // helper functions
    signed char readMessage(sockMsg*);
    signed char writeMessage(sockMsg*);

#ifdef	__cplusplus
}
#endif

#endif	/* INTERFACE_H */

