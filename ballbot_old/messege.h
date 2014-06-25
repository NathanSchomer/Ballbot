/* 
 * File:   messege.h
 * Author: Rich
 *
 * Created on September 11, 2013, 10:52 PM
 */

#ifndef MESSEGE_H
#define	MESSEGE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "interface.h"

    #define NUM_MESSAGES    4
    // These are the socket message handler functions
    void echoSocket(sockMsg*, sockMsg*);
    void getStatus(sockMsg*, sockMsg*);
    void getLog(sockMsg*, sockMsg*);
    void setGains(sockMsg*, sockMsg*);
    void undefMID(sockMsg*, sockMsg*);

    extern void (* processMessage[NUM_MESSAGES]) (sockMsg*, sockMsg*);
    
    void putLog (char*, int);
    
#ifdef	__cplusplus
}
#endif

#endif	/* MESSEGE_H */

