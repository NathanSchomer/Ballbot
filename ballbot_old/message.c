/* 
 * File:   messege.c
 * Author: Rich
 *
 * Created on September 11, 2013, 10:52 PM
 */

#include "messege.h"
#include "interface.h"
#include "controller.h"

#include <pthread.h>
#include <string.h>

// message handler function pointer table
void (* processMessage[NUM_MESSAGES]) (sockMsg*, sockMsg*) = {
    echoSocket,         // Opcode 0 - 
    getStatus,          // Opcode 1 - 
    getLog,             // Opcode 2 - 
    setGains,           // Opcode 3 - 
};

// message handler functions
void echoSocket(sockMsg* pRecMsg, sockMsg* pReplyMsg) {
    pReplyMsg->length = pRecMsg->length;
    pReplyMsg->data = pRecMsg->data;
}

void getStatus(sockMsg* pRecMsg, sockMsg* pReplyMsg) {
    pReplyMsg->length = 0;
}

void setGains(sockMsg* pRecMsg, sockMsg* pReplyMsg) {
    //float gain[4];
    
//    if (pRecMsg->length == 16) {             //4 floats = 16 bytes
//        memcpy(gain, pRecMsg->data, 16);
//        printf("Gains:\t%f\t%f\t%f\t%f\n", gain[0], gain[1], gain[2], gain[3]);
//    }
    
    //extern struct foo gain;
    
    pthread_mutex_lock(&(gain.lock));                          // lock the gain vector
    
    if (pRecMsg->length == 16) {             //4 floats = 16 bytes
        memcpy(gain.k, pRecMsg->data, 16);
        printf("Gains:\t%f\t%f\t%f\t%f\n", gain.k[0], gain.k[1], gain.k[2], gain.k[3]);
    }   
    
    pthread_mutex_unlock(&(gain.lock));                          // lock the gain vector
    
    pReplyMsg->length = 0;
}

void undefMID(sockMsg* pRecMsg, sockMsg* pReplyMsg) {
    pReplyMsg->length = 0;
    printf("Undefined handler for MID %i\n", pRecMsg->MID);
}


struct {
    uint top[2];                                // points to next empty buffer char
    uint8_t buffer[2][SOCK_BUF_LENGTH];         // ping-pong tx buffers
    char to_dump;                               // the ping-pong buffer that is locked and ready to dump
    pthread_mutex_t lock;                       // used to control access to struct
} dataLog = {.lock = PTHREAD_MUTEX_INITIALIZER};

void putLog (char* data, int length) {
    
    char active = !dataLog.to_dump;
    pthread_mutex_lock(&dataLog.lock);                          // lock the buffer struct
    
    // if there's room, write the message to the buffer
    if (length <= (SOCK_BUF_LENGTH - dataLog.top[active])) {
        memcpy(&(dataLog.buffer[active][dataLog.top[active]]), data, length);
        dataLog.top[active] += length;                          // increment the head pointer
    }
    
    pthread_mutex_unlock(&dataLog.lock);                        // unlock the buffer
    
}

void getLog(sockMsg* pRecMsg, sockMsg* pReplyMsg) {   
    
    /* The buffer this is currently locked had its data dumped last time this
     function was called. This buffer should now be reset and freed. The other
     buffer will now be locked and dumped.*/
    
    pthread_mutex_lock(&dataLog.lock);                          // lock the buffer struct
    
    dataLog.top[dataLog.to_dump] = 0;
    dataLog.to_dump = !dataLog.to_dump;                         // toggle the locked buffer

    pthread_mutex_unlock(&dataLog.lock);                        // unlock the buffer
    
    /* The buffer that is now locked will be dumped. Indicate the length to be
     written and pass a pointer to the data array.*/
    pReplyMsg->length = dataLog.top[dataLog.to_dump];
    pReplyMsg->data = dataLog.buffer[dataLog.to_dump];
    
//    printf("Buffer being dumped: %i\n", active);
//    printf("Logged %i bytes. \n", length);
//    printf("Buffer filled to: %i\n\n", dataLog.top[active]);
    
}