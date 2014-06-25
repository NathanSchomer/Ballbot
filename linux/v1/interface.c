/* 
 * File:   interface.c
 * Author: Rich
 *
 * Created on August 31, 2013, 10:41 PM
 */
#include "interface.h"

#include "messege.h"
#include "robot.h"

#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <errno.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/prctl.h>
#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <unistd.h>           /*  misc. UNIX functions      */

#define procMessage(rxMsg, txMsg)        processMessage[(*rxMsg).MID](rxMsg, txMsg)

uint _socket_handle;
pthread_t _thread_socket;

void socketConfig(void) {
    pthread_create(&_thread_socket, NULL, messageServer, NULL);
}

void *messageServer(void *ptr) {
    uint listen_sock;
    struct sockaddr_in servaddr;
    const struct timeval tv = {.tv_sec = 0, .tv_usec = 0};
    signed char error_status;
    
    prctl(PR_SET_NAME,"socket",0,0,0);
    
    char rxBuffer[SMALL_MESSAGE];
    sockMsg sockRxBuff = {.data = rxBuffer};
    
    char replyBuffer[SMALL_MESSAGE];
    sockMsg sockReplyBuff = {.data = replyBuffer};

    /*  Create the listening socket  */
    listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if ( listen_sock < 0 ) {
	printf("Socket: Error creating listening socket.\n");         //TODO: handle error
    }
    
    // set server address structure elements
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(INTERFACE_PORT);
    
    int yes = 1;
    if ( setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1 )
        perror("Socket: Error setting SO_REUSEADDR property\n");
    
    /*  Bind our socket address to the listening socket, and call listen()  */
    if ( bind(listen_sock, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0 ) {
	printf("Socket: Error calling bind()\n");                     //TODO: handle error
    }

    if ( listen(listen_sock, 5) < 0 ) {
	printf("Socket: Error calling listen()\n");                     //TODO: handle error
    }
    
    // loop forever, accepting connections and processing messages
    while ( 1 ) {
        
        // ensure that 'accept' will block forever
        setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));
        
        // block on accept connection
        _socket_handle = accept(listen_sock, NULL, NULL);
        
	if (_socket_handle < 0) {
	    printf("Socket: Error calling accept()\n");                 //TODO: handle error
	}
        
        error_status = readMessage(&sockRxBuff);
        
        if (error_status == SOCK_ERR_OK) {                      // message received without error
            if (sockRxBuff.MID < NUM_MESSAGES) {
                procMessage(&sockRxBuff, &sockReplyBuff);
                sockReplyBuff.MID = 1000 + sockRxBuff.MID;
            }
            else {
                sockReplyBuff.MID = 2001;
                sockReplyBuff.length = 0;
            }
        }
        else {
            sockReplyBuff.MID = 2002;
            sockReplyBuff.length = 0;
        }

        error_status = writeMessage(&sockReplyBuff);
                
	// Close the connected socket
	if (close(_socket_handle) < 0) {
	    printf("Socket: Error calling close()\n");                  //TODO: handle error
            exit(0);
	}
    }
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////// Helper functions /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
signed char readMessage(sockMsg *rxBuffer) {
    size_t num_bytes;
    const struct timeval tv = {.tv_sec = TIMEOUT_SEC, .tv_usec = TIMEOUT_US};
    char header[4];
    
    #define DB_SIZE     10
    char discard[DB_SIZE];
     
    // Set a timeout for the blocking 'read'. This gives time for all bytes to arrive
    setsockopt(_socket_handle, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));
    
    // Read the first two ints (4 bytes), the message ID and LENGTH
    // All messages will have these two characters as a minimum
    num_bytes = recv(_socket_handle, (char*) header, 4, MSG_WAITALL);
    if (num_bytes == 4) {       // correct number of header bytes received
        rxBuffer->MID = *(uint16_t*) header;
        rxBuffer->length = *(uint16_t*) (header + 2);
        
        if (rxBuffer->length > SMALL_MESSAGE) {
            rxBuffer->length = SMALL_MESSAGE;
        }
    }
    else {                      // too few header bytes or error
        rxBuffer->MID = 0;
        rxBuffer->length = 0;
        return SOCK_ERR_HEADER;
    }

    // read in LENGTH number of bytes, timeout after TIMEOUT_US
    num_bytes = recv(_socket_handle, rxBuffer->data, rxBuffer->length, MSG_WAITALL);
    // did we receive the correct number of bytes?
    if (num_bytes < rxBuffer->length) {                                // too few header bytes or error
        rxBuffer->MID = 0;
        rxBuffer->length = num_bytes;
        return SOCK_ERR_DATA;
    }
    
    // check to see if more bytes remain (this is a warning)
    ioctl(_socket_handle, FIONREAD, &num_bytes);
    if (num_bytes) {
        while (recv(_socket_handle, discard, DB_SIZE, 0) > 0) {}        // discard extra bytes
        rxBuffer->MID = 0;
        rxBuffer->length = num_bytes;                                   // return the number of bytes dropped
        return SOCK_ERR_EXTRA;
    }
    else {
        return SOCK_ERR_OK;
    }
    
}

signed char writeMessage(sockMsg *pm) {
    
    // write header part
    if (write(_socket_handle, (char*) pm, 4) < 4) {
        return 1;
    };
    
    // write data part
    if (write(_socket_handle, (char*) pm->data, pm->length) < pm->length ) {
        return 2;
    }
    
    return 0;
}

//void printMessage(sockMsg *message) {
//    
//    printf("Message\n");
//    printf("  ID:\t\t%i\n", message->MID);
//    printf("  Length:\t%i\n", message->length);
//    
//    int i;
//    printf("  Data:\t\t");
//    for (i = 0; i < message->length; i++) {
//        printf("%i  ", message->data[i]);
//    }
//    printf("\n");
//    
////    printf("  Raw:\t\t");
////    for (i = 0; i < message->length + 3; i++) {
////        printf("%u  ", *(uint8_t*)(message + i));
////    }
////    printf("\n");
////    
////    if (message->MID == 1) {
////        float f1 = *(float*)(message->data);
////        printf("  Float:\t%f\n", f1);
////    }
////    
////    if (message->MID == 2) {
////        int16_t i1 = *(int16_t*)(message->data);
////        printf("  Int16:\t%i\n", i1);
////    }
////    
////    if (message->MID == 3) {
////        printf("  Float:\t%f\n", *(float*)(message->data));
////        printf("  Float:\t%f\n", *(float*)(message->data + 4));       //this is magic
////    }
////    
////    if (message->MID == 4) {
////        int i;
////        printf("  Floats:");
////        for (i = 0; i < message->length; i+=4) {
////            printf("  %f", *(float*)(message->data + i));
////        }
////        printf("\n");
////    }
//    
//}