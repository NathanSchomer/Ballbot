/* 
 * File:   can_utils.c
 * Author: Rich
 *
 * Created on August 31, 2013, 10:40 PM
 */


#include "can_utils.h"

#include "robot.h"

//#include <stdio.h>
//#include <stdlib.h>
#include <unistd.h>
//#include <string.h>
 
//#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>

int _can_socket;

char canOpen(void) {

    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        printf("CAN: Error on socket()");
        return -1;                                              // socket open error code
    }
    
    //int recv_own_msgs = 1; /* 0 = disabled (default), 1 = enabled */
    //setsockopt(_can_socket, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));

    strcpy(ifr.ifr_name, CAN_INTERFACE);
    ioctl(_can_socket, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(_can_socket, (struct sockaddr *) &addr, sizeof (addr)) < 0) {
        printf("CAN: Error on bind()");
        return -2;                                              // socket bind error
    }

    return 0;
}

char canClose(void) {
    
}

void canWrite(can_buffer* pb) {
    
    // update the "can_id" portion of the struct
    pb->frame.can_id = (pb->IID & 0x1f) << 6 | ((pb->source & 0x07) << 3) | (pb->dest & 0x07);
    
    // write can_frame to socket
    write(_can_socket, &(pb->frame), sizeof (struct can_frame));

}

void canRead(can_buffer* pb) {
    
    read(_can_socket, &(pb->frame), sizeof (struct can_frame));
    
    pb->IID = (pb->frame.can_id >> 6) & 0x1f;
    pb->source = (pb->frame.can_id >> 3) & 0x07;
    pb->dest = (pb->frame.can_id) & 0x07;
    
}