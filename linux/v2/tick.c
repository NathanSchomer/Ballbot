/* 
 * File:   tick.h
 * Author: Rich
 *
 * Created on September 1, 2013, 12:02 AM
 */

// Functions for creating and waiting on a file descriptor based timer

#include "tick.h"

#include <sys/timerfd.h>

int makeTimer(unsigned int frequency) {
    
    int fd;
    struct itimerspec itval;
    unsigned int period = (unsigned int) 1000000000.0 / frequency;

    /* Create the timer */
    fd = timerfd_create(CLOCK_MONOTONIC, 0);

    /* Make the timer periodic */
    itval.it_interval.tv_sec = 0;
    itval.it_interval.tv_nsec = period;
    itval.it_value.tv_sec = 0;
    itval.it_value.tv_nsec = period;

    timerfd_settime(fd, 0, &itval, NULL);

    return fd;
}

void waitOnTimer(int fd) {
    
    unsigned long long missed;

    read (fd, &missed, sizeof (missed));
    
}