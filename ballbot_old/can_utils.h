/* 
 * File:   can_utils.h
 * Author: Rich
 *
 * Created on August 31, 2013, 10:40 PM
 */

#ifndef CAN_UTILS_H
#define	CAN_UTILS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <linux/can.h>
#include <linux/can/raw.h>
    
    // extend the CAN frame to include source, destination, and IID fields
    typedef struct {
        char source;
        char dest;
        char IID;
        struct can_frame frame;
    } can_buffer ;
    
    
    char canOpen(void);
    char canClose(void);
    
    void canWrite(can_buffer*);
    void canRead(can_buffer*);
    
#ifdef	__cplusplus
}
#endif

#endif	/* CAN_UTILS_H */

