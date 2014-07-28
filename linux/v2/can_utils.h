/* 
 * File:   can_utils.h
 * Author: NathanSchomer
 *
 * Created on July 17, 2014, 10:52 AM
 */

#ifndef CAN_UTILS_H
#define	CAN_UTILS_H

#ifdef	__cplusplus
extern "C" {
#endif

    typedef struct {
        char cmd;      //Command number (1 byte)
        char type;     //Type Number    (1 byte)
        char motor;    //Motor or Bank number   (1 byte)
        char val;      //Value (MSB first!)  (4 bytes)
        struct can_frame frame;
    } can_buffer;
    
    char canOpen(void);
    char canClose(void);
    
    void canWrite(can_buffer*);
    void canRead(can_buffer*);
    
#ifdef	__cplusplus
}
#endif

#endif	/* CAN_UTILS_H */