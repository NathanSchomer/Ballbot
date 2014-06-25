/* 
 * File:   tick.h
 * Author: Rich
 *
 * Created on September 1, 2013, 12:02 AM
 */

#ifndef TICK_H
#define	TICK_H

#ifdef	__cplusplus
extern "C" {
#endif

int makeTimer(unsigned int);
void waitOnTimer(int);

#ifdef	__cplusplus
}
#endif

#endif	/* TICK_H */

