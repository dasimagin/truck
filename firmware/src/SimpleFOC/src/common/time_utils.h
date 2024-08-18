#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include "foc_utils.h"
#define delay(X) _delay(X)

/** 
 * Function implementing delay() function in milliseconds 
 * - blocking function
 * - hardware specific

 * @param ms number of milliseconds to wait
 */
void _delay(unsigned long ms);

/** 
 * Function implementing timestamp getting function in microseconds
 * hardware specific
 */
unsigned long _micros();

void delayMicroseconds(int microsec);

#endif