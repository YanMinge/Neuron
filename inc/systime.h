#ifndef __SYSTTIME_H__
#define __SYSTTIME_H__

#include "M051Series.h"
extern volatile unsigned long system_time;

unsigned long millis(void);
void delayMicroseconds(uint32_t us);
void delay(unsigned long ms);
#endif //__SYSTTIME_H__

