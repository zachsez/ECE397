/*
 * utilFunc.c
 *
 *  Created on: Feb 22, 2018
 *      Author: Zach
 */

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"

#include "utilFunc.h"


void delay_ms(uint32_t num)
{
    SysCtlDelay( num * (SysCtlClockGet() / (3 * 1000 )));
}


void delay_us(uint32_t num)
{
    SysCtlDelay( num * (SysCtlClockGet() / ( 3* 1000000)));
}
