#include "driverlib/pin_map.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "launchpad.h"

#include "transmitter.h"
#include "utilFunc.h"
#include "servo.h"
#include "lcd.h"

int main(void)
{
    uint32_t period = 400; //25 us (16Mhz / 6.25)
    uint32_t pulseWidth = 200;    //50% duty
    char binaryCode[3] = "101";
    int n = 3;

    //Set the clock
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    initTransmitter(period, pulseWidth);
    initServo();
    initI2C();
    initLCD();

    while(1)
    {
        write(0, 0, "Team Phantom!");
        sendBinary(binaryCode, n);
        rotateServo();
        delay_ms(2000);
    }
}
