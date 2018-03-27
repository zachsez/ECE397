/*
 * transmitter.c
 *
 *  Created on: Feb 22, 2018
 *      Author: Zach Szczesniak, Michael Dritlein
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"


#include "transmitter.h"


/*
 * Delays ms milliseonds
 */
void t_delay_ms(int ms)
{
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms );
}

void t_delay_us(int us)
{
    SysCtlDelay( (SysCtlClockGet()/(3*1000000))*us );
}

/*
 * Set the period (clock ticks)
 */
void setPeriod(uint32_t period)
{
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, period);
}


/*
 * Set the pulse Width (clock ticks)
 */
void setPulseWidth(uint32_t pulseWidth)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, pulseWidth);
}


/*
 * Setup the square wave output on PF1
 */
void initTransmitter(uint32_t period, uint32_t pulseWidth)
{
	//Set PWM to use the 16 MHz clock so a 40 KHz pulsetrain can be generated.
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    //Enable Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);     //Enable PWM Module 1

    GPIOPinConfigure(GPIO_PF1_M1PWM5);              //Configure PF1 to M1PWM5 type
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);    //Set PF1 to PWM

    //Configure PWM Options
    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, period);

    //Set PWM pulseWidth (expressed in clock ticks)
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, pulseWidth);

    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);

    // Turn off the Output pins
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);
}


/*
 *  Sends a single binary bit of data
 *  Carries out a dead time period then high time period based on bit
 *  Return code 0 success
 *         code 1 bit is not 0 or 1
 */
int sendBit(char bit)
{
    if(bit != '0' && bit != '1')
    {
        return 1;
    }

    //Set state false in case it is not
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);

    //Delay for dead period
    //t_delay_ms(DEADTIME);
    t_delay_us(DEADTIME);

    //Set state true
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);

    //Delay for active time depending on bit value
    if(bit == '0')
    {
        t_delay_us(ZEROTIME);
    }
    else
    {
        t_delay_us(ONETIME);
    }

    //Set state false
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);

    //Return success
    return 0;
}


/*
 * Sends an array of binary data
 * Return code 0 success
 * Return code 1x failure in sendBit
 * Return code 2  n is less than 0
 */
int sendBinary(char binaryCode[], int n)
{
    if(n < 0)
    {
        return 2;
    }

    int i;
    int flag = 1;
    for (i = 0; i < n; ++i)
    {
        flag = sendBit(binaryCode[i]);

        if(flag)
        {
            PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);
            return flag + 10;
        }
    }

    // Set false in case error some where
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);

    return 0;
}

