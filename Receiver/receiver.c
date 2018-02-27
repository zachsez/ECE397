#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_uart.h"
#include "inc/hw_timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/comp.h"
#include "launchpad.h"

uint32_t adcValue[2];
uint32_t comparatorValue;
int32_t received = 0;
uint32_t timePassed = 0;
int binaryReceived = 0;
double time = 0.0;

void _delay_ms(int milliSecond) {
    SysCtlDelay((SysCtlClockGet() / (3 * 1000)) * milliSecond) ;
}

void ultrasoundReceive()
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    received = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5);

    if (received != 0) {
        timePassed = 0;
        time = 0.0;
        binaryReceived = 1;
        TimerEnable(TIMER2_BASE, TIMER_A);
        TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    }
    else if (binaryReceived == 1) {
        binaryReceived = 0;
        IntDisable(INT_TIMER2A);
        TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
        TimerDisable(TIMER2_BASE, TIMER_A);
        timePassed = TimerValueGet(TIMER2_BASE, TIMER_A);
        timePassed /= SysCtlClockGet();
        HWREG(TIMER2_BASE + TIMER_O_TAV) = 0;
    }
}

void initTimer0A()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerIntRegister(TIMER0_BASE, TIMER_A, ultrasoundReceive);
	TimerEnable(TIMER0_BASE, TIMER_A);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void initTimer2A()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT_UP);
    TimerPrescaleSet(TIMER2_BASE, TIMER_A, 128);
}

void initGPIOC() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_5);
}

void initReceive(int firstPhasePassed) {
	//Divide the PWM clock by 64 so the servo operates properly.
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	initGPIOC();
	initTimer0A();
	initTimer2A();
	TimerLoadSet(TIMER0_BASE, TIMER_A, 8000);
}

