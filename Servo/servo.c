#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "lcd.h"

float previousAngle = 0.0;
int clockWise = 0;
int counterClockwise = 1;

void s_delay_ms(int ms) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}

void rotateServo(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    float duty;
    float dutySpan = 375;
    float minDuty = 187.5;
    float angle;
    int i;

    if (previousAngle < 180.0 && counterClockwise == 1) {
        if (previousAngle >= 179.0) {
            counterClockwise = 0;
            clockWise = 1;
        }
        angle = previousAngle + 1.0;
        previousAngle = angle;
        clear();
        writeAngle(angle);
        duty = minDuty + ((dutySpan * angle) / 180);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, duty);
    }
    else if (previousAngle > 0.0 && clockWise == 1) {
        if (previousAngle == 1.0) {
            counterClockwise = 1;
            clockWise = 0;
        }
        angle = previousAngle - 1.0;
        previousAngle = angle;
        clear();
        writeAngle(angle);
        duty = minDuty + ((dutySpan * angle) / 180);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, duty);
    }
}

void initTimer1A()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerIntRegister(TIMER1_BASE, TIMER_A, rotateServo);
    TimerEnable(TIMER1_BASE, TIMER_A);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

void initServo(void)
{
    uint32_t pwmPeriod = 5000; //Period = (16 MHz / 64 * 5000) = 20 ms.

    //Set the PWM clock to be the system clock divided by 64.
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    //Enable peripherals.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //Setup PinB6 as PWM.
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Set the period for PWM.
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, pwmPeriod);

    //Enable PWM
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    //Turn on PinB6 for PWM.
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);

    initTimer1A();

    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 4);
}

