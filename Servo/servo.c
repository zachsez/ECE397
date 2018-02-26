#include "driverlib/pin_map.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

void s_delay_ms(int ms) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
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
}

void rotateServo(void) 
{
    float duty;
    float dutySpan = 375;
    float minDuty = 187.5;
    float angle;
    int i;

	//Beginning at 0 degrees, increment the angle by one degree until 180 degrees is reached.
	for (i = 0; i < 180; ++i) {
		angle = i + 1;
		duty = minDuty + ((dutySpan * angle) / 180);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, duty);
		s_delay_ms(100);
	}
	//Beginning at 180 degrees, decrement the angle by one degree until 0 degrees is reached.
	for (i = 180; i > 0; --i) {
		angle = i;
		duty = minDuty + ((dutySpan * angle) / 180);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, duty);
		s_delay_ms(100);
	}
}

