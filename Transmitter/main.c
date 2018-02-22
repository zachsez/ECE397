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

void _delay_ms(int ms)
{
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}

void sendBinary(char binaryCode[], int n)
{
    int i;

    for (i = 0; i < n; ++i) {
        PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);

        _delay_ms(500);

        PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);

        if (binaryCode[i] == '0')
            _delay_ms(1000);
        else
            _delay_ms(2000);
    }
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);
}

int main(void)
{
    uint32_t period = 400; //25 us (16Mhz / 6.25)
    uint32_t duty = 200;    //pulse width
    char binaryCode[3] = "101";
    int n = 3;

    //Set the clock
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

   //Configure PWM Clock divide system clock by 64
   //SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

   // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins

    //Configure PF1,PF2,PF3 Pins as PWM
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

    //Configure PWM Options
    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, period);

    //Set PWM duty
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, duty);

    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);

    // Turn on the Output pins
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);

    while(1)
    {
        sendBinary(binaryCode, n);
        _delay_ms(2000);
    }
}
