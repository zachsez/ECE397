/*
 * main.c
 * 
 * Author: Michael Dritlein
 */
#include "utils/uartstdio.c"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
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
#include "transmitter.h"
#include "servo.h"
#include "lcd.h"
/*
 * Ultrasonic tracking system. Comprised of two nodes with a microcontroller, ultrasonic transducers for
 * transmitting and receiving, servos, and an LCD. One node starts in the receive phase and the other
 * begins in the lock on (transmit) phase. If the node starts in the lock on phase,  a servo is rotated
 * in increments of 5 degrees and a binary code is sent at each increment. This binary code is used to
 * verify a transmit and receive. A distance calculation is also performed with each transmit and receive.
 * The servo sweeps through angles from 0 to 180 degrees and back. Then, the sweeping range is divided by
 * 3 and centered around the angle with the minimum distance (180, 60, 20, and ~7 degree sweeps). At the
 * end of the smallest sweep, the angle found with the minimum distance is the angle locked on to. If the
 * node starts in the receive phase, it relays the binary code until a timer reaches a predetermined value.
 * Then, the node switches to the lock on phase. To begin the tracking system, press the SW2 push button
 * one one node (receive phase) and then immediately press the SW1 button on the other node (lock on phase).
 * Then, the nodes will run through their lock on and receive phases until each node points towards the
 * other and displays the distance and angle to the other node.
 */

//Used to verify if a binary code corresponds to a lock on phase code or a switch phase code.
char verifiedCode[2]; 
volatile int binaryCounter = 0; //Count the number of binary digits received.
volatile int endReceive = 0; //Boolean to tell the node to receive binary.
volatile int startTransmit = 0; //Boolean to tell the node to transmit a binary code.
volatile int binaryReceived = 0; //Boolean used to check if a rising edge has occurred.
//Measure the time of a logic high to determine if it is a binary digit.
volatile int time = 0;
//Variable used to check if a logic low is noise or an intended delay.
volatile int timeOut = 0; 
int lockOnPhase = 0; //Boolean to tell the node if it is in its lock on phase or not.
uint32_t angleList[150]; //Array used to store the angles during a servo sweep.
//Maintain a list of the distances during a reading of multiple distances.
int distanceList[300];
volatile int nodeEnabled = 0; //Boolean used to enable the node for normal operation.
uint32_t startingAngle = 0; //The angle that the servo starts at before tracking.
//Boolean used to know whether to increment the angle being chosen by the user
//(before tracking) or decrement the angle.
volatile int incrementAngle = 1;
volatile int reportDistance = 0;
volatile int sendDistance = 0;
volatile uint32_t currentAngle = 0;
volatile int updateCW = 0;
volatile int updateCCW = 1;
volatile int receiveAngleListCCW[150];
volatile int receiveAngleListCW[150];
volatile int receiveIndex = 0;
volatile int receiveMinIndexCCW;
volatile int receiveMinIndexCW;
volatile int receiveDistanceList[150];
volatile int minDistanceCCW = 100000;
volatile int minDistanceCW = 100000;

static enum{
    Send_Transmit, Send_End, Lock_On_Wait, Lock_On_End, Idle
} currentState = Idle;

//Milliseconds delay.
void _delay_ms(int milliSecond) {
    SysCtlDelay((SysCtlClockGet() / (3 * 1000)) * milliSecond) ;
}

//Microseconds delay.
void _delay_us(int us)
{
    SysCtlDelay( (SysCtlClockGet()/(3*1000000))*us );
}

int absValue(int arg1, int arg2) {
    int value = arg1 - arg2;
    if (value < 0)
        value *= -1;

    return value;
}

int findMax(int arg1, int arg2) {
    int max;

    if (arg1 > arg2)
        max = arg1;
    else
        max = arg2;

    return max;
}

void updateAngle() {
    uint32_t finalAngle;
    int i;
    int averagedDistance = 1000000;

    TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

    if (updateCCW == 1) {
        receiveAngleListCCW[receiveIndex] = currentAngle;
        currentAngle += 5;
        ++receiveIndex;
        if (currentAngle == 180) {
            TimerIntDisable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
            _delay_ms(500);
            TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
            updateCCW = 0;
            updateCW = 1;
            for (i = 3; i < 36; ++i) {
                averagedDistance = (receiveDistanceList[i - 3] +receiveDistanceList[i - 2] +
                        receiveDistanceList[i - 1] + receiveDistanceList[i]) / 4;
                if (averagedDistance < minDistanceCCW) {
                    minDistanceCCW = averagedDistance;
                    receiveMinIndexCCW = i;
                }
            }
        }
    }
    else if (updateCW == 1) {
        receiveAngleListCW[receiveIndex] = currentAngle;
        currentAngle -= 5;
        ++receiveIndex;
        if (currentAngle == 0) {
            updateCCW = 1;
            updateCW = 0;
            TimerIntDisable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
            for (i = 39; i < 72; ++i) {
                averagedDistance = (receiveDistanceList[i - 3] + receiveDistanceList[i - 2] +
                        receiveDistanceList[i - 1] + receiveDistanceList[i]) / 4;
                if (averagedDistance < minDistanceCW) {
                    minDistanceCW = averagedDistance;
                    receiveMinIndexCW = i;
                }
            }
            if (receiveAngleListCCW[receiveMinIndexCCW] == 0)
                finalAngle = receiveAngleListCW[receiveMinIndexCW];
            else if (receiveAngleListCW[receiveMinIndexCW] == 0)
                finalAngle = receiveAngleListCCW[receiveMinIndexCCW];
            else
                finalAngle = (receiveAngleListCCW[receiveMinIndexCCW]
                         + receiveAngleListCW[receiveMinIndexCW]) / 2;
            rotateServo(finalAngle);
            GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_2);
            _delay_ms(200);
            GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_2);
            uprintf("finalAngle: %u\n", finalAngle);
            uprintf("receiveAngleListCCW: %d\n receiveAngleListCW: %d\n", receiveAngleListCCW[receiveMinIndexCCW],
                         receiveAngleListCW[receiveMinIndexCW]);
            clear();
            writeAngle(finalAngle);
            PWMGenDisable(PWM0_BASE, PWM_GEN_0);
        }
    }

    uprintf("currentAngle: %u\n", currentAngle);
}

void initTimer5A() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    TimerIntRegister(TIMER5_BASE, TIMER_A, updateAngle);
    TimerLoadSet(TIMER5_BASE, TIMER_A, 3840000);
    IntEnable(INT_TIMER5A);
    TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER5_BASE, TIMER_A);
}

//Initialize UART to be used for uprintf().
void initUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    GPIOPinTypeUART(GPIO_PORTA_BASE, (GPIO_PIN_0 | GPIO_PIN_1));

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    UARTStdioConfig(0, 115200, 16000000);
}

//Initialize the SW1 and SW2 push buttons.
void initPushButtons() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

//Start the phase timer used to switch to the lock on phase after
//a certain amount of time has passed.
void startPhaseTimer() {
    IntEnable(INT_TIMER4A);
    TimerEnable(TIMER4_BASE, TIMER_A);
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
}

//Disable and clear the phase timer.
void endPhaseTimer() {
    IntDisable(INT_TIMER4A);
    TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    TimerDisable(TIMER4_BASE, TIMER_A);
    HWREG(TIMER4_BASE + TIMER_O_TAV) = 0;
}

//Check the timer used to switch to the lock on phase.
void checkPhaseTimer(int *phaseTime) {
    *phaseTime = TimerValueGet(TIMER4_BASE, TIMER_A) / 16000;
	//If the time is greater than this value, switch to the lock on phase.
    if (*phaseTime > 20000 && currentState == Lock_On_Wait) {
        lockOnPhase = 1;
        currentState = Lock_On_End;
        reportDistance = 1;
        sendDistance = 1;
        time = 0;
        endPhaseTimer();
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);
    }
    else if (*phaseTime > 20000 && currentState == Send_Transmit) {
        lockOnPhase = 0;
        currentState = Send_End;
        endPhaseTimer();
        time = 0;
        clear();
        writeTransmitFinished();
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);
        PWMGenDisable(PWM0_BASE, PWM_GEN_0); //Disable the servo PWM module.
    }
}

//GPIO interrupt used to check if the push buttons SW1 or SW2 have been
//pushed.
void checkPushButtons() {
    uint8_t readButton;

    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);

	//Check the GPIO input of the push buttons (active low).
    readButton = ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

	//SW2 has been pushed, this enables a receive phase.
    if ((readButton & GPIO_PIN_0) && nodeEnabled) {
        clear();
        writeWaiting();
		//Start the timer used to switch to a lock on phase after a predetermined time.
        startPhaseTimer();
        startTransmit = 0;
        endReceive = 0;
        lockOnPhase = 0;
        currentState = Lock_On_Wait;
		//Turn on the LED corresponding to the receive phase.
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
        SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
        GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
        GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
        initTimer5A();
    }
	//SW1 has been pushed, this enables a lock on phase.
    else if ((readButton & GPIO_PIN_4) && nodeEnabled){
        clear();
        writeFirstNode();
        startPhaseTimer();
        startTransmit = 1;
        endReceive = 1;
        lockOnPhase = 1;
        currentState = Send_Transmit;
		//Turn on the LED corresponding to the lock on phase.
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
        GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4);
        GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_0);
    }

    //If the node has not been enabled, show the user their choice to pick the
    //starting angle and/or start the node (begin tracking).
    if ((readButton & GPIO_PIN_0) == 1 && !nodeEnabled) {
        nodeEnabled = 1; //SW2 starts the node's normal operation.
    }
    //If SW1 is pressed, the angle is incremented by 5 with each press until it
    //reaches 180. If it reaches 180 then the angle can be decremented by 5 with
    //each press. Then, if it reaches back to 0, the angle is again incremented
    //by 5. This can continue until the user chooses to start the node.
    else if (!nodeEnabled) {
        if (incrementAngle)
            startingAngle += 5;
        else
            startingAngle -= 5;

        if (startingAngle == 180) {
            incrementAngle = 0;
        }
        else if (startingAngle == 0) {
            incrementAngle = 1;
        }
        clear();
        writeChooseAngle(startingAngle);
    }
    _delay_ms(250); //Delay for debouncing the button.
}

//Interrupt called when a rising edge or falling edge corresponding to a binary digit
//is encountered.
void ultrasoundReceive()
{
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_2); //Clear the interrupt handler.

	int received = 0;
    char receiveString[2] = "10";
    char lockOnString[2] = "11";

	//Check if binary has been received.
    received = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2);

	//Condition met if a rising edge occurs.
    if (received != 0 && binaryReceived == 0) {
        time = 0;
        binaryReceived = 1;

		//Start the timer used to identify binary digits.
        IntEnable(INT_TIMER2A);
        TimerEnable(TIMER2_BASE, TIMER_A);
        TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

		//End the timer used to check for a long enough logic low. 
		//One that corresponds to the delay between binary digits.
        IntDisable(INT_TIMER3A);
        TimerIntDisable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
        TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
        TimerDisable(TIMER3_BASE, TIMER_A);
        timeOut = TimerValueGet(TIMER3_BASE, TIMER_A) / 16;
        HWREG(TIMER3_BASE + TIMER_O_TAV) = 0;

		//If the logic low is long enough, it is corresponds to the delay
		//used between binary values. Used to prevent noise from changing
		//the timer for binary digits.
        if (timeOut > 2000) {

			//Clear and store the value of the timer used to identify a binary digit.
            IntDisable(INT_TIMER2A);
            TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
            TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
            TimerDisable(TIMER2_BASE, TIMER_A);
            time = TimerValueGet(TIMER2_BASE, TIMER_A) / 16;
            HWREG(TIMER2_BASE + TIMER_O_TAV) = 0;

            IntEnable(INT_TIMER2A);
            TimerEnable(TIMER2_BASE, TIMER_A);
            TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

            //if (time > 0)
                //uprintf("time: %d\n", time);
        }
    }
	//Condition met if a binary digit was already received.
    else if (binaryReceived == 1) {
		//Start the timer used to check if a logic low lasts for long enough
		//to qualify as the delay used between binary values.
        TimerEnable(TIMER3_BASE, TIMER_A);
        TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
        binaryReceived = 0;
    }

	//Time corresponding to a binary 1.
    if (time > 4400 && time < 8000) {
        verifiedCode[binaryCounter] = '1';
        time = 0;
        ++binaryCounter;
    }
	//Time corresponding to a binary 0.
    else if (time > 3000 && time < 4300) {
        verifiedCode[binaryCounter] = '0';
        time = 0;
        ++binaryCounter;
    }
	//If two binary digits have been identified, check the code.
    if (binaryCounter == 2) {
        binaryCounter = 0;
		//Code used during the lock on phase.
        if (strcmp(verifiedCode, receiveString) == 0) {
            endReceive = 1;
        }
        else if ((strcmp(verifiedCode, lockOnString) == 0) && reportDistance) {
            endReceive = 1;
        }
        else {
            verifiedCode[0] = '0';
            verifiedCode[1] = '0';
        }
        IntDisable(INT_TIMER2A);
        TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
        TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
        TimerDisable(TIMER2_BASE, TIMER_A);
        HWREG(TIMER2_BASE + TIMER_O_TAV) = 0;
    }
}

//Initialize the GPIO interrupts for the binary received and for the
//push buttons.
void initGPIOInterrupts()
{
	//GPIO interrupt for the binary received.
    GPIOIntRegister(GPIO_PORTD_BASE, ultrasoundReceive);
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_2);
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_2,
                GPIO_BOTH_EDGES);
    IntPrioritySet(INT_GPIOD, 0);

	//GPIO interrupt for SW1 (push button).
    GPIOIntRegister(GPIO_PORTF_BASE, checkPushButtons);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4,
                    GPIO_LOW_LEVEL);
    IntPrioritySet(INT_GPIOF, 0xD0);

	//GPIO interrupt for SW2 (push button).
    GPIOIntRegister(GPIO_PORTF_BASE, checkPushButtons);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0,
                        GPIO_LOW_LEVEL);
    IntPrioritySet(INT_GPIOF, 0xD0);
}

//Initialize the timer used for the watch dog.
void initTimer0A()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT_UP);
}

//Initialize the timer used to measure the flight time of an ultrasound
//wave (including the processing time of a receive and transmit on the 
//other node).
void initTimer1A()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT_UP);
}

//Initialize the timer used to measure the length of each binary code 
//received to classify it as a 1 or 0.
void initTimer2A() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT_UP);
}

//Initialize the timer used to check if a logic low is noise/unintended or 
//the delay specified for a binary code.
void initTimer3A() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT_UP);
}

//Initialize the timer used to check the phase time. Used to set the node
//to the lock on phase after a predetermined amount of time.
void initTimer4A() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    TimerConfigure(TIMER4_BASE, TIMER_CFG_ONE_SHOT_UP);
}

void initGPIO() {
	//Set PC5 to read the logic level for binary codes.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_2);

	//Set PE1 and PE2 for output to LEDs signifying a lock on (transmit)
	//phase or a receiving phase. PE1 -> lock on phase, PE2 -> receive phase.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

//Begin the timer for the flight timer of the ultrasound (including transmit
//and receive processing time).
void startFlightTimer() {
    IntEnable(INT_TIMER1A);
    TimerEnable(TIMER1_BASE, TIMER_A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

//End the flight timer and use the value to calculate the distance to the 
//other node.
int endFlightTimer(int foundAngle, int *numReceived) {
    int distance = 0;
    int clockCycles = 0;
    int flightTime = 0;
    int finalDistance = 0;
    int totalAverage = 0;
    int i;

    IntDisable(INT_TIMER1A);
    TimerIntDisable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerDisable(TIMER1_BASE, TIMER_A);
    clockCycles = TimerValueGet(TIMER1_BASE, TIMER_A); //Total clock cycles measured.
	//If the angle was found, calculate the flight time of the ultrasound
	//waves. Subtract the processing time for the transmit and receive
	//from this flight time.
    if (foundAngle || reportDistance) {
        flightTime = ((clockCycles / 16) / 2) - 58400;
    }
	//Otherwise, calculate the total time for a binary verification to allow
	//for easier angle lock on (the correct angle to the other node should 
	//still correspond to the angle at which the minimum distance is found).
    else
        flightTime = (clockCycles / 16);
    distance = flightTime * 0.0343;
    HWREG(TIMER1_BASE + TIMER_O_TAV) = 0;
    uprintf("flight time: %d sec, distance: %d m\n", flightTime, distance);
    uprintf("clock cycles: %d\n", clockCycles);

    if (distance != 0)
        receiveDistanceList[receiveIndex] = distance;

    if (reportDistance && (distance > 0 && distance < 100)) {
        distanceList[*numReceived] = distance;
        ++(*numReceived);
        time = 0;
    }

    if (*numReceived >= 1) {
        for (i = 0; i < *numReceived; ++i) {
            if (i > 1) {
                if (absValue(distanceList[i], distanceList[i - 1]) >= 10) {
                    distanceList[i] = distanceList[i - 2];
                }
            }
            totalAverage += distanceList[i];
        }
        finalDistance = totalAverage / *numReceived;
        writeDistance(finalDistance);
    }

    return distance;
}

//Initiate the watch dog timer.
void startWatchDog()
{
    IntEnable(INT_TIMER0A);
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

//Check the watch dog timer value.
uint32_t checkWatchDog()
{
    uint32_t watchTime = TimerValueGet(TIMER0_BASE, TIMER_A) / 16000;

    return watchTime;
}

//Clear the watch dog timer.
void clearWatchDog()
{
    IntDisable(INT_TIMER0A);
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerDisable(TIMER0_BASE, TIMER_A);
    HWREG(TIMER0_BASE + TIMER_O_TAV) = 0;
}

//Rotate the servo from 0 to 180 degrees and find the minimum distance. Then, go to the angle
//that the minimum distance was found at. Rotate 60 degrees around this angle (+30 above, -30
//below) and find the minimum distance. Then, rotate 20 degress around this angle (+10 above, -10 below). 
//Repeat for ~7 degrees and the angle corresponding to the minimum distance at this range is the angle 
//locked onto.
void angleLock(int index, int *foundAngle, int distance, int *previousDistanceMin,
               int *clockWise, int *counterClockWise, int *minIndex,
               uint32_t *angleUpperLimit, uint32_t *angleLowerLimit, uint32_t *previousAngle,
               int *previousDistance, int *distanceBefore, int *findNextDistance)
{
    uint32_t angle;
    int restingAngle = 90;

	//Rotate the servo clockwise and counter clockwise until the angle is locked onto.
    if (!*foundAngle) { 
        if (startingAngle == 0)
            startingAngle = 1000;
		//Rotate counter clockwise.
        if (*counterClockWise == 1) {
            angle = *previousAngle; //Set the angle to the previous angle.
            angleList[index] = angle; //Place the current angle into an array.

			//If the distance is less than the previous distance, keep track of the index
			//corresponding to this new minimum distance.
            if (distance > 0 && distance < *previousDistanceMin) {
                *minIndex = index;
                *previousDistanceMin = distance;
            }

            angle += 5; //Increment the angle by 5 degrees.

			//Rotate the servo to the new angle.
            rotateServo(angle);

            _delay_ms(80); //Delay needed for the servo to physically rotate.

			//If the upper angle limit is reached, rotate the opposite way.
            if (angle >= *angleUpperLimit) {
                *clockWise = 1;
                *counterClockWise = 0;
            }

            if (angle == startingAngle) {
                startingAngle = 1000; //So this condition can't be met again.
                *clockWise = 1;
                *counterClockWise = 0;
                *angleUpperLimit = angleList[*minIndex] + (*angleUpperLimit - *angleLowerLimit) / 6;
                *angleLowerLimit = angleList[*minIndex] - (*angleUpperLimit - *angleLowerLimit) / 6;
                //If the angle limits are pass the boundaries, round them to their respective
                //boundary values.
                if (*angleUpperLimit > 180) {
                    *angleUpperLimit = 180;
                }
                angle = angleList[*minIndex];
                *previousAngle = angle;
            }
        }
        else if (*clockWise == 1) {
            angle = *previousAngle; //Set the angle to the previous angle.
            angleList[index] = angle; //Place the current angle into an array.

            //If the distance is less than the previous distance, keep track of the index
            //corresponding to this new minimum distance.
            if (distance > 0 && distance < *previousDistanceMin) {
                *minIndex = index;
                *previousDistanceMin = distance;
            }

            angle -= 5; //Decrement the angle by 5 degrees.

			//Rotate the servo to the new angle.
            rotateServo(angle); 

            _delay_ms(80); //Delay needed for the servo to physically rotate.

			//If the lower angle limit is reached, rotate the opposite way. Also divide
			//the angle limits by three (centered around the new angle).
            if (angle <= *angleLowerLimit && startingAngle > 500) {
                rotateServo(restingAngle);
				//If the range is less than 10, the angle to lock onto was found. Rotate this angle
				//and update the foundAngle boolean.
                _delay_ms(100);
                *foundAngle = 1;
                angle = angleList[*minIndex];
            }
            else if (angle <= *angleLowerLimit) {
                *clockWise = 0;
                *counterClockWise = 1;
            }
        }
        *previousAngle = angle; //Keep the current angle for the next function call.
        if (distance != 0) {
            *previousDistance = distance;
            distanceList[index] = distance;
        }
    }
}

//A binary code has been received and verified. Now a transmission of binary can
//occur.
void finishReceive(foundAngle) {
    GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_2);
    startTransmit = 1;
}

//Phase for receiving binary digits.
void receivePhase(int foundAngle, int numTransmits, int *findDistance) {
    uint32_t watchDogTime = 0;

	//Enable the GPIO interrupt used to receive binary.
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_2);
    startTransmit = 0;
    watchDogTime = checkWatchDog(); //Check the watch dog timer.

    //If the watch dog timer gives a timer that reflects a missed receive,
	//send another binary code.
    if (watchDogTime > 140 && lockOnPhase) {
        GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_2);
        startTransmit = 1;
        endReceive = 1;
        clearWatchDog();
    }
    else if (watchDogTime > 250 && !lockOnPhase) {
        time = 0;
        timeOut = 0;
    }
	//If the angle has been locked onto, start reading multpile
	//distances.
    else if (foundAngle && numTransmits == 0) {
        //GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_2);
        //startTransmit = 1;
        //endReceive = 1;
        //*findDistance = 1;
        PWMGenDisable(PWM0_BASE, PWM_GEN_0); //Disable the servo PWM module.
        time = 0;
    }
}

//Transmit the lock on phase binary code to the other node.
void transmitPhase(char binaryCode[], int n, int foundAngle) {
    _delay_ms(100); //Delay needed to allow for a proper receive.

    sendBinary(binaryCode, n); //Send the binary code.

    //Start the timer of the ultrasound's flight including receiving and 
	//transmission processing on the other node.
    startFlightTimer();
	//Start the watch dog timer to send another transmission after a
	//predetermined amount of time if the current transmission has not
	//been received.
    startWatchDog(); 

    endReceive = 0; //End the receive of a binary digit.
}

//Obtain multiple readings of the distance to the other node.
void getDistance(int *numTransmits, int *findDistance, int distance,
                 int minIndex, int *numReadings)
{
	//If the angle has been locked onto, send multiple transmissions
    //for the other side to find the distance.
    if (sendDistance) {
        ++*numTransmits; //Update the number of transmits performed.
        time = 0;
		//numTransmits represents the amount of distance readings taken. 
        if (*numTransmits == 100) {
            *numTransmits = 200; //Make sure this condition is never met again.
            sendDistance = 0; //Don't poll for any more distance readings.
            lockOnPhase = 0; //Finish the lock on phase.
            endReceive = 0; //Open the node for receiving.
            startTransmit = 0; 
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0); //Turn off the receiving phase LCD.
			//Light up the lock on phase LCD.
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);
            time = 0; //Ensure the time to measure a binary bit is reset.
            PWMGenDisable(PWM0_BASE, PWM_GEN_0); //Disable the servo PWM module.
            _delay_ms(100);
        }
    }
}

int main(void)
{
    char binaryCode[3] = "101"; //Binary code used for a typical receive.
    //Binary code used when lock on is finished and the other node needs to lock on.
    int index = 0; //Index for the angeList array. 
    int findDistance = 0; //Boolean used to enable a distance reading.
    int numTransmits = 0; //Keep track of the number of transmits when reading distances.
    int distance = 0; //Distance to the other node.
    //Minimum distance during a lock on phase, used to find lock on angle.
    int previousDistanceMin = 100000;
    int clockWise = 0; //Boolean used to specify the direction of the servo's rotation.
    int counterClockWise = 1; //Boolean used to specify the direction of the servo's rotation.
    int foundAngle = 0; //Boolean used to check if the angle has been found.
	//Keep track of the index to the angle array that corresponds to the minimum distance.
    int minIndex = 0; 
    uint32_t angleUpperLimit = 180; //Upper limit of the servo's rotation range.
    uint32_t angleLowerLimit = 0; //Lower limit of the servo's rotation range.
    uint32_t previousAngle = 0; //Previous angle of the servo's rotation.
    int previousDistance = 10000000;
    int distanceBefore = 0;
    int findNextDistance = 0;
	//Used with a timer to switch to a lock on phase after a predetermined amount of time.
    int phaseTime = 0;
    int numReadings = 0; //Keep track of the number of distance readings.
    int numReceived = 0;
    int i;

    for (i = 0; i < 150; ++i) {
        receiveDistanceList[i] = 100000;
    }

	//Use an external cystal oscillator running at 16 MHz.
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	initUART(); //Initialize UART to allow for uprintf().
	//Initialize the push buttons that specify starting with a receive or a transmit.
	initPushButtons();

	initGPIO(); //Initialize all GPIO outputs and inputs needed.
	//Initialize GPIO interrupts used for finding bindary codes and for checking
	//the push butttons.
	initGPIOInterrupts();

	initTimer0A(); //Timer used for watch dog.
 	//Timer used to find the flight time of the ultrasound (including receive and transmit processing).
	initTimer1A();
	initTimer2A(); //Timer used to measure binary codes.
	//Timer used to ensure noise and dips in logic levels does not trick the timing of binary codes.
	initTimer3A(); 
	initTimer4A(); //Timer used to switch to lock on phase after a predetermined amount of time.

	initI2C(); //Intialize I2C.
	initLCD(); //Initialize the LCD.

	initTransmitter(400, 200); //Initialize the PWM for ultrasound transmission.
	initServo(); //Initalize the PWM for the servo.

	//Tell the user to choose the starting angle.
	writeStartStatement();
	_delay_ms(2000);

	clear();
	//Show the choice for updating the angle (SW1) or beginning normal operation (SW2).
	writeChooseAngle(startingAngle);

	//Wait until the user picks an angle and enables the tracking system.
	while (!nodeEnabled) {
	}

	//rotateServo(startingAngle + 10);
	//_delay_ms(200);
	rotateServo(startingAngle); //Rotate the servo to the starting angle.

	_delay_ms(300);

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);

	previousAngle = startingAngle; //Start the sweep at the starting angle.

	clear();
	writeInitialState(); //Show what each push button is for on the LCD.

    while(1)
    {
		//Binary was successfully received, prepare for a transmit, store the distance
		//and clear the watch dog.
        if (endReceive == 1) {
            finishReceive(foundAngle);
            distance = endFlightTimer(foundAngle, &numReceived);
            clearWatchDog();
        }
		//Receive binary or set up another transmit after a predetermined amount of time has
		//passed.
        else {
            receivePhase(foundAngle, numTransmits, &findDistance);
        }
		//Transmit the binary code used during the lock on phase.
        if (startTransmit == 1) {
            if (findDistance && sendDistance)
               _delay_ms(100);

            transmitPhase(binaryCode, 3, foundAngle); //Transmit the binary code.

			//When in the lock on phase, move the servo until the angle is locked onto.
            if (lockOnPhase) {
                angleLock(index, &foundAngle, distance,
                          &previousDistanceMin, &clockWise, &counterClockWise,
                          &minIndex, &angleUpperLimit, &angleLowerLimit,
                          &previousAngle, &previousDistance, &distanceBefore,
                          &findNextDistance);
                ++index; //Increase the index for the angleList array.
            }

			//If the angle has been locked onto, transmit binary through ultrasound to obtain
			//multiple distance readings.
            getDistance(&numTransmits, &findDistance, distance, minIndex, &numReadings);

        }
 		//Check if the node needs to switch to the lock on phase. Based upon the timing of
		//the current receiving phase.
        checkPhaseTimer(&phaseTime);
    }
}
