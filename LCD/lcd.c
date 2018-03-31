#include "stdlib.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <string.h>
#include <inttypes.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_uart.h"
#include "inc/hw_pwm.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"

int address = 0x27;
int BLEN = 1;

void l_delay_ms(int milliSecond) {
    SysCtlDelay((SysCtlClockGet() / (3 * 1000)) * milliSecond) ;
}

void initI2C(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6); // special I2CSCL treatment for M4F devices
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
    SysCtlDelay(10000);
}

void sendI2C(uint16_t address, uint8_t data1)
{
    //specify that we want to communicate to device address with an intended write to bus
    I2CMasterSlaveAddrSet(I2C1_BASE, address, false);

    //register to be read
    I2CMasterDataPut(I2C1_BASE, data1);

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while (I2CMasterBusy(I2C1_BASE)); //&& !I2CMasterTimeout( I2C0_BASE) );
}

void writeWord(int data){
    int temp = data;
    if ( BLEN == 1 )
        temp |= 0x08;
    else
        temp &= 0xF7;
    sendI2C(address, temp);
}

void sendCommand(int comm){
    int buf;
    // Send bit 7-4 firstly
    buf = comm & 0xF0;
    buf |= 0x04;            // RS = 0, RW = 0, EN = 1
    writeWord(buf);
    l_delay_ms(2);
    buf &= 0xFB;            // Make EN = 0
    writeWord(buf);

    // Send bit 3-0 secondly
    buf = (comm & 0x0F) << 4;
    buf |= 0x04;            // RS = 0, RW = 0, EN = 1
    writeWord(buf);
    l_delay_ms(2);
    buf &= 0xFB;            // Make EN = 0
    writeWord(buf);
}

void sendData(int data){
    int buf;
    // Send bit 7-4 firstly
    buf = data & 0xF0;
    buf |= 0x05;            // RS = 1, RW = 0, EN = 1
    writeWord(buf);
    l_delay_ms(2);
    buf &= 0xFB;            // Make EN = 0
    writeWord(buf);

    // Send bit 3-0 secondly
    buf = (data & 0x0F) << 4;
    buf |= 0x05;            // RS = 1, RW = 0, EN = 1
    writeWord(buf);
    l_delay_ms(2);
    buf &= 0xFB;            // Make EN = 0
    writeWord(buf);
}

void initLCD(void){
    sendCommand(0x33); // Must initialize to 8-line mode at first
    l_delay_ms(5);
    sendCommand(0x32); // Then initialize to 4-line mode
    l_delay_ms(5);
    sendCommand(0x28); // 2 Lines & 5*7 dots
    l_delay_ms(5);
    sendCommand(0x0C); // Enable display without cursor
    l_delay_ms(5);
    sendCommand(0x01); // Clear Screen
    sendI2C(address, 0x08);
}

void clear(void){
    sendCommand(0x01); //clear Screen
}

void write(int x, int y, char data[]){
    int position, i;
    int tmp;
    if (x < 0)  x = 0;
    if (x > 15) x = 15;
    if (y < 0)  y = 0;
    if (y > 1)  y = 1;

    // Move cursor
    position = 0x80 + 0x40 * y + x;
    sendCommand(position);

    tmp = strlen(data);
    for (i = 0; i < tmp; i++){
        sendData(data[i]);
    }
}

void writeAngle(int angle)
{
	char *angleString = (char*)malloc(4 * sizeof(char));
	
	sprintf(angleString, "%0.1d", angle);

	write(0, 1, "Angle: ");
	write(7, 1, angleString);;
	write(12, 1, "deg");

	free(angleString);
}

void writeDistance(int distanceLCD)
{
    char *distanceString = (char*)malloc(4 * sizeof(char));

    sprintf(distanceString, "%0.1d", distanceLCD);

    write(0, 0, "Distance: ");
    write(9, 0, distanceString);
    write(14, 0, "cm");

    free(distanceString);
}

void writeLockOn() {
    write(1, 0, "Finding target");
}

void writeWaiting() {
    write(0, 0, "Waiting to be");
    write(0, 1, "found");
}

void writeInitialState() {
    write(0, 0, "SW1: Send");
    write(0, 1, "SW2: Receive");
}

void writeStartStatement() {
    write(0, 0, "Choose starting");
    write(0, 1, "angle");
}

void writeChooseAngle(int startingAngle) {
    char *angleString = (char*)malloc(4 * sizeof(char));

    sprintf(angleString, "%0.1d", startingAngle);

    write(0, 0, "SW1: Angle = ");
    write(0, 1, "SW2: Begin ");
    write(13, 0, angleString);

    free(angleString);
}

