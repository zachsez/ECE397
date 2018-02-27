/*
 * transmitter.h
 *
 *  Created on: Feb 22, 2018
 *      Author: Zach Szczesniak
 */
#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#define ZEROTIME 1000   //Time active for zero bit in ms
#define ONETIME  2000   //Time active for one bit in ms
#define DEADTIME 500    //Time inactive between bits in ms


/*
 *  Sends a single binary bit of data
 *  Carries out a dead time period then high time period based on bit
 *  Return code 0 success
 *         code 1 bit is not 0 or 1
 */
extern int sendBit(char bit);


/*
 * Sends an array of binary data
 * Return code 0 success
 * Return code 1x failure in sendBit
 * Return code 2  n is less than 0
 */
extern int sendBinary(char binaryCode[], int n);


/*
 * Setup the square wave output on PF1
 */
extern void initTransmitter(uint32_t period, uint32_t pulseWidth, int firstPhasePassed);


/*
 * Set the period (clock ticks)
 */
extern void setPeriod(uint32_t period);


/*
 * Set the pulse Width (clock ticks)
 */
extern void setPulseWidth(uint32_t pulseWidth);

#endif
