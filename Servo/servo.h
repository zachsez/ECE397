#ifndef SERVO_H
#define SERVO_H

/*
 * Configures the servo PWM on PinF2
 */
extern void initServo(void);

/*
 * Rotates the servo in increments of one degree from
 * 0 to 180 degrees and then decrements by one degree
 * from 180 to 0 degrees.
 */
extern void rotateServo(void);

#endif
