#ifndef LCD_H
#define LCD_H

/*
 * Initialize the I2C peripheral to allow writing to the LCD.
 */
extern void initI2C(void);

/*
 * Initialize the LCD.
 */
extern void initLCD(void);

/*
 * Write a string to the LCD.
 */
extern void write(int x, int y, char data[]);

/*
 * Write an angle to the LCD as a string.
 */
extern void writeAngle(float angle);

/*
 * Write a distance to the LCD as a string.
 */
extern void writeDistance(float distanceLCD);

/*
 * Write to the LCD that the node is looking for
 * its target (lock on phase).
 */
extern void writeLockOn();

/*
 * Write to the LCD that the node is waiting to
 * be locked onto (receive phase).
 */
extern void writeWaiting();

/*
 * Write to the LCD which push button corresponds
 * to lock on phase or receive phase.
 * SW1 -> lock on phase.
 * SW2 -> receive phase.
 */
extern void writeInitialState();

/*
 * Convert a floating point number to a string.
 */
extern char *floatToString(float input);

/*
 * Clear the LCD screen of all characters.
 */
extern void clear(void);

extern void writeStartStatement();

extern void writeChooseAngle();

extern void writeFirstNode();

#endif
