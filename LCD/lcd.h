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
 * Convert a floating point number to a string.
 */
extern char *floatToString(float input);

/*
 * Clear the LCD screen of all characters.
 */
extern void clear(void);

#endif
