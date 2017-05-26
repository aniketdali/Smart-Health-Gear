/*
 * gpio.h
 *
 *  Created on: Feb 12, 2017
 *      Author: Aniket Dali
 */

#ifndef L2_DRIVERS_GPIO_H_
#define L2_DRIVERS_GPIO_H_


#define  NOPULL  	0
#define  PULLUP  	1
#define  PULLDOWN  	2
#define  REPEATER   3

#define  FUNC1  	0
#define  FUNC2  	1
#define  FUNC3  	2
#define  FUNC4      3

#define  INPUT 		0
#define  OUTPUT 	1

#define  LOW		0
#define  HIGH		1

#define  Rising     1
#define  Falling    0

#define  Port0Clear (0x66478FCF)
#define  Port2Clear (0x7FF)


/**
 * GPIO Driver
 */


/**
 * GPIOSetDir
 * Inputs: port number, pin number , direction of pin
 */
void GPIOSetDir( uint8_t portNum, uint32_t pinNum, uint32_t pinDir );
/**
 * GPIOSetValue
 * Inputs: port number, pin number , value to be set
 */
void GPIOSetValue( uint8_t portNum, uint32_t pinNum, uint32_t pinVal );
/**
 * GPIOSetPull
 * Inputs: port number, pin number ,
 * PinMode:
 *   NOPULL  	0
 *   PULLUP  	1
 *   PULLDOWN  	2
 *   REPEATER   3
 *
 */
void GPIOSetPull(uint8_t portNum, uint32_t pinNum, uint32_t pinMode);
/**
 * GPIOGetValue
 * Inputs: port number, pin number
 * Output: value at the port-pin ( 0 or 1)
 */
uint32_t GPIOGetValue (uint8_t portNum, uint32_t pinNum);
/**
 * GPIOSetMode
 * Inputs: port number, pin number, pin function
 */
void GPIOSetMode(uint8_t portNum, uint32_t pinNum, uint32_t pinMode);

void GPIOSetInterrupt (uint8_t portNum,uint32_t pinNum,uint8_t edge);

void GPIODisInterrupt (uint8_t portNum,uint32_t pinNum,uint8_t edge);

uint32_t GPIODetect(uint8_t portNum,uint8_t edge);

void GPIOIntClear();

/********************lab 4 *****************************/


#endif /* L2_DRIVERS_GPIO_H_ */
