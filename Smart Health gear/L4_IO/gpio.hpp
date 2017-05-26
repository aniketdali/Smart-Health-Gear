
/**
 * @file
 * @ingroup BoardIO
 */
 
#ifndef LPC_GPIO_H__
#define LPC_GPIO_H__

/******************************************************************************
File name   				:gpio.h
Purpose of the File			:It contains the header file for GPIO related
							:functionalities
Version No & Revision no	:1.0
Author						:JMM
Date Written				:02-14-17

******************************************************************************/

#include <stdint.h>
#include "utilities.h"
#include "LPC17xx.h"

/* GPIO_CUSTOM Class definition */
class GPIO_CUSTOM
{
	/* public function declarations */
    public:

        void setInputDir( uint8_t port, uint8_t position );                     /* Sets pin as input pin */
        void setOutputDir( uint8_t port, uint8_t position );                    /* Sets pin as output pin */
        void setPinValue( uint8_t port, uint8_t position, uint8_t value );      /* Sets the pin to logical HIGH or LOW */
        void enablePullUp( uint8_t port, uint8_t position );                    /* Enables pull-up resistor */
        void enablePullDown( uint8_t port, uint8_t position );                  /* Enables pull-down resistor */
        void togglePinValue( uint8_t port, uint8_t position );					/* Toggle pin value */
        bool readPinValue( uint8_t port, uint8_t position );					/* Read pin value */

};



#endif /* GPIO_CUSTOM_H__ */

