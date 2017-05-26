/******************************************************************************
File name   				:gpio.cpp
Purpose of the File			:It contains the device driver definitions for GPIO related
							:functionalities
Version No & Revision no	:1.0
Author						:JMM
Date Written				:02-14-17
******************************************************************************/

#include "gpio.hpp"
uint32_t basePortAddress[] = { LPC_GPIO0_BASE, LPC_GPIO1_BASE, LPC_GPIO2_BASE, LPC_GPIO3_BASE, LPC_GPIO4_BASE};

/******************************************************************************
Function Name				:setInputDir()
Purpose of the Function		:Configure a particular pin as input
Author						:JMM
Date Written				:02-14-17
@param 						:port number and bit position
******************************************************************************/
void GPIO_CUSTOM::setInputDir( uint8_t port, uint8_t position )
{
	volatile LPC_GPIO_TypeDef *GPIOPtr = (LPC_GPIO_TypeDef*) basePortAddress[port];
	GPIOPtr->FIODIR &= ~(1<<position);

}

/******************************************************************************
Function Name				:setOutputDir()
Purpose of the Function		:Configure a particular pin as output
Author						:JMM
Date Written				:02-14-17
@param 						:port number and bit position
******************************************************************************/
void GPIO_CUSTOM::setOutputDir( uint8_t port, uint8_t position )
{
	volatile LPC_GPIO_TypeDef *GPIOPtr = (LPC_GPIO_TypeDef*) basePortAddress[port];
	GPIOPtr->FIODIR |= (1<<position);
}

/******************************************************************************
Function Name				:setPinValue()
Purpose of the Function		:Set a particular pin value as 0 or 1
Author						:JMM
Date Written				:02-14-17
@param 						:port number, bit position and value to be set
******************************************************************************/
void GPIO_CUSTOM::setPinValue( uint8_t port, uint8_t position, uint8_t value )
{

    volatile LPC_GPIO_TypeDef *GPIOPtr = (LPC_GPIO_TypeDef*) basePortAddress[port];
	if (value == 0)
	{
		GPIOPtr->FIOCLR = (1<<position);
	}
    else if (value >= 1)
	{
    	GPIOPtr->FIOSET = (1<<position);
	}
}

/******************************************************************************
Function Name				:togglePinValue()
Purpose of the Function		:Toggle a particular pin from 0 to 1
Author						:JMM
Date Written				:02-14-17
@param 						:port number and bit position
******************************************************************************/
void GPIO_CUSTOM::togglePinValue( uint8_t port, uint8_t position )
{
    volatile LPC_GPIO_TypeDef *GPIOPtr = (LPC_GPIO_TypeDef*) basePortAddress[port];

    GPIOPtr->FIOCLR = (1<<position);
    delay_ms(1000);
    GPIOPtr->FIOSET = (1<<position);

}

/******************************************************************************
Function Name				:readPinValue()
Purpose of the Function		:Read the pin value
Author						:JMM
Date Written				:02-14-17
@param 						:port number and bit position
@return						:Bool value true or false
******************************************************************************/
bool GPIO_CUSTOM::readPinValue( uint8_t port, uint8_t position )
{
	volatile LPC_GPIO_TypeDef *GPIOPtr = (LPC_GPIO_TypeDef*) basePortAddress[port];

	bool val;
	val = (GPIOPtr->FIOPIN) & (1 << position);
	if(val)
		return 1;
	else
		 return 0;
}

/******************************************************************************
Function Name				:enablePullUp()
Purpose of the Function		:Enable pull up for pin
Author						:JMM
Date Written				:02-14-17
@param 						:port number and bit position
******************************************************************************/
void GPIO_CUSTOM::enablePullUp( uint8_t port, uint8_t position )
{
    volatile uint32_t *mode = &(LPC_PINCON->PINMODE0);
    mode += (2 * port);
    *mode &= ~(3 << ( 2 * position));
}

/******************************************************************************
Function Name				:enablePullDown()
Purpose of the Function		:Enable pull down for pin
Author						:JMM
Date Written				:02-14-17
@param 						:port number and bit position
******************************************************************************/
void GPIO_CUSTOM::enablePullDown( uint8_t port, uint8_t position )
{
    volatile uint32_t *mode = &(LPC_PINCON->PINMODE0);
    mode += (2 * port);
    *mode |= (3 << (2*position));
}


