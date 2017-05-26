#include "lpc17xx.h"
#include "GPIO.h"
#include <stdio.h>          // printf
static LPC_GPIO_TypeDef * LPC_GPIO[5] = { LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3, LPC_GPIO4  };


void GPIOSetValue( uint8_t portNum, uint32_t pinNum, uint32_t pinVal )
{
  if (pinVal == LOW)
  {
	  LPC_GPIO[portNum]->FIOCLR |= (1<<pinNum);
  }
  else if (pinVal >= HIGH)
  {
	  LPC_GPIO[portNum]->FIOSET |= (1<<pinNum);
  }
}

void GPIOSetDir( uint8_t portNum, uint32_t pinNum, uint32_t pinDir )
{
  if(pinDir == OUTPUT)
	LPC_GPIO[portNum]->FIODIR |= 1<<pinNum;
  else if(pinDir == INPUT)
	LPC_GPIO[portNum]->FIODIR &= ~(1<<pinNum);
}

void GPIOSetMode(uint8_t portNum, uint32_t pinNum, uint32_t pinMode)
{

	switch (portNum)
		{
			case 0:

				if (pinNum < 16 )
				{
					pinNum = pinNum << 1;
					LPC_PINCON->PINSEL0 &= ~(0b11<<pinNum);
					LPC_PINCON->PINSEL0 |= pinMode<<pinNum;
				}
				else if (pinNum > 15)
				{
					pinNum = pinNum - 16;
					pinNum = pinNum << 1;
					LPC_PINCON->PINSEL1 &= ~(0b11<<pinNum);
					LPC_PINCON->PINSEL1 |= pinMode<<pinNum;
				}

			break;

			case 1:

				if (pinNum < 16 )
				{
					pinNum = pinNum << 1;
					LPC_PINCON->PINSEL2 &= ~(0b11<<pinNum);
					LPC_PINCON->PINSEL2 |= pinMode<<pinNum;
				}
				else if (pinNum > 15)
				{
					pinNum = pinNum - 16;
					pinNum = pinNum << 1;
					LPC_PINCON->PINSEL3 &= ~(0b11<<pinNum);
					LPC_PINCON->PINSEL3 |= pinMode<<pinNum;
				}

			break;

			default:;
		}


}

void GPIOSetPull( uint8_t portNum, uint32_t pinNum, uint32_t pinMode)
{
	//no Pull
	if (pinMode == 0)
	{
		pinMode = 0b10;
	}
	//Pull up
	else if(pinMode == 1)
	{
		pinMode = 0b00;
	}
	//Pull down
	else if(pinMode == 2)
	{
		pinMode = 0b11;
	}
	// Repeater Mode
	else if (pinMode == 3)
	{
		pinMode = 0b01;
	}



	switch (portNum)
	{
		case 0:

			if (pinNum < 16 )
			{
				pinNum = pinNum << 1;
				LPC_PINCON->PINMODE0 |= pinMode<<pinNum;
			}
			else if (pinNum > 15)
			{
				pinNum = pinNum - 16;
				pinNum = pinNum << 1;
				LPC_PINCON->PINMODE1 |= pinMode<<pinNum;
			}

		break;

		case 1:

			if (pinNum < 16 )
			{
				pinNum = pinNum << 1;
				LPC_PINCON->PINMODE2 |= pinMode<<pinNum;
			}
			else if (pinNum > 15)
			{
				pinNum = pinNum - 16;
				pinNum = pinNum << 1;
				LPC_PINCON->PINMODE3 |= pinMode<<pinNum;
			}

		break;

		case 2:

			if (pinNum < 14 )
			{
				pinNum = pinNum << 1;
				LPC_PINCON->PINMODE4 |= pinMode<<pinNum;
			}

		break;

		case 3:

			if (pinNum == 25)
			{
				LPC_PINCON->PINMODE7 |= pinMode<<18;
			}
			else if (pinNum == 26)
			{
				LPC_PINCON->PINMODE7 |= pinMode<<20;
			}

		break;

		case 4:
			if (pinNum == 28)
			{
				LPC_PINCON->PINMODE9 |= pinMode<<24;
			}
			else if (pinNum == 29)
			{
				LPC_PINCON->PINMODE9 |= pinMode<<26;
			}

			break;

		default:
			;

	}
}

uint32_t GPIOGetValue (uint8_t portNum, uint32_t pinNum)
{
    uint32_t portVal;
    LPC_GPIO[portNum]->FIOMASK = ~(1<<pinNum);
    portVal = LPC_GPIO[portNum]->FIOPIN;
    portVal = portVal >> pinNum;
    LPC_GPIO[portNum]->FIOMASK = 0x00000000;
    return portVal;
}


void GPIOSetInterrupt (uint8_t portNum,uint32_t pinNum,uint8_t edge)
{


	if(edge == Rising)
	{

		if(portNum == 0)
		{
			LPC_GPIOINT->IO0IntEnR |= (1<<pinNum);
		}
		else if(portNum==2)
		{
			LPC_GPIOINT->IO2IntEnR |= (1<<pinNum);
		}

	}
	else if(edge == Falling)
	{

		if(portNum == 0)
		{
			LPC_GPIOINT->IO0IntEnF |= (1<<pinNum);
		}
		else if(portNum==2)
		{
			LPC_GPIOINT->IO2IntEnF |= (1<<pinNum);
		}

	}

	NVIC_EnableIRQ(EINT3_IRQn);

}


void GPIODisInterrupt (uint8_t portNum,uint32_t pinNum,uint8_t edge)
{

	if(edge == Rising)
	{

		if(portNum == 0)
		{
			LPC_GPIOINT->IO0IntEnR &= ~(1<<pinNum);
		}
		else if(portNum==2)
		{
			LPC_GPIOINT->IO2IntEnR &= ~(1<<pinNum);
		}

	}
	else if(edge == Falling)
	{

		if(portNum == 0)
		{
			LPC_GPIOINT->IO0IntEnF &= ~(1<<pinNum);
		}
		else if(portNum==2)
		{
			LPC_GPIOINT->IO2IntEnF &= ~(1<<pinNum);
		}

	}

}

uint32_t GPIODetect(uint8_t portNum,uint8_t edge)
{

	if(edge == Rising)
	{

		if(portNum == 0)
		{
			return(LPC_GPIOINT->IO0IntStatR);
		}
		else if(portNum==2)
		{
			return(LPC_GPIOINT->IO2IntStatR);
		}

	}
	else if(edge == Falling)
	{

		if(portNum == 0)
		{
			return(LPC_GPIOINT->IO0IntStatF);
		}
		else if(portNum==2)
		{
			return(LPC_GPIOINT->IO2IntStatF);
		}

	}

	return 0;
}

void GPIOIntClear()
{
	LPC_GPIOINT->IO0IntClr = Port0Clear;
	LPC_GPIOINT->IO2IntClr = Port2Clear;
}




