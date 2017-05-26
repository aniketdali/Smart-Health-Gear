/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief UART0 Interrupt driven IO driver
 * @ingroup Drivers
 */
#ifndef SPI0_HPP_
#define SPI0_HPP_
#include <stdint.h>
#include <stdio.h>
#include "utilities.h"
#include "LPC17xx.h"
#include "gpio.hpp"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"


class SPI0_task : public scheduler_task
{
    public:

		void set_output_dir(void)
		{
			gpio_pin.setOutputDir(0, 6);
		}


	    SPI0_task(uint8_t priority) : scheduler_task("task", 2000, priority)
        {
	    	set_output_dir();

            /* Nothing to init */
        }
	    char spi0_ExchangeByte(char out)
	    {
	        LPC_SSP1->DR = out;
	        while(LPC_SSP1->SR & (1 << 4)); // Wait until SSP is busy
	        return LPC_SSP1->DR;
	    }
        bool init(void)
        {
        	LPC_SC->PCONP |= (1 << 10);     // SPI1 Power Enable
        	LPC_SC->PCLKSEL0 &= ~(3 << 20); // Clear clock Bits
        	LPC_SC->PCLKSEL0 |=  (1 << 20); // CLK / 1

        	// Select MISO, MOSI, and SCK pin-select functionality
        	LPC_PINCON->PINSEL0 &= ~( (3 << 14) | (3 << 16) | (3 << 18) );
        	LPC_PINCON->PINSEL0 |=  ( (2 << 14) | (2 << 16) | (2 << 18) );

        	LPC_SSP1->CR0 = 7;          // 8-bit mode
        	LPC_SSP1->CR1 = (1 << 1);   // Enable SSP as Master
        	LPC_SSP1->CPSR = 8;         // SCK speed = CPU / 8
            return true;
        }

        void CS_select(void)
        {
        	gpio_pin.setPinValue(0, 6, 0);
        }

        void CS_deselect(void)
        {
        	gpio_pin.setPinValue(0, 6, 1);
        }

        bool run(void *p)
        {
        	        char data;
        			SPI0_task spi0_obj(1);
        			spi0_obj.CS_select();

        			for(int i =0; i<5; i++)
        			{
        				data = spi0_obj.spi0_ExchangeByte(0x9F);
        				if(i==1)
        				{
        					printf("Manufacturing Id is:%xh\n",data);
        				}
        				if(i==2)
        				{
        					printf("Device Id is:%xh (Byte 1)\n",data);
        				}
        				if(i==3)
        				{
        					printf("Device Id is:%xh (Byte 2)\n",data);
        				}
        				//printf("data is:%x\n",spi0_obj.spi0_ExchangeByte(0x9F));
        			    delay_ms(250);

        			}

        			//printf("data is");
        			spi0_obj.CS_deselect();

		/*	printf("data is:%x\n",spi0_ExchangeByte(0x9F));
		    delay_ms(250);*/


            return true;
        }
    private:
        GPIO_CUSTOM gpio_pin;
};


#endif /* SPI0_HPP_ */
