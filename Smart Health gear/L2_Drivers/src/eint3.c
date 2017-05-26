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

#include <stdlib.h>
#include "eint3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct eint3_entry {
    uint32_t pin_mask;
    void_func_t callbackFunction;
    struct eint3_entry* next;
} eint3_struct;


static eint3_struct *rising_list  = NULL;
static eint3_struct *falling_list  = NULL;

/******************************************************************************
Function Name				:handler_eint3()
Purpose of the Function		:Process the callback function based on the interrupt
Author						:JMM
Date Written				:03-14-17
@param 						:no parameters
******************************************************************************/
static void handler_eint3()
{
	eint3_struct *obj = rising_list;
	uint32_t rising2  = LPC_GPIOINT->IO2IntStatR;
	uint32_t falling2  = LPC_GPIOINT->IO2IntStatF;

	while (obj && rising2) {
		LPC_GPIOINT->IO2IntClr = obj->pin_mask;

		   if (obj->pin_mask ) {
			   (obj->callbackFunction)();
			   rising2 &= ~(obj->pin_mask);
			   LPC_GPIOINT->IO2IntClr = obj->pin_mask;
		   }
		   obj = obj->next;
	   };
	eint3_struct *obj1 = falling_list;
	while (obj1 && falling2) {
		LPC_GPIOINT->IO2IntClr = obj1->pin_mask;

		   if (obj1->pin_mask ) {
			   (obj1->callbackFunction)();
			   falling2 &= ~(obj1->pin_mask);
			   LPC_GPIOINT->IO2IntClr = obj1->pin_mask;
		   }
		   obj1 = obj1->next;
	   };
}

/******************************************************************************
Function Name				:EINT3_IRQHandler()
Purpose of the Function		:External interrupt 3 handler
Author						:JMM
Date Written				:03-14-17
@param 						:no parameters
******************************************************************************/
/*void EINT3_IRQHandler(void)
{

	handler_eint3();

}
#ifdef __cplusplus
}
#endif
*/

/******************************************************************************
Function Name				:enable_port2()
Purpose of the Function		:Configure a particular pin of port2 for interrupts
Author						:JMM
Date Written				:03-14-17
@param 						:pin number, type and callback function
******************************************************************************/
void enable_port2(uint8_t pin_num, eint_intr_t type, void_func_t func)
{
	eint3_struct *obj = NULL;

	obj = malloc(sizeof(*obj));

    if ( NULL != obj )
    {
    	obj->callbackFunction = func;
    	obj->pin_mask = (UINT32_C(1) << pin_num);

	if(rising_edge == type)
	{
    	obj->next = rising_list;
    	rising_list = obj;

		LPC_GPIOINT->IO2IntEnR |= obj->pin_mask;
	}
	else
	{
    	obj->next = falling_list;
    	falling_list = obj;
		LPC_GPIOINT->IO2IntEnF |= obj->pin_mask;
	}

	NVIC_EnableIRQ(EINT3_IRQn);
    }
}


