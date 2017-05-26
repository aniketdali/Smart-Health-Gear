/*
 *
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
 *     You can reach the main author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 *
 *     The file has been modified by Aniket Dali for Academic project made for San Jose State University
 *     You can reach the current author of this software at:
 *     aniket.dali@sjsu.edu
 */


/*****************************************************************************
$Work file     : main.cpp $
Description    : This file contains the Initialization of display and sensor tasks.
Project(s)     : Smart Health Gear
Compiler       : Cross ARM GCC
OS			   : RTOS
Original Author: $ Preetpal Kang
$Author        : $ Aniket Dali
$Date          : $ 26 May 2017
$Revision      : 1.0 $
*****************************************************************************/
/****************************************************************************/
/*                       INCLUDE FILES                                      */
/****************************************************************************/

#include "io.hpp"
#include "queue.h"
#include "tasks.hpp"
#include "display.hpp"



/*----------------------------------------------------------------------------
Function    :  main
Inputs      :  None
Processing  :  This function is the main loop of the program
Outputs     :  None
Returns     :  None
Notes       :  None
----------------------------------------------------------------------------*/
int main()
{
	scheduler_add_task(new heartRate(PRIORITY_LOW));
	scheduler_add_task(new display_Task(PRIORITY_MEDIUM));
	scheduler_add_task(new button_Task(PRIORITY_MEDIUM));
	scheduler_add_task(new tempMeasure(PRIORITY_LOW));
    scheduler_add_task(new orient_compute(PRIORITY_LOW));
    scheduler_add_task(new orient_process(PRIORITY_LOW));
    scheduler_start();
    return 0;
}
/*===================================================================
// $Log: $1.0 AVD:Added comments to increase the readability
//
//--------------------------------------------------------------------*/
