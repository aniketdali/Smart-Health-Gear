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
 * @brief This is the application entry point.
 */
#include <stdio.h>
#include <iostream>
#include "utilities.h"
#include "gpio.hpp"
#include "eint3.h"
#include "io.hpp"
#include "queue.h"
#include "tasks.hpp"
#include "display.hpp"
#include <i2c2.hpp>
using namespace std;


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
