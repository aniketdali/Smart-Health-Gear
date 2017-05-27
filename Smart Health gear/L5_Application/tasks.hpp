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
 * @brief Contains FreeRTOS Tasks
 */
#ifndef TASKS_HPP_
#define TASKS_HPP_

#include <iostream>
#include <string>
#include <sstream>
#include <bitset>

#include "scheduler_task.hpp"
#include "soft_timer.hpp"
#include "command_handler.hpp"
#include "wireless.h"
#include "char_dev.hpp"

#include "FreeRTOS.h"
#include "semphr.h"
#include "io.hpp"
#include "i2c1.hpp"

#include <stdint.h>
#include <stdio.h>
#include "utilities.h"
#include "LPC17xx.h"
#include "gpio.hpp"
#include "queue.h"
#include  "display.hpp"
#include "fat/disk/spi_flash.h"
#include "fat/ff.h"
#include "Thermistor.hpp"
#include "i2c1.hpp"
#include <algorithm>
#define	SS(fs)	((fs)->ssize)
using namespace std;

/**
 * Terminal task is our UART0 terminal that handles our commands into the board.
 * This also saves and restores the "disk" tele-metry.  Disk tele-metry variables
 * are automatically saved and restored across power-cycles to help us preserve
 * any non-volatile information.
 */
class terminalTask : public scheduler_task
{
    public:
        terminalTask(uint8_t priority);     ///< Constructor
        bool regTlm(void);                  ///< Registers telemetry
        bool taskEntry(void);               ///< Registers commands.
        bool run(void *p);                  ///< The main loop

    private:
        // Command channels device and input command str
        typedef struct {
            CharDev *iodev; ///< The IO channel
            str *cmdstr;    ///< The command string
            bool echo;      ///< If input should be echo'd back
        } cmdChan_t;

        VECTOR<cmdChan_t> mCmdIface;   ///< Command interfaces
        CommandProcessor mCmdProc;     ///< Command processor
        uint16_t mCommandCount;        ///< terminal command count
        uint16_t mDiskTlmSize;         ///< Size of disk variables in bytes
        char *mpBinaryDiskTlm;         ///< Binary disk telemetry
        SoftTimer mCmdTimer;           ///< Command timer

        cmdChan_t getCommand(void);
        void addCommandChannel(CharDev *channel, bool echo);
        void handleEchoAndBackspace(cmdChan_t *io, char c);
        bool saveDiskTlm(void);
};

/**
 * Remote task is the task that monitors the IR remote control signals.
 * It can "learn" remote control codes by typing "learn" into the UART0 terminal.
 * Thereafter, if a user enters a 2-digit number through a remote control, then
 * your function handleUserEntry() is called where you can take an action.
 */
class remoteTask : public scheduler_task
{
    public:
        remoteTask(uint8_t priority);   ///< Constructor
        bool init(void);                ///< Inits the task
        bool regTlm(void);              ///< Registers non-volatile variables
        bool taskEntry(void);           ///< One time entry function
        bool run(void *p);              ///< The main loop

    private:
        /** This function is called when a 2-digit number is decoded */
        void handleUserEntry(int num);
        
        /**
         * @param code  The IR code
         * @param num   The matched number 0-9 that mapped the IR code.
         * @returns true if the code has been successfully mapped to the num
         */
        bool getNumberFromCode(uint32_t code, uint32_t& num);

        uint32_t mNumCodes[10];      ///< IR Number codes
        uint32_t mIrNumber;          ///< Current IR number we're decoding
        SemaphoreHandle_t mLearnSem; ///< Semaphore to enable IR code learning
        SoftTimer mIrNumTimer;       ///< Time-out for user entry for 1st and 2nd digit
};

/**
 * Nordic wireless task to participate in the mesh network and handle retry logic
 * such that packets are resent if an ACK has not been received
 */
class wirelessTask : public scheduler_task
{
    public:
        wirelessTask(uint8_t priority) :
            scheduler_task("wireless", 512, priority)
        {
            /* Nothing to init */
        }

        bool run(void *p)
        {
            wireless_service(); ///< This is a non-polling function if FreeRTOS is running.
            return true;
        }
};

/**
 * Periodic callback dispatcher task
 * This task gives the semaphores that end up calling functions at periodic_callbacks.cpp
 */
class periodicSchedulerTask : public scheduler_task
{
    public:
        periodicSchedulerTask(void);
        bool init(void);
        bool regTlm(void);
        bool run(void *p);

    private:
        bool handlePeriodicSemaphore(const uint8_t index, const uint8_t frequency);
};

/*
 * SSP0 communication task
 * Contains all the functions related to the task
 */

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
	    char spi0_ExchangeByte(char sent)
	    {
	        LPC_SSP1->DR = sent;
	        while(LPC_SSP1->SR & (1 << 4));
	        return LPC_SSP1->DR;
	    }

	    void power_Enable()
	    {
	    	LPC_SC->PCONP |= (1 << 10);     // Power Enable
	    }

	    void pinsel_Pclock()
	   	{

        	LPC_SC->PCLKSEL0 &= ~(3 << 20); // Clear clock
        	LPC_SC->PCLKSEL0 |=  (1 << 20); // Set CLK / 1

        	// Select pin-select functionality
        	LPC_PINCON->PINSEL0 &= ~( (3 << 14) | (3 << 16) | (3 << 18) );
        	LPC_PINCON->PINSEL0 |=  ( (2 << 14) | (2 << 16) | (2 << 18) );
	   	}
	    void prescalar()
	    {
        	LPC_SSP1->CR0 = 7;          // 8-bit mode
        	LPC_SSP1->CR1 = (1 << 1);   // Enable as Master
        	LPC_SSP1->CPSR = 8;         // Speed = CPU / 8
	    }

        bool init(void)
        {
        	power_Enable();
        	pinsel_Pclock();
        	prescalar();
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

       static void display_MID_DID(void)
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

        		delay_ms(250);

        	}


        	spi0_obj.CS_deselect();

        }
       static void signature_read(void)
       {
			SPI0_task spi0_obj(1);
			spi0_obj.CS_select();
			char data;

			data = spi0_obj.spi0_ExchangeByte(0x03);
			data = spi0_obj.spi0_ExchangeByte(0x00);
			data = spi0_obj.spi0_ExchangeByte(0x01);
			data = spi0_obj.spi0_ExchangeByte(0xFE);

			printf("\nSignature details\n");
			printf("Signature (Byte 1) 0x:%x\n",spi0_obj.spi0_ExchangeByte(0x03));
			printf("Signature (Byte 2) 0x:%x\n",spi0_obj.spi0_ExchangeByte(0x01));


			spi0_obj.CS_deselect();
       }

       static void sector_details(void)
       {
			SPI0_task spi0_obj(1);
			spi0_obj.CS_select();
			void* pData;
			FATFS *fs;
			char data;
			//cout<<flash_get_mem_size_bytes() ;
			cout<<flash_ioctl(1 , &fs);
			printf("cluster (Byte 1) %d\n",fs->csize);
            cout<< "cluster"<<hex<<fs->csize<<endl;
            cout<< "sector"<<hex<<fs->ssize<<endl;
            cout<< "sector"<<hex<<fs->fsize<<endl;
			cout<<flash_get_page_count()<<endl;
			cout<<flash_get_page_size()<<endl;

			data = spi0_obj.spi0_ExchangeByte(0x01);
			data = spi0_obj.spi0_ExchangeByte(0x00);
			data = spi0_obj.spi0_ExchangeByte(0x00);
			data = spi0_obj.spi0_ExchangeByte(0x0D);


			printf("signature (Byte 1) 0x:%x\n",spi0_obj.spi0_ExchangeByte(0x00));
			printf("signature (Byte 2) 0x:%x\n",spi0_obj.spi0_ExchangeByte(0x00));


			spi0_obj.CS_deselect();
       }

       static void read_status_display(void)
       {

    	   SPI0_task spi0_obj(1);
    	   char a[8]={0};
    	   spi0_obj.CS_select();

    	   spi0_obj.spi0_ExchangeByte(0xD7);
    	   cout<<"\nByte 1 of status register details:\n";
    	   string first_byte = bitset<8>((int)spi0_obj.spi0_ExchangeByte(0xD7)).to_string();

    	   cout<<"\nPage Size Configuration:"<<first_byte.at(7);
    	   cout<<"\nSector Protection Status:"<<first_byte.at(6);
    	   for(int i = 2; i<6; i++)
    	   {
    		   a[i-2] = first_byte.at(i);
    	   }
    	   cout<<"\nDensity Code:"<<hex<<a;
    	   cout<<"\nCompare Result:"<<first_byte.at(1);
    	   cout<<"\nReady/Busy' Status:"<<first_byte.at(0);


    	   string second_byte = bitset<8>((int)spi0_obj.spi0_ExchangeByte(0xD7)).to_string();
    	   cout<<"\nByte 2 of status register details:\n";
    	   cout<<"\nErase Suspend:"<<second_byte.at(7);
    	   cout<<"\nProgram Suspend Status 1:"<<second_byte.at(6);
    	   cout<<"\nProgram Suspend Status 2:"<<second_byte.at(5);
    	   cout<<"\nSector Lockdown Enabled:"<<second_byte.at(4);
    	   cout<<"\nReserved for future:"<<second_byte.at(3);
    	   cout<<"\nErase/Program Error:"<<second_byte.at(2);
    	   cout<<"\nReserved for future"<<second_byte.at(1);
    	   cout<<"\nReady/Busy' Status:"<<second_byte.at(0);
    	   delay_ms(250);
    	   spi0_obj.CS_deselect();

       }
        bool run(void *p)
        {


           while(1)
           {

           }


            return true;
        }
    private:
        GPIO_CUSTOM gpio_pin;
};

/*
 * UART2 communication task
 * Contains all the functions related to the task
 */

static char receive_Byte()
{
	//while(! (LPC_UART2->LSR & (1 << 0)));
    return LPC_UART2->RBR;
}

class UART2_task : public scheduler_task
{
    public:

	    UART2_task(uint8_t priority) : scheduler_task("task", 2000, priority)
        {

        }
	    static void transmit_Byte(char sent)
	    {
	        LPC_UART2->THR = sent;
	        while(! (LPC_UART2->LSR & (1 << 6)));
	        return;
	    }

	    void power_Enable()
	    {
	    	LPC_SC->PCONP |= (1 << 24);     // Power Enable
	    }

	    void pinsel_Pclock()
	   	{
        	LPC_SC->PCLKSEL1 &= ~(3 << 16); // Clear clock
        	LPC_SC->PCLKSEL1 |=  (1 << 16); // Set CLK / 1
	   	}
	    void pinsel_Function()
	   	{
        	// Select pin-select functionality
        	LPC_PINCON->PINSEL4 &= ~( (3 << 16) | (3 << 18) );
        	LPC_PINCON->PINSEL4 |=  ( (2 << 16) | (2 << 18));
	   	}
	    void prescalar_Settings()
	    {
	    	const uint32_t baud_Rate = 9600;
	    	uint32_t dll_val = (sys_get_cpu_clock()/(16*baud_Rate)) + 0.5;
	    	LPC_UART2->DLL = dll_val%(0xFF);
	    	LPC_UART2->DLM = dll_val/(0xFF);
	    	LPC_UART2->LCR = 3;         // 8-bit data
	    }

	    void enable_DLAB()
	    {
	    	LPC_UART2->LCR = (1 << 7);
	    }

        bool init(void)
        {
        	power_Enable();
        	pinsel_Pclock();
        	pinsel_Function();
        	enable_DLAB();
        	prescalar_Settings();

        	LPC_UART2->IER = 0x01;
        	NVIC_EnableIRQ(UART2_IRQn);
            return true;
        }


        bool run(void *p)
        {


           while(1)
           {

           }


            return true;
        }
    private:
        GPIO_CUSTOM gpio_pin;

};

// IDs used for getSharedObject() and addSharedObject(), to share the same queue between both the task
typedef enum {
   shared_OrientQueueId,
} sharedHandleId_t;

static LPC_TIM_TypeDef * one_ms_timer_ptr = NULL;
static SemaphoreHandle_t triggerTenMicroSec = NULL;
static SemaphoreHandle_t triggerHundredMilliSec	   = NULL;
void caliberate(void);
#define  one_ms_timer    (2)
#define  SET			 (1)
#define  HUNDRED_MILLI	 (100)
#define  TENMILLI	     (1)
void Update_timers_acceleration();


typedef enum {
	invalid,
	forw,
	back
} forBack_Count;
static uint32_t step_Count =0;
extern  uint32_t check;
// Timer to re- calibrate the accelerometer
void Timer2_init(void);



// Orientation Computation Task
 class orient_compute : public scheduler_task
 {
     public:
        int first =0;
 		int16_t x_value, y_value, z_value, x_th, y_th, z_th, x_prev, y_prev,z_prev, x_old, y_old, z_old;
 		int16_t X[50],Y[50],Z[50];
 		int16_t X_run[25],Y_run[25],Z_run[25];
 		int step =0;
 		uint32_t time 		        = 0;
        orient_compute(uint8_t priority);
        void calibrate(void);
        void sort_Function(void);
        void sort_Window(void);
        forBack_Count calculate_count(void);
        bool run(void *p);
 };
class tempMeasure : public scheduler_task
{
    public:
	tempMeasure (uint8_t priority) : scheduler_task("temp", 2048, priority)
    {
        /* Nothing to init */
    }
	bool run(void *p);

};
// Body Temperature Task
class bodyTemperature : public scheduler_task
{
    public:
	bodyTemperature (uint8_t priority) : scheduler_task("body", 2048, priority)
    {
        /* Nothing to init */
    }
	bool run(void * p);
};

// Heart Rate Task
class heartRate : public scheduler_task
{
    public:
	heartRate (uint8_t priority) : scheduler_task("hrt-rt", 5120, priority)
    {
        /* Nothing to init */
    }
  //  void static heartrate_irq(void);
	bool maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led);
	bool run(void * p);

};


#endif /* TASKS_HPP_ */
