/*
 * tasks.cpp
 *
 *  Created on: May 9, 2017
 *      Author: Sobey
 */

#include "tasks.hpp"
#include "algorithm.hpp"
#include "eint.h"
#include "handlers.hpp"
#include "queue.h"
//#include  "display.hpp"
#include "lpc_timers.h"

QueueHandle_t temp_data = NULL;
SemaphoreHandle_t startheart = NULL;
QueueHandle_t step_data =  NULL;

I2C1& i2c1 = I2C1::getInstance();
volatile bool flag1 = false;
extern volatile bool start;
bool bodyTemperature :: run(void *p)
{
	while(1)
	{
		if(start == true)
		{
			i2c1.readReg(I2CAddr_TemperatureSensor, 0);
			delay_ms(500);
		}
	}

}


bool heartRate :: maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
/**
* \brief        Read a set of samples from the MAX30102 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30102 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
{
  uint32_t un_temp;
 // unsigned char uch_temp;
  uint8_t uch_temp;

  *pun_red_led=0;
  *pun_ir_led=0;
  uint8_t ach_i2c_data[6] = {0};
  //uint8_t*
  i2c1.readRegisters(0xAE ,0x00, &uch_temp, 1);
  i2c1.readRegisters(0xAE ,0x01, &uch_temp, 1);

  i2c1.readRegisters(0xAE ,0x07, ach_i2c_data, 6);

  un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *pun_red_led+=un_temp;

  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]

  return true;
}
#define MAX_BRIGHTNESS 255
GPIO_CUSTOM gpioObj;
bool heartRate :: run(void *p)
{


			// GPIO 2 as INPUT to read Interrupt value
			gpioObj.setInputDir(2, 0);
			uint32_t aun_ir_buffer[500]; //IR LED sensor data
			int32_t n_ir_buffer_length;    //data length
			uint32_t aun_red_buffer[500];    //Red LED sensor data
			int32_t n_sp02; //SPO2 value
			int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
			int32_t n_heart_rate;   //heart rate value
			int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
			uint8_t uch_dummy;
			uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
			int i;
			int32_t n_brightness;
			float f_temp;
			i2c1.readRegisters(0xAE ,0x00, &uch_dummy, 1);
			n_brightness=0;
			un_min = 0x3FFFF;
			un_max=0;
			// def
			uint32_t pun_red_led;
			uint32_t pun_ir_led;

			Board_I2C_Device_AddressesI2C1 deviceAdd;
			deviceAdd = I2CAddr_HeartRateSensor;
			uint8_t data[4] = {5, 1, 0, 0};
		//	int a;

			i2c1.writeReg(deviceAdd, 0x09, 0x03); // HR mode
			i2c1.writeReg(deviceAdd, 0x02, 0xC0); // intr 1 enable
			i2c1.writeReg(deviceAdd, 0x03, 0x00); // intr 2 enable
			i2c1.writeReg(deviceAdd, 0x04, 0x00); // fifo write ptr
			i2c1.writeReg(deviceAdd, 0x05, 0x00); // fifo ovf ptr
			i2c1.writeReg(deviceAdd, 0x06, 0x00); // fifo read ptr
			i2c1.writeReg(deviceAdd, 0x08, 0x0F); // fifo config
			i2c1.writeReg(deviceAdd, 0x0C, 0x24); // led 1
			i2c1.writeReg(deviceAdd, 0x0D, 0x24); // led 2
			i2c1.writeReg(deviceAdd, 0x10, 0x7F); // pilot led
			i2c1.writeReg(deviceAdd, 0x0A, 0x27); // so2 config



			n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps

			// enable port 2 interrupt
			eint3_enable_port2(0, eint_falling_edge, heartrate_irq_callback);

			//read the first 500 samples, and determine the signal range
			for(i=0;i<n_ir_buffer_length;i++)
			{
				//while(((LPC_GPIOINT->IO2IntStatF & 0x01))!=1);   //wait until the interrupt pin asserts
				while(!flag1);
				flag1 = false;

				maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO

				if(un_min>aun_red_buffer[i])
					un_min=aun_red_buffer[i];    //update signal min
				if(un_max<aun_red_buffer[i])
					un_max=aun_red_buffer[i];    //update signal max
			}


				//calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
			maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

				//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
				while(1)
				{

					i=0;
					un_min=0x3FFFF;
					un_max=0;

					//dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
					for(i=100;i<500;i++)
					{
						aun_red_buffer[i-100]=aun_red_buffer[i];
						aun_ir_buffer[i-100]=aun_ir_buffer[i];

						//update the signal min and max
						if(un_min>aun_red_buffer[i])
						un_min=aun_red_buffer[i];
						if(un_max<aun_red_buffer[i])
						un_max=aun_red_buffer[i];
					}

					//take 100 sets of samples before calculating the heart rate.
					for(i=400;i<500;i++)
					{
						un_prev_data=aun_red_buffer[i-1];
					  //  while(((LPC_GPIOINT->IO2IntStatF & 0x01))!=1);
						while(!flag1);
						flag1 = false;
						maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));

						if(aun_red_buffer[i]>un_prev_data)
						{
							f_temp=aun_red_buffer[i]-un_prev_data;
							f_temp/=(un_max-un_min);
							f_temp*=MAX_BRIGHTNESS;
							n_brightness-=(int)f_temp;
							if(n_brightness<0)
								n_brightness=0;
						}
						else
						{
							f_temp=un_prev_data-aun_red_buffer[i];
							f_temp/=(un_max-un_min);
							f_temp*=MAX_BRIGHTNESS;
							n_brightness+=(int)f_temp;
							if(n_brightness>MAX_BRIGHTNESS)
								n_brightness=MAX_BRIGHTNESS;
						}
					}
					maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

					// temp measurement
			    	if(ch_hr_valid == 1 && n_heart_rate <170 && n_heart_rate>50)
			    	{
			    			if(xQueueSend(heart_data,&n_heart_rate,100))
			    			{

			    			}
			    	}
			    	if( ch_spo2_valid == 1 && n_sp02 >70)
			    	{
			        	if(xQueueSend(oxygen_data,&n_sp02,100))
			        	{

			        	}
			    	}

				}

	    return true;
}

bool tempMeasure:: run(void *p)
{
		while(1)
		{
				float resistance;
				float temp = ((LS.getRawValue() * 3.3)/4096);
				temp = 3.3/temp;
				resistance = 10/(temp - 1);
				float temperature, T1,T2,R1,R2;
				for(int i =0; i<240; i++)
				{
					if((resistance > resistance_array[i] )  )
					{
						if(resistance == resistance_array[i - 1])
						{
							temperature = i - 41;
							break;
						}
						if(resistance < resistance_array[i - 1] )
						{
							R1 = resistance_array[i - 1];
							R2 = resistance_array[i];
							T1 = i - 41;
							T2 = i - 40;
							temperature = ((resistance - R1) * (R2 - R1))/(T2-T1);
							temperature = temperature + T1;
							break;
						}

					}

				}
				int32_t body_temp = temperature;
				xQueueSend(temp_data,&body_temp,portMAX_DELAY);
			}
}


orient_compute::orient_compute(uint8_t priority) :scheduler_task("compute", 4096, priority)
 {
		     	Timer2_init();
	            QueueHandle_t my_queue = xQueueCreate(1, sizeof(forBack_Count));

	            addSharedObject(shared_OrientQueueId, my_queue);
				 // Create a binary semaphore 1 sec delay
				 vSemaphoreCreateBinary( triggerOneSec );
				 // Create a binary semaphore 1 Minute delay
				 vSemaphoreCreateBinary( triggerTenMilliSec ); // Create the semaphore
				 xSemaphoreTake(triggerOneSec, 0);
				 xSemaphoreTake(triggerTenMilliSec, 0);
	             caliberate();
 }

void orient_compute::caliberate(void)
 {
	if(first == 0)
	{
		for(int i=0; i<50; i++)
		{

			X[i] = AS.getX();
			Y[i] = AS.getY();
			Z[i] = AS.getZ();

		}
	}
	else
	{

			for(int i=0; i<25; i++)
			{

				X[i] = AS.getX();
				Y[i] = AS.getY();
				Z[i] = AS.getZ();

			}

	}
	first++;
	sort_Function();
 }
void orient_compute::sort_Function(void)
{
				sort(X, X + 25);
	        	sort(Y, Y + 25);
	        	sort(Z, Z + 25);
	        	x_old = x_prev;
	        	y_old = y_prev;
	        	z_old = z_prev;
	        	x_prev = x_th;
	        	y_prev = y_th;
	        	z_prev = z_th;
	        	x_th = (X[0] + X[24])/2;
	        	y_th = (Y[0] + Y[24])/2;
	        	z_th = (Z[0] + Z[24])/2;
}

void orient_compute:: sort_Window(void)
{

	sort(X_run, X_run + 25);
	sort(Y_run, Y_run + 25);
	sort(Z_run, Z_run + 25);

}

forBack_Count orient_compute::calculate_count(void)
{

	forBack_Count count = invalid;
		if( xSemaphoreTake(triggerTenMilliSec,portMAX_DELAY))
		{
          for(int i=0; i<24; i++)
          {
			X[i] = AS.getX();
			Y[i] = AS.getY();
			Z[i] = AS.getZ();
          }

		}

		sort_Window();
		 if(((x_old + 15 < x_prev )  && (x_th + 15 < x_prev )) )
		{
			count = forw;
			step_Count++;
			step++;
		}
		else if(((x_old > x_prev + 10)  && (x_th > x_prev + 10)) )
		{
			count = back;
			step_Count++;
			step++;
		}
		return count;
}

bool  orient_compute::run(void *p)
 {


     	forBack_Count count = calculate_count();

			xQueueSend(getSharedObject(shared_OrientQueueId), &count, portMAX_DELAY);


			if( xSemaphoreTake(triggerOneSec,portMAX_DELAY))
			{
					 caliberate();
			}

     return true;
 }

orient_process::orient_process (uint8_t priority) : scheduler_task("process", 4096, priority)
{
	// do nothing
}

bool orient_process:: run(void *p)
{
           /* We first get the queue handle the other task added using addSharedObject() */

       	forBack_Count orientation = invalid;
        QueueHandle_t qid = getSharedObject(shared_OrientQueueId);


        while(1)
        {
				/* Sleep the task forever until an item is available in the queue */
				if (xQueueReceive(qid, &orientation, portMAX_DELAY))
				{
					if(orientation == forw)
						printf("%d Step:\n", step_Count);
					else if(orientation == back)
						printf("%d Step:\n", step_Count);
					else
						;

			        	if(xQueueSend(step_data,&step_Count,0))
			        	{

			        	}


				}
        }
           return true;
}

extern "C"
{

void TIMER2_IRQHandler()
{
	// clear the interrupt
	one_ms_timer_ptr->IR =0b1;
	// Trigger 1ms task
	// Update 1 sec and 1 min timer
	Update_timers_acceleration();
}

}
 void Timer2_init(void)
{

	 const lpc_timer_t one_ms_timer_source = (lpc_timer_t) one_ms_timer;
	 // set timer resolution for 1 micro-second
	 const uint32_t one_micro_second = 1;
	 // Initialize the timer structure pointer
	 one_ms_timer_ptr = lpc_timer_get_struct(one_ms_timer_source);
	 // enable the timer for a precision of one milli second to increment TC count
	 lpc_timer_enable(one_ms_timer_source, one_micro_second);
	 // Stop on Interrupt , No auto re-start
	 one_ms_timer_ptr->MCR = 0b11;
	 //  timeout is 1 msec
	 one_ms_timer_ptr->MR0 = 10000;
	 // enable LPC timer
	 lpc_timer_enable(one_ms_timer_source, one_micro_second);
	 // Get the IRQ number for timer interrupt
	 const IRQn_Type timer_irq = lpc_timer_get_irq_num(one_ms_timer_source);
	 // Enable timer interrupt
	 NVIC_EnableIRQ(timer_irq);

}

  void Update_timers_acceleration()
   {
  	 // Timer count to increment 1 sec timer
  	 static uint16_t timer_count = 0;
  	 // Timer count to increment 1 minute timer
  	 static uint8_t  cpu_timer_count = 0;

  	 timer_count++;

  	 if(timer_count == TENMILLI)
  	 {
  		 // trigger SD_card write routine
  		 xSemaphoreGive(triggerTenMilliSec);
  		 // trigger WatchDog write routine

  		 // Increment seconds
  		 cpu_timer_count ++;
  		 // Is it 1 Minute?
  		 if(cpu_timer_count == SEC)
  		 {
  			 xSemaphoreGive(triggerOneSec);
  			 cpu_timer_count = 0;
  		 }
  		 timer_count = 0;
  	 }


   }
