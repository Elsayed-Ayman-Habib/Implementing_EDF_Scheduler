/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

#define Not_Availbale 0
/* defination of the period of each task */
#define ButtonMonitor1_Period      50
#define ButtonMonitor2_Period      50
#define PeriodicTransmitter_Period 100
#define UartReceiver_Period        20
#define LoadSimulation1_Period     10
#define LoadSimulation2_Period     100

/* Create a Handler for each task */
TaskHandle_t ButtonMonitor1_TaskHandler      = NULL;
TaskHandle_t ButtonMonitor2_TaskHandler      = NULL;
TaskHandle_t PeriodicTransmitter_TaskHandler = NULL;
TaskHandle_t UartReceiver_TaskHandler        = NULL;
TaskHandle_t LoadSimulation1_TaskHandler     = NULL;
TaskHandle_t LoadSimulation2_TaskHandler     = NULL;

BaseType_t xReturned;
QueueHandle_t xQueue;
char* TaskExecute = (void*)0;

int button1_TimeIn ,  button1_TimeOut ,  button1_TotalTime;
int button2_TimeIn ,  button2_TimeOut ,  button2_TotalTime;
int periodic_TimeIn,  periodic_TimeOut,  periodic_TotalTime;
int uart_TimeIn    ,  uart_TimeOut    ,  uart_TotalTime;
int load1_TimeIn   ,  load1_TimeOut   ,  load1_TotalTime;
int load2_TimeIn   ,  load2_TimeOut   ,  load2_TotalTime;

int system_Time;
int cpu_Load;

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

/*
 *This task will monitor rising and falling edge on button 1 and send this event to the consumer ( Uart_Receiver ) task. 
 *(Note: The rising and failling edges are treated as separate events, hence they have separate strings).
 */
void Button_1_Monitor (void * pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
	
	pinState_t Button1_State = GPIO_read(PORT_0, PIN1);
	pinState_t Button1_Value;
	char* MassageButton1_Holder = (void*)0;


	TaskExecute = " Task_Button1_is_Execute ";
	
	vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );

	for(;;)
	{
	 Button1_Value =  GPIO_read(PORT_0, PIN1);
		
	  if ( Button1_Value == PIN_IS_HIGH )
		{
			if ( Button1_State == PIN_IS_LOW )
			{
				if (xQueue != Not_Availbale )
				{
					MassageButton1_Holder = "Task_Button1_is_Rising_Edge\n";
					xQueueOverwrite( xQueue, &MassageButton1_Holder );
				}
		  }
		}
		else if ( Button1_Value == PIN_IS_LOW ) 
		 {
				if ( Button1_State == PIN_IS_HIGH )
				{
					if (xQueue != Not_Availbale )
					{
						MassageButton1_Holder = "Task_Button1_is_Falling_Edge\n";
						xQueueOverwrite( xQueue, ( void * ) &MassageButton1_Holder );
					}
				}
			}
		Button1_State = Button1_Value; 
			
		GPIO_write (PORT_0, PIN3, PIN_IS_LOW);
		vTaskDelayUntil( &xLastWakeTime, ButtonMonitor1_Period );
		GPIO_write (PORT_0, PIN3, PIN_IS_HIGH);
					
		/* when idle task execute */
		GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
	}
}


/*
 *This task will monitor rising and falling edge on button 2 and send this event to the consumer ( Uart_Receiver) task. 
 *(Note: The rising and failling edges are treated as separate events, hence they have separate strings)
 */
void Button_2_Monitor (void * pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	pinState_t Button2_State = GPIO_read(PORT_0, PIN2);
	pinState_t Button2_Value;
	char* MassageButton2_Holder = (void*)0;
	
	
	TaskExecute = " Task_Button2_is_Execute ";
	
	vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
	
	for(;;)
	{
	 Button2_Value =  GPIO_read(PORT_0, PIN2);
		
	  if ( Button2_Value != Button2_State )
		{
			if ( Button2_Value == PIN_IS_HIGH )
			{
				if (xQueue != Not_Availbale )
				{
					MassageButton2_Holder = "Task_Button2_is_Rising_Edge\n";
					xQueueOverwrite( xQueue, ( void * ) &MassageButton2_Holder);
				}
			}
		}
		else if ( Button2_Value == PIN_IS_LOW )
		{
			if ( Button2_State == PIN_IS_HIGH )
				{
					if (xQueue != Not_Availbale )
					{
						MassageButton2_Holder = "Task_Button2_is_Falling_Edge\n";
						xQueueOverwrite( xQueue, ( void * ) &MassageButton2_Holder);
					}
			}
		}
			Button2_State = Button2_Value; 
			GPIO_write (PORT_0, PIN4, PIN_IS_LOW);
		  vTaskDelayUntil( &xLastWakeTime, ButtonMonitor2_Period );
			GPIO_write (PORT_0, PIN4, PIN_IS_HIGH);
		
				
	  	/* when idle task execute */
	  	GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
 }
}

/*
 *This task will send preiodic string every 100ms to the consumer ( Uart_Receiver) task
 */
void Periodic_Transmitter (void * pvParameters)
{
 TickType_t xLastWakeTime = xTaskGetTickCount();
	
 char* MassageTX_Holder = (void*)0;
	
	TaskExecute = " Task_TX_is_Execute ";

	vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );
	
	for(;;)
	  {
		if (xQueue != Not_Availbale )
				{
					MassageTX_Holder = "Task_Periodic_Transmitter   \n";
					xQueueSend( xQueue, ( void * ) &MassageTX_Holder, ( TickType_t ) PeriodicTransmitter_Period );
				}
					GPIO_write (PORT_0, PIN5, PIN_IS_LOW);
					vTaskDelayUntil( &xLastWakeTime, PeriodicTransmitter_Period );
					GPIO_write (PORT_0, PIN5, PIN_IS_HIGH);
						
		      /* when idle task execute */
		      GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
		}
}

/*
 *This is the consumer task which will write on UART any received string from other tasks
 */
void Uart_Receiver (void * pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	char* MassageRX_Holder = (void*)0;	

	
	TaskExecute = " Task_RX_is_Execute ";

	vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	
	for(;;)
	{
	if( xQueueReceive( xQueue,&(MassageRX_Holder ),( TickType_t ) 0 ) == pdPASS )
		{
			vSerialPutString((const signed char *)(MassageRX_Holder), 30);
			xQueueReset(xQueue);
		}
				GPIO_write (PORT_0, PIN6, PIN_IS_LOW);
				vTaskDelayUntil( &xLastWakeTime, UartReceiver_Period );
				GPIO_write (PORT_0, PIN6, PIN_IS_HIGH);
		 		
		    /* when idle task execute */
	    	GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
	}
}

void Load_1_Simulation (void * pvParameters)
{ 
 TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t load_1;
	
	TaskExecute = " Task_Load1_is_Execute ";

  vTaskSetApplicationTaskTag( NULL, ( void * ) 7 );
	
	for (;;)
	{
		for(load_1 =0 ; load_1 < 37000 ; load_1++){}
		GPIO_write (PORT_0, PIN7, PIN_IS_LOW);
		vTaskDelayUntil( &xLastWakeTime, LoadSimulation1_Period );
		GPIO_write (PORT_0, PIN7, PIN_IS_HIGH);
		
		/* when idle task execute */
		GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
	}
}

void Load_2_Simulation (void * pvParameters)
{
 TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t load_2;
	
	TaskExecute = " Task_Load2_is_Execute ";

	vTaskSetApplicationTaskTag( NULL, ( void * ) 8 );
	
	for (;;)
	{
		for( load_2 =0 ; load_2 < 90500 ; load_2++){}
		GPIO_write (PORT_0, PIN8, PIN_IS_LOW);
		vTaskDelayUntil( &xLastWakeTime, LoadSimulation2_Period );
		GPIO_write (PORT_0, PIN8, PIN_IS_HIGH);
		
		/* when idle task execute */
		GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
	}
}

void vApplicationTickHook( void )
{
	GPIO_write (PORT_0, PIN0, PIN_IS_HIGH);
	GPIO_write (PORT_0, PIN0, PIN_IS_LOW);
}

void vApplicationIdleHook( void )
{
	GPIO_write (PORT_0, PIN9, PIN_IS_HIGH);
}


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
  xQueue = xQueueCreate( 1 , sizeof( char* ) );
	
    /* Create Tasks here */
	
	/*Task_1 --> Button_1_Monitor */
	 
	xReturned = xTaskPeriodicCreate( Button_1_Monitor,
											 "ButtonMonitor_1", 
											 configMINIMAL_STACK_SIZE, 
											 (void * ) 0, 
											 ( 1 ), 
											 &ButtonMonitor1_TaskHandler,
											 ButtonMonitor1_Period );
											 
	
	
	/*Task_2 --> Button__Monitor */
	 
	xReturned = xTaskPeriodicCreate( Button_2_Monitor,
											 "ButtonMonitor_2", 
											 configMINIMAL_STACK_SIZE, 
											 (void * ) 0, 
											 ( 1 ), 
											 &ButtonMonitor2_TaskHandler,
											 ButtonMonitor2_Period );
											 
	

	/*Task_3 --> Periodic_Transmitter */
	 
	xReturned = xTaskPeriodicCreate( Periodic_Transmitter,
											 "PeriodicTransmitter", 
											 configMINIMAL_STACK_SIZE, 
											 (void * ) 0, 
											 ( 1 ), 
											 &PeriodicTransmitter_TaskHandler,
											 PeriodicTransmitter_Period );
											 
	
	
	/*Task_4 --> Uart_Receiver */
	 
	xReturned = xTaskPeriodicCreate( Uart_Receiver,
											 "UartReceiver", 
											 configMINIMAL_STACK_SIZE, 
											 (void * ) 0, 
											 ( 1 ), 
											 &UartReceiver_TaskHandler,
											 UartReceiver_Period );
											 
	/*Task_5 --> Load_1_Simulation */
	 
  xReturned = xTaskPeriodicCreate( Load_1_Simulation,
											 "LoadSimulation_1", 
											 configMINIMAL_STACK_SIZE, 
											 (void * ) 0, 
											 ( 1 ), 
											 &LoadSimulation1_TaskHandler,
											 LoadSimulation1_Period );

	/*Task_6 --> Load_2_Simulation */
	 
	xReturned = xTaskPeriodicCreate( Load_2_Simulation,
											 "LoadSimulation_2", 
											 configMINIMAL_STACK_SIZE, 
											 (void * ) 0, 
											 ( 1 ), 
											 &LoadSimulation2_TaskHandler,
											 LoadSimulation2_Period );
											 
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


