/*
    FreeRTOS V6.1.1 - Copyright (C) 2011 Real Time Engineers Ltd.

    ***************************************************************************
    *                                                                         *
    * If you are:                                                             *
    *                                                                         *
    *    + New to FreeRTOS,                                                   *
    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
    *    + Looking for basic training,                                        *
    *    + Wanting to improve your FreeRTOS skills and productivity           *
    *                                                                         *
    * then take a look at the FreeRTOS books - available as PDF or paperback  *
    *                                                                         *
    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
    *                  http://www.FreeRTOS.org/Documentation                  *
    *                                                                         *
    * A pdf reference manual is also available.  Both are usually delivered   *
    * to your inbox within 20 minutes to two hours when purchased between 8am *
    * and 8pm GMT (although please allow up to 24 hours in case of            *
    * exceptional circumstances).  Thank you for your support!                *
    *                                                                         *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public 
    License and the FreeRTOS license exception along with FreeRTOS; if not it 
    can be viewed here: http://www.freertos.org/a00114.html and also obtained 
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks
 * (which just exist to test the kernel port and provide an example of how to use
 * each FreeRTOS API function).
 *
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Check" hook -  This only executes fully every five seconds from the tick
 * hook.  Its main function is to check that all the standard demo tasks are
 * still operational.  The status can be viewed using on the Task Stats page
 * served by the WEB server.
 *
 * "uIP" task -  This is the task that handles the uIP stack.  All TCP/IP
 * processing is performed in this task.
 * 
 * "USB" task - Enumerates the USB device as a CDC class, then echoes back all
 * received characters with a configurable offset (for example, if the offset
 * is 1 and 'A' is received then 'B' will be sent back).  A dumb terminal such
 * as Hyperterminal can be used to talk to the USB task.
 */

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "system_LPC17xx.h"
#include "lpc17xx_gpio.h"

#ifndef   PCONP_PCTIM0
/* MTJ_NOTE: This will not compile properly if you do not delete the old version of */
/*       system_LPC17xx.h from the Keil compiler installation */
You should read the note above.
#endif


//Turn on web server
#define USE_WEB_SERVER 1

/* Define whether or not to start the standard FreeRTOS demo tasks (the code is still included in the project
   unless the files are actually removed from the project */
#define USE_FREERTOS_DEMO 0
// Define whether or not to use my LCD task
#define USE_MTJ_LCD 1
// Define whether to use my USB task
#define USE_MTJ_USE_USB 0
// Define whether to use MS1 Code
#define USE_MS1_CODE 0
// Define whether to use MS2 Code
#define USE_MS2_CODE 1

#if USE_FREERTOS_DEMO == 1
/* Demo app includes. */
#include "BlockQ.h"
#include "integer.h"
#include "blocktim.h"
#include "flash.h"
#include "semtest.h"
#include "PollQ.h"
#include "GenQTest.h"
#include "QPeek.h"
#include "recmutex.h"
#include "timers.h"
#endif

#include "partest.h"

// Include file for MTJ's LCD & i2cTemp tasks
#include "vtUtilities.h"
#include "lcdTask.h"
#include "vtI2C.h"
#include "myTimers.h"
#include "conductor.h"

//John's work
#include "adcTask.h"
#include "sensorTask.h"
#include "motorTask.h"

/* syscalls initialization -- *must* occur first */
#include "syscalls.h"
#include "extUSB.h"
#include <stdio.h>
/*-----------------------------------------------------------*/

/* The time between cycles of the 'check' functionality (defined within the
tick hook). */

#define mainCHECK_DELAY						( ( portTickType ) 5000 / portTICK_RATE_MS )

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY)
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY)
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY)
#define mainUIP_TASK_PRIORITY				( tskIDLE_PRIORITY)
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY)
#define mainGEN_QUEUE_TASK_PRIORITY			( tskIDLE_PRIORITY)
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY)
#define mainLCD_TASK_PRIORITY				( tskIDLE_PRIORITY)
#define mainADC_TASK_PRIORITY				( tskIDLE_PRIORITY)
#define mainUSB_TASK_PRIORITY				( tskIDLE_PRIORITY)
#define mainI2CMONITOR_TASK_PRIORITY		( tskIDLE_PRIORITY+2)
#define mainCONDUCTOR_TASK_PRIORITY			( tskIDLE_PRIORITY+1)
#define mainSENSOR_TASK_PRIORITY			( tskIDLE_PRIORITY+1)
#define mainMOTOR_TASK_PRIORITY				( tskIDLE_PRIORITY)

/* The WEB server has a larger stack as it utilises stack hungry string
handling library calls. */
#define mainBASIC_WEB_STACK_SIZE            ( configMINIMAL_STACK_SIZE * 4 )

/* The message displayed by the WEB server when all tasks are executing
without an error being reported. */
#define mainPASS_STATUS_MESSAGE				"All tasks are executing without error."

/*-----------------------------------------------------------*/

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/*
 * The task that handles the uIP stack.  All TCP/IP processing is performed in
 * this task.
 */
extern void vuIP_Task( void *pvParameters );

/*
 * The task that handles the USB stack.
 */
extern void vUSBTask( void *pvParameters );

/*
 * Simply returns the current status message for display on served WEB pages.
 */
char *pcGetTaskStatusMessage( void );

/*-----------------------------------------------------------*/

/* Holds the status message displayed by the WEB server. */
static char *pcStatusMessage = mainPASS_STATUS_MESSAGE;

// data structure required for conductor task
static vtConductorStruct conductorData;

#if USE_MTJ_LCD == 1
// data structure required for LCDtask API
static vtLCDStruct vtLCDdata; 
#endif

//John's data
static adcStruct adcData;
static sensorStruct sensorData;
static motorStruct motorData;

// data structure required for one I2C task
static vtI2CStruct vtI2C0;

/*-----------------------------------------------------------*/

int main( void )
{
	/* MTJ: initialize syscalls -- *must* be first */
	// syscalls.c contains the files upon which the standard (and portable) C libraries rely 
	init_syscalls();

	// Set up the LED ports and turn them off
	vtInitLED();

	/* Configure the hardware for use by this demo. */
	prvSetupHardware();

	//init GPIO debug
	GPIO_SetDir(0,0x78000,1);
	GPIO_ClearValue(0,0x78000);

	#if USE_FREERTOS_DEMO == 1
	/* Start the standard demo tasks.  These are just here to exercise the
	kernel port and provide examples of how the FreeRTOS API can be used. */
	vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );
    vCreateBlockTimeTasks();
    vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
    vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
    vStartIntegerMathTasks( mainINTEGER_TASK_PRIORITY );
    vStartGenericQueueTasks( mainGEN_QUEUE_TASK_PRIORITY );
    vStartQueuePeekTasks();
    vStartRecursiveMutexTasks();
	vStartLEDFlashTasks( mainFLASH_TASK_PRIORITY );
	#endif

	#if USE_WEB_SERVER == 1
	// Not a standard demo -- but also not one of mine (MTJ)
	/* Create the uIP task.  The WEB server runs in this task. */
    xTaskCreate( vuIP_Task, ( signed char * ) "uIP", mainBASIC_WEB_STACK_SIZE, ( void * ) NULL, mainUIP_TASK_PRIORITY, NULL );
	#endif

	#if USE_MTJ_LCD == 1
	// MTJ: My LCD demonstration task
	StartLCDTask(&vtLCDdata,mainLCD_TASK_PRIORITY);
	SendLCDPrintMsg(&vtLCDdata,20,"LCDTask Init",portMAX_DELAY);
	#endif
	
	// Start up an I2C task and associate it with the I2C0 hardware on the ARM (there are 3 I2C devices, we need this one)
	// See vtI2C.h & vtI2C.c for more details on this task and the API to access the task
	// Initialize I2C0 for I2C0 at an I2C clock speed of 100KHz
	if (vtI2CInit(&vtI2C0,0,mainI2CMONITOR_TASK_PRIORITY,100000) != vtI2CInitSuccess) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	// start up a "conductor" task that will move messages around
	vStartConductorTask(&conductorData,mainCONDUCTOR_TASK_PRIORITY,&vtI2C0,&adcData, &sensorData, &motorData);

    /* Create the USB task. MTJ: This routine has been modified from the original example (which is not a FreeRTOS standard demo) */
	#if USE_MTJ_USE_USB == 1
	initUSB();  // MTJ: This is my routine used to make sure we can do printf() with USB
    xTaskCreate( vUSBTask, ( signed char * ) "USB", configMINIMAL_STACK_SIZE, ( void * ) NULL, mainUSB_TASK_PRIORITY, NULL );
	#endif

	//Start John's Tasks
	#if USE_MS1_CODE == 1
	vStartadcTask( &adcData, mainADC_TASK_PRIORITY, &vtI2C0, &vtLCDdata);
	startTimerForADC(&adcData);
	#endif
	
	#if USE_MS2_CODE == 1
	vStartsensorTask(&sensorData, mainSENSOR_TASK_PRIORITY, &vtI2C0, &vtLCDdata, &motorData);
	vStartmotorTask(&motorData, mainMOTOR_TASK_PRIORITY, &vtI2C0, &vtLCDdata);
	//start the ARM's control loop (this will be on the webserver eventually)
	//init the check timer and store its handler
	sensorData.checkTimerHandle = initCheckTimer(&sensorData);
	SendsensorGatherMsg(&sensorData);
	#endif
	
	/* Start the scheduler. */
	// IMPORTANT: Once you start the scheduler, any variables on the stack from main (local variables in main) can be (will be...) written over
	//            because the stack is used by the interrupt handler
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
static unsigned long ulTicksSinceLastDisplay = 0;

	/* Called from every tick interrupt as described in the comments at the top
	of this file.

	Have enough ticks passed to make it	time to perform our health status
	check again? */
	ulTicksSinceLastDisplay++;
	if( ulTicksSinceLastDisplay >= mainCHECK_DELAY )
	{
		/* Reset the counter so these checks run again in mainCHECK_DELAY
		ticks time. */
		ulTicksSinceLastDisplay = 0;

#if USE_FREERTOS_DEMO == 1
		/* Has an error been found in any task? */
		if( xAreGenericQueueTasksStillRunning() != pdTRUE )
		{
			pcStatusMessage = "An error has been detected in the Generic Queue test/demo.";
		}
		else if( xAreQueuePeekTasksStillRunning() != pdTRUE )
		{
			pcStatusMessage = "An error has been detected in the Peek Queue test/demo.";
		}
		else if( xAreBlockingQueuesStillRunning() != pdTRUE )
		{
			pcStatusMessage = "An error has been detected in the Block Queue test/demo.";
		}
		else if( xAreBlockTimeTestTasksStillRunning() != pdTRUE )
		{
			pcStatusMessage = "An error has been detected in the Block Time test/demo.";
		}
	    else if( xAreSemaphoreTasksStillRunning() != pdTRUE )
	    {
	        pcStatusMessage = "An error has been detected in the Semaphore test/demo.";
	    }
	    else if( xArePollingQueuesStillRunning() != pdTRUE )
	    {
	        pcStatusMessage = "An error has been detected in the Poll Queue test/demo.";
	    }
	    else if( xAreIntegerMathsTaskStillRunning() != pdTRUE )
	    {
	        pcStatusMessage = "An error has been detected in the Int Math test/demo.";
	    }
	    else if( xAreRecursiveMutexTasksStillRunning() != pdTRUE )
	    {
	    	pcStatusMessage = "An error has been detected in the Mutex test/demo.";
	    }
#endif
	}
}
/*-----------------------------------------------------------*/

char *pcGetTaskStatusMessage( void )
{
	/* Not bothered about a critical section here. */
	return pcStatusMessage;
}
/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
	/* Disable peripherals power. */
	SC->PCONP = 0;

	/* Enable GPIO power. */
	SC->PCONP = PCONP_PCGPIO;

	/* Disable TPIU. */
	PINCON->PINSEL10 = 0;

	/*  Setup the peripheral bus to be the same as the PLL output (64 MHz). */
	SC->PCLKSEL0 = 0x05555555;

	/* Configure the LEDs. */
	vParTestInitialise();
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will get called if a task overflows its stack. */

	( void ) pxTask;
	( void ) pcTaskName;

	// MTJ: I have directed this to the fatal error handler
	VT_HANDLE_FATAL_ERROR(0);
	for( ;; );
}
/*-----------------------------------------------------------*/

void vConfigureTimerForRunTimeStats( void )
{
const unsigned long TCR_COUNT_RESET = 2, CTCR_CTM_TIMER = 0x00, TCR_COUNT_ENABLE = 0x01;

	/* This function configures a timer that is used as the time base when
	collecting run time statistical information - basically the percentage
	of CPU time that each task is utilising.  It is called automatically when
	the scheduler is started (assuming configGENERATE_RUN_TIME_STATS is set
	to 1). */

	/* Power up and feed the timer. */
	SC->PCONP |= 0x02UL;
	SC->PCLKSEL0 = (SC->PCLKSEL0 & (~(0x3<<2))) | (0x01 << 2);

	/* Reset Timer 0 */
	TIM0->TCR = TCR_COUNT_RESET;

	/* Just count up. */
	TIM0->CTCR = CTCR_CTM_TIMER;

	/* Prescale to a frequency that is good enough to get a decent resolution,
	but not too fast so as to overflow all the time. */
	TIM0->PR =  ( configCPU_CLOCK_HZ / 10000UL ) - 1UL;

	/* Start the counter. */
	TIM0->TCR = TCR_COUNT_ENABLE;
}
/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
	// Here we decide to go to sleep because we *know* that no other higher priority task is ready *and* we
	//   know that we are the lowest priority task (we are the idle task)
	// Important: We are just being *called* from the idle task, so we cannot run a loop or anything like that
	//   here.  We just go to sleep and then return (which presumably only happens when we wake up).
	vtITMu8(vtITMPortIdle,SCB->SCR);
	__WFI(); // go to sleep until an interrupt occurs
	// DO NOT DO THIS... It is not compatible with the debugger: __WFE(); // go into low power until some (not quite sure what...) event occurs
	vtITMu8(vtITMPortIdle,SCB->SCR+0x10);
}
