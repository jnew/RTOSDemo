#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "semphr.h"

/* include files. */
#include "vtUtilities.h"
#include "vtI2C.h"
#include "LCDtask.h"
#include "sensorTask.h"
#include "motorTask.h"
#include "I2CTaskMsgTypes.h"
#include "lpc17xx_gpio.h"

#define i2cSTACK_SIZE		(5*configMINIMAL_STACK_SIZE)

typedef struct {
	uint8_t msgType;
	uint8_t data[4];
} motorMsg;

static portTASK_FUNCTION_PROTO( vmotorTask, pvParameters );
//start the task
void vStartmotorTask(motorStruct *params ,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c, vtLCDStruct *lcd) {

	if ((params->inQ = xQueueCreate(20,sizeof(motorMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	if ((retval = xTaskCreate( vmotorTask , ( signed char * ) "motorTask", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendmotorMoveMsg(motorStruct *motorData,uint8_t length,uint8_t* value,portTickType ticksToBlock)
{
	motorMsg moveMsg;

	if (motorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	moveMsg.msgType = SENSORTASK_MSG;
	uint8_t i;
	for( i = 0; i < length; i = i+1) {
		moveMsg.data[i] = value[i];
	}
	return(xQueueSend(motorData->inQ,(void *) (&moveMsg),ticksToBlock));
}

portBASE_TYPE SendmotorERRORMsg(motorStruct *motorData, uint8_t errorType, portTickType ticksToBlock)
{
	motorMsg errorMsg;

	if (motorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	errorMsg.msgType = errorType;

	return(xQueueSend(motorData->inQ,(void *) (&errorMsg),ticksToBlock));
}

int getMsgType(motorMsg *Msg)
{
	return(Msg->msgType);
}

uint8_t* getData(motorMsg *Msg)
{
	return (uint8_t *) &Msg->data[0];
}

static portTASK_FUNCTION(vmotorTask, pvParameters) {
	motorStruct *param = (motorStruct *) pvParameters;
	motorMsg msg;
	
	const uint8_t motorCommand[]= {0xBB};
	
	SendLCDPrintMsg(param->lcdData,20,"motorTask Init",portMAX_DELAY);
	
	for( ;; ) {
		//wait forever or until queue has something
		if (xQueueReceive(param->inQ,(void *) &msg,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		switch(getMsgType(&msg)) {
			//only one type of message so far
			case SENSORTASK_MSG: {


				
				//this is where the motorTask will translate from human readable movement to block of sabertooth stuff



				//current slave address is 0x4F, take note
				if (vtI2CEnQ(param->dev,vtRoverMovementAck,0x4F,sizeof(motorCommand), motorCommand, 3) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				SendLCDPrintMsg(param->lcdData,20,"Sent motor move",portMAX_DELAY);
			break;
			}
			case ROVERACK_ERROR: {
				//this is where the arm will re-request the movement ack from the rover
				//right now, it does nothing
			break;
			}
		}

	}
}

