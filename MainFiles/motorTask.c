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
#include "messageDefs.h"

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
	
	const uint8_t *motorCommand;
	const uint8_t forward_ten[]= {0xBA, 0x9F, 0x1F, 0x64, 0x00};
	const uint8_t forward_five[]= {0xBA, 0x9F, 0x1F, 0x32, 0x00};
	const uint8_t turn_right[]= {0xBA, 0x9F, 0x62, 0x0B, 0x00};
	const uint8_t turn_left[]= {0xBA, 0xE1, 0x1F, 0x0B, 0x00};
	const uint8_t backwards_five[]= {0xBA, 0xE1, 0x62, 0x32, 0x00};
	const uint8_t stop[]= {0xBA, 0x00, 0x00, 0x00, 0x00};
	unsigned int demoInt = 0;
	
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
				if(demoInt == 0)
					motorCommand = forward_ten;
				else if(demoInt == 1)
					motorCommand = turn_left;
				else if (demoInt == 2)
					motorCommand = forward_five;
				else
					motorCommand = stop;
				if (vtI2CEnQ(param->dev,vtRoverMovementCommand,0x4F, 5, motorCommand, 3) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				demoInt = demoInt + 1;
				SendLCDPrintMsg(param->lcdData,20,"SND: Move Command",portMAX_DELAY);
			break;
			}
			case ROVERACK_ERROR: {
				//this is where the arm will re-request the movement ack from the rover
				if(demoInt == 0)
					motorCommand = forward_ten;
				else if(demoInt == 1)
					motorCommand = turn_left;
				else if (demoInt == 2)
					motorCommand = forward_five;
				else
					motorCommand = stop;
				if (vtI2CEnQ(param->dev,vtRoverMovementCommand,0x4F, 5, motorCommand, 3) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				demoInt = demoInt + 1;
				SendLCDPrintMsg(param->lcdData,20,"RSND: Move Command",portMAX_DELAY);
			break;
			}
		}

	}
}

