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
#include "sensorTask.h"
#include "I2CTaskMsgTypes.h"
#include "lpc17xx_gpio.h"
#include "messageDefs.h"

#define i2cSTACK_SIZE		(5*configMINIMAL_STACK_SIZE)

typedef struct {
	uint8_t msgType;
	uint8_t data[4];
} sensorMsg;

static portTASK_FUNCTION_PROTO( vsensorTask, pvParameters );
//start the task
void vStartsensorTask(sensorStruct *params ,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c, vtLCDStruct *lcd, motorStruct *motorData) {

	if ((params->inQ = xQueueCreate(20,sizeof(sensorMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	params->motorData = motorData;
	if ((retval = xTaskCreate( vsensorTask , ( signed char * ) "sensorTask", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendsensorGatherMsg(sensorStruct *sensorData)
{
	if (sensorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	sensorMsg gatherMsg;
	gatherMsg.msgType = GATHER_MSG;
	return(xQueueSend(sensorData->inQ,(void *) (&gatherMsg),portMAX_DELAY));
}

portBASE_TYPE SendmessageCheck(sensorStruct *sensorData)
{
	if (sensorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	sensorMsg gatherMsg;
	gatherMsg.msgType = sensorData->checkType;
	return(xQueueSend(sensorData->inQ,(void *) (&gatherMsg),portMAX_DELAY));
}

portBASE_TYPE SendsensorValueMsg(sensorStruct *sensorData, uint8_t msgtype, uint8_t length, uint8_t* value, portTickType ticksToBlock)
{
	sensorMsg valueMsg;

	if (sensorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	valueMsg.msgType = msgtype;
	uint8_t i;
	for( i = 0; i < length; i = i+1) {
		valueMsg.data[i] = value[i];
	}
	return(xQueueSend(sensorData->inQ,(void *) (&valueMsg),ticksToBlock));
}

portBASE_TYPE SendsensorERRORMsg(sensorStruct *sensorData, uint8_t errorType, portTickType ticksToBlock)
{
	sensorMsg errorMsg;

	if (sensorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	errorMsg.msgType = errorType;

	return(xQueueSend(sensorData->inQ,(void *) (&errorMsg),ticksToBlock));
}

int getMsgType(sensorMsg *Msg)
{
	return(Msg->msgType);
}

uint8_t* getData(sensorMsg *Msg)
{
	return (uint8_t *) &Msg->data[0];
}

void clearData(sensorMsg *Msg)
{
  	int i;
	for (i = 0; i < 4; i = i + 1)
		Msg->data[i] = 0;
}

static portTASK_FUNCTION(vsensorTask, pvParameters) {
	sensorStruct *param = (sensorStruct *) pvParameters;
	sensorMsg msg;

	const uint8_t gatherReq[]= {0xAA};
	const uint8_t gatherCheck[]= {0xAB};
	const uint8_t motorCheck[]= {0xBB};
	
	SendLCDPrintMsg(param->lcdData,20,"sensorTask Init",portMAX_DELAY);
	
	
	for( ;; ) {
		//wait forever or until queue has something
		if (xQueueReceive(param->inQ,(void *) &msg,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		switch(getMsgType(&msg)) {
			//0 is gather, send an I2C request out
			case GATHER_MSG: {
				//current slave address is 0x4F, take note
					if (vtI2CEnQ(param->dev,vtSensorGatherRequest,0x4F,sizeof(gatherReq),gatherReq,3) != pdTRUE)
						VT_HANDLE_FATAL_ERROR(0);
					SendLCDPrintMsg(param->lcdData,20,"SND: Gather Req",portMAX_DELAY);
			break;
			}
			//this is a check for sensor data called by a timer callback
			case GATHER_CHECK: {
					if (vtI2CEnQ(param->dev,vtSensorGatherCheck,0x4F,sizeof(gatherCheck),gatherCheck,5) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
					SendLCDPrintMsg(param->lcdData,20,"SND: Gather Chk",portMAX_DELAY);
			break;
			}
			//1 is incoming i2c data, this is where we handle sensor data and call algorithm subroutines... hopefully
			case SENSORVALUE_MSG: {
				uint8_t *dataPtr = getData(&msg);
				SendLCDPrintMsg(param->lcdData,20,"RCV: Sensor Data",portMAX_DELAY);
				
				
				
				//this is where the movement algorithm will decide what to issue as a command


				//SendsensorGatherMsg(param);
				SendmotorMoveMsg(param->motorData, SENSORTASK_MSG, dataPtr, portMAX_DELAY);
			break;
			}
			//bad/no data from the rover, we need to regather
			case GATHER_ERROR_MSG: {
			  	SendLCDPrintMsg(param->lcdData,20,"RSND: Gather Req",portMAX_DELAY);
				SendsensorGatherMsg(param);
			break;
			}
			case ROVERACK_CHECK: {
				if (vtI2CEnQ(param->dev,vtRoverMovementCheck,0x4F,sizeof(motorCheck),motorCheck,3) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				SendLCDPrintMsg(param->lcdData,20,"SND: Move Check",portMAX_DELAY);
			break;
			}
			case ROVERMOVE_MSG: {
				SendLCDPrintMsg(param->lcdData,20,"RCV: Move Data",portMAX_DELAY);

				
				//this is where the actual movement will be recorded in the map	


				SendsensorGatherMsg(param);
			break;
			}
		}

	}
}

