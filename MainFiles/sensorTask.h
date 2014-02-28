#ifndef SENSORTASK_H
#define SENSORTASK_H
#include "vtI2C.h"
#include "lcdTask.h"
#include "motorTask.h"

typedef struct {
	vtI2CStruct *dev;
	vtLCDStruct *lcdData;
	motorStruct *motorData;
	xQueueHandle inQ;
} sensorStruct;

//message types
#define GATHER_MSG 0
#define SENSORVALUE_MSG 1
#define ROVERMOVE_MSG 2
#define GATHER_ERROR_MSG 3


// Maximum length of a message that can be received by this task
#define msgMaxLen   (sizeof(portTickType))

void vStartsensorTask(sensorStruct *sensorData ,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c, vtLCDStruct *lcd, motorStruct *motorData);
//send message from timer
portBASE_TYPE SendsensorGatherMsg(sensorStruct *sensorData);
//send message from conductor with i2c data from pic
portBASE_TYPE SendsensorValueMsg(sensorStruct *sensorData,uint8_t msgtype, uint8_t length,uint8_t* value,portTickType ticksToBlock);
//send message that there was an error on the i2c bus
portBASE_TYPE SendsensorERRORMsg(sensorStruct *sensorData, uint8_t errorType, portTickType ticksToBlock);
#endif