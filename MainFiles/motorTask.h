#ifndef MOTORTASK_H
#define MOTORTASK_H
#include "vtI2C.h"
#include "lcdTask.h"

typedef struct {
	vtI2CStruct *dev;
	vtLCDStruct *lcdData;
	xQueueHandle inQ;
} motorStruct;

//message types
#define SENSORTASK_MSG 0
#define ROVERACK_ERROR 1

// Maximum length of a message that can be received by this task
#define msgMaxLen   (sizeof(portTickType))

void vStartmotorTask(motorStruct *motorData ,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c, vtLCDStruct *lcd);
//send message from sensorTask to motorTask
portBASE_TYPE SendmotorMoveMsg(motorStruct *motorData,uint8_t length,uint8_t* value,portTickType ticksToBlock);
portBASE_TYPE SendmotorERRORMsg(motorStruct *motorData, uint8_t errorType, portTickType ticksToBlock);
#endif