#ifndef _MY_TIMERS_H
#define _MY_TIMERS_H
#include "lcdTask.h"
#include "i2cTemp.h"
#include "adcTask.h"
void startTimerForLCD(vtLCDStruct *vtLCDdata);
void startTimerForTemperature(vtTempStruct *vtTempdata);
void startTimerForADC(adcStruct *adcData);
#endif