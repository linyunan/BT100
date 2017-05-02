/***********************************************
 *创建人：kino
 *时  间：2016-5-11
 *功  能：外部实时时钟
 **********************************************/
#ifndef DS1302_H
#define DS1302_H

#include "OnBoard.h"
#include "hal_types.h"

#define DS1302_RST      P0_3
#define DS1302_SCLK     P0_1
#define DS1302_IO       P0_2
  
#define	ISLeapYear(yr)	(!((yr) % 400) || (((yr) % 100) && !((yr) % 4)))
#define	DSBEGYEAR	        16     // UTC started at 00:00:00 January 1, 2016
#define	DSDAY             86400UL  // 24 hours * 60 minutes * 60 seconds
#define	DsYearLength(yr)	(ISLeapYear(yr) ? 366 : 365)

typedef struct 
{
  uint8 cYear;
  uint8	cMon;
  uint8	cDay;
  uint8	cHour;
  uint8	cMin;
  uint8	cSec;
//  uint8	cWeek;
}SYSTIME;

extern SYSTIME systime;

void Ds1302_Init(void);
void DS1302_InitConf(void);
void DS1302_Write(uint8 Data);
uint8 DS1302_Read(void);
void WDS1302(uint8 ucAddr, uint8 ucDat)	;
uint8 RDS1302(uint8 ucAddr);
void DS_WriteProtect(void);
void DS_WriteUnProtect(void);
uint32 Ds1302_ConverUTCSecs( SYSTIME *tm );

void DsSet_Time();

void DsGet_Time();

void DS1302_InitData(void);
#endif