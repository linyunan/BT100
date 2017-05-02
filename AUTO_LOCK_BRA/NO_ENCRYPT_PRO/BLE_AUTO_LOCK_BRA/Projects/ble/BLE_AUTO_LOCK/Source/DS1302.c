#include "ds1302.h"


#define SECOND 0x80		//秒  00 - 59
#define MINUTE 0x82		//分  00 - 59 
#define HOUR   0x84		//小时01 - 12/ 00 - 23
#define DAY    0x86		//天  01 - 28,29,30,31
#define MONTH  0x88		//月  01 - 12
#define WEEK   0x8A		//周  01 - 07
#define YEAR   0x8C		//年  00 - 99

SYSTIME systime;	

		
void Ds1302_Init(void)
{
  P1SEL &= ~(BV(6));    // Configure Port 0.1 as GPIO
  P1SEL &= ~(BV(7));    // Configure Port 0.2 as GPIO
  P2SEL &= ~(BV(0));    // Configure Port 0.3 as GPIO
  P1DIR |= BV(6);       // port 0 pins (P0.1) as output
  P1DIR |= BV(7);       // port 0 pins (P0.2) as output
  P2DIR |= BV(0);       // port 0 pins (P0.3) as output
  P1 &= ~(BV(6));       // pin0.1 on port 0 to low
  P1 &= ~(BV(7));       // pin0.2 on port 0 to low
  P2 &= ~(BV(0));       // pin0.3 on port 0 to low
  

//  WDS1302(0x90,0x00);

}

void DS1302_InitConf(void)
{
  systime.cYear = 0x16;
  systime.cMon  = 0x08;
//  systime.cWeek = 0x03;
  systime.cDay  = 0x10;
  systime.cHour = 0x11;
  systime.cMin  = 0x25;
  systime.cSec  = 0x00;
//  WDS1302(0x90,0x03);	   //禁止涓流充电
  DsSet_Time();
}


void DS1302_InitData(void)
{
  systime.cYear = 0;
  systime.cMon  = 0;
//  systime.cWeek = 0x03;
  systime.cDay  = 0;
  systime.cHour = 0;
  systime.cMin  = 0;
  systime.cSec  = 0;

}

void Ds1302_IoIN(void)
{
  P1DIR &= ~(BV(6));
}
void Ds1302_IoOUT(void)
{
  P1DIR |= BV(6);
}

void DS1302_Write(uint8 Data)
{
  uint8 i;	
  Ds1302_IoOUT();
  for(i=0;i<8;i++)
  {	
    DS1302_IO = Data&0x01;
    DS1302_SCLK = 1;
    asm("nop");
    DS1302_SCLK = 0;
    asm("nop");
    Data = Data>>1;				
  }	
}

uint8 DS1302_Read(void)
{
  uint8 TempDat=0,i;
  Ds1302_IoIN();
  for(i=0;i<8;i++)
  {	
    TempDat>>=1;
    if(DS1302_IO) 
    {
      TempDat=TempDat|0x80;
    }
    DS1302_SCLK=1;
    asm("nop");
    DS1302_SCLK=0;
    asm("nop");
    
  }	
  return TempDat;
}


void WDS1302(uint8 ucAddr, uint8 ucDat)	
{
  DS1302_RST = 0;
  DS1302_SCLK = 0;
  DS1302_RST = 1;
  DS1302_Write(ucAddr);       	// 地址命令
  DS1302_Write(ucDat);       	// 写一字节
  DS1302_SCLK = 1;
  DS1302_RST = 0;
} 

uint8 RDS1302(uint8 ucAddr)
{
  uint8 ucDat;
  DS1302_RST = 0;
  DS1302_SCLK = 0;
  DS1302_RST = 1;
  DS1302_Write(ucAddr);       	//写地址命令
  ucDat=DS1302_Read();       
  DS1302_SCLK = 1;
  DS1302_RST = 0;
  return ucDat;
}
// 开启数据写保护
void DS_WriteProtect(void)
{
  WDS1302(0x8E, 0x80);
}
// 关闭数据写保护
void DS_WriteUnProtect(void)
{
  WDS1302(0x8E, 0x00);
}

//更新DS1302时间
void DsSet_Time()
{
  DS_WriteUnProtect();
#if 0  
  WDS1302(YEAR,sys.cYear); 
  WDS1302(MONTH,sys.cMon&0x1f);
  WDS1302(DAY,sys.cDay&0x3f);
  WDS1302(HOUR,sys.cHour&0x3f);
  WDS1302(MINUTE,sys.cMin&0x7f);
  WDS1302(SECOND,sys.cSec&0x7f);	
//  WDS1302(WEEK,sys.cWeek&0x07);
#else
//  WDS1302(0x90,0x03);
  WDS1302(YEAR,systime.cYear); 
  WDS1302(MONTH,systime.cMon&0x1f);
  WDS1302(DAY,systime.cDay&0x3f);
  WDS1302(HOUR,systime.cHour&0x3f);
  WDS1302(MINUTE,systime.cMin&0x7f);
  WDS1302(SECOND,systime.cSec&0x7f);	
//  WDS1302(WEEK,systime.cWeek&0x07);
#endif
  DS_WriteProtect();
}


// 获取DS1302时间
void DsGet_Time()
{
#if 0
  uint8 uiTempDat;
  uiTempDat=RDS1302(YEAR|0x01);			  
  sys->cYear=(uiTempDat>>4)*16+(uiTempDat&0x0f);

  uiTempDat=RDS1302(MONTH|0x01);
  sys->cMon=((uiTempDat&0x1f)>>4)*16+(uiTempDat&0x0f);	

  uiTempDat=RDS1302(DAY|0x01);
  sys->cDay=((uiTempDat&0x3f)>>4)*16+(uiTempDat&0x0f);

  uiTempDat=RDS1302(HOUR|0x01);
  sys->cHour=((uiTempDat&0x3f)>>4)*16+(uiTempDat&0x0f);

  uiTempDat=RDS1302(MINUTE|0x01);
  sys->cMin=((uiTempDat&0x7f)>>4)*16+(uiTempDat&0x0f);

  uiTempDat=RDS1302(SECOND|0x01);
  sys->cSec=((uiTempDat&0x7f)>>4)*16+(uiTempDat&0x0f);

//  uiTempDat=RDS1302(WEEK|0x01);
//  sys->cWeek=uiTempDat&0x07;
#else
  
  uint8 uiTempDat;
  uiTempDat=RDS1302(YEAR|0x01);			  
  systime.cYear=(uiTempDat>>4)*16+(uiTempDat&0x0f);

  uiTempDat=RDS1302(MONTH|0x01);
  systime.cMon=((uiTempDat&0x1f)>>4)*16+(uiTempDat&0x0f);	

  uiTempDat=RDS1302(DAY|0x01);
  systime.cDay=((uiTempDat&0x3f)>>4)*16+(uiTempDat&0x0f);

  uiTempDat=RDS1302(HOUR|0x01);
  systime.cHour=((uiTempDat&0x3f)>>4)*16+(uiTempDat&0x0f);

  uiTempDat=RDS1302(MINUTE|0x01);
  systime.cMin=((uiTempDat&0x7f)>>4)*16+(uiTempDat&0x0f);

  uiTempDat=RDS1302(SECOND|0x01);
  systime.cSec=((uiTempDat&0x7f)>>4)*16+(uiTempDat&0x0f);

//  uiTempDat=RDS1302(WEEK|0x01);
//  systime.cWeek=uiTempDat&0x07;
#endif
}

uint8 MonthLength( uint8 lpyr, uint8 mon )
{
  uint8 days = 31;

  if ( mon == 1 ) // feb
  {
    days = ( 28 + lpyr );
  }
  else
  {
    if ( mon > 6 ) // aug-dec
    {
      mon--;
    }

    if ( mon & 1 )
    {
      days = 30;
    }
  }

  return ( days );
}

/*********************************************************************
 * @fn      osal_ConvertUTCSecs
 *
 * @brief   Converts a UTCTimeStruct to UTCTime
 *
 * @param   tm - pointer to provided struct
 *
 * @return  number of seconds since 00:00:00 on 01/01/2000 (UTC)
 */
uint32 Ds1302_ConverUTCSecs( SYSTIME *tm )
{
  uint32 seconds;
  
  /* Seconds for the partial day */
  seconds = (((tm->cHour * 60UL) + tm->cMin) * 60UL) + tm->cSec;
  
  /* Account for previous complete days */
  {
    /* Start with complete days in current month */
    uint16 days = tm->cDay;
    
    /* Next, complete months in current year */
    {
      int8 month = tm->cMon;
      month --;  //当月的不能加
      while ( --month >= 0 )
      {
        days += MonthLength( ISLeapYear( tm->cYear ), month );
      }
    }

    /* Next, complete years before current year */
    {
      uint16 year = tm->cYear;
      year --;
      while ( --year >= DSBEGYEAR )
      {
        days += DsYearLength( year );
      }  
    }
    
    /* Add total seconds before partial day */
    seconds += (days * DSDAY);
  
  }
  return ( seconds );
}
