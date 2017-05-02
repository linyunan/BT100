#include "wdog.h"
#include "ioCC2541.h"



//////////////////////////////////初始化看门狗///////////////////////////////////////
void watchdog_init(void)
{
  WDCTL = 0x00;     //时间间隔为1秒，看门狗模式；   
  WDCTL |= 0x08;     //启动看门狗；
}
///////////////////////////////////喂狗函数//////////////////////////////////////////////
void feetdog(void)
{
   WDCTL = 0xa0;
   WDCTL = 0x50;
}


void Closedog(void)
{
  WDCTL |= ~((1<<2)|(1<<3)); 
}

void Opendog(void)
{ 
  WDCTL = 0x00;     //时间间隔为1秒，看门狗模式；   
  WDCTL |= 0x08;     //启动看门狗；
}