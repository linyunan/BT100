#include <ioCC2541.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "pwm.h"


#include "hal_mcu.h"
//此资料为amomcu参考网络上的资料

//pwm pins:
//P0_3:R  (TX)--ch1
//P0_4:G  (CTS)--ch2
//P0_5:B  (RTS)--ch3
//P0_6:w 
/*
static uint16 gRed =1;
static uint16 gGreen =1;
static uint16 gBlue =1;
static uint16 gWhite =1;
*/

//extern SYS_CONFIG  sys_config;

pwm_rgbw PWM_rgbw;

void PWM_Init()
{
   
    PERCFG |= 0x20;             // Timer 3 Alternate location 2    
    P1DIR |= BV(3);              // P1_3 = output    
    P1SEL |= BV(3);              // Peripheral function on P1_3    
    
    T3CTL &= ~0x10;             // Stop timer 3 (if it was running)    
    T3CTL |= 0x04;              // Clear timer 3    
    T3CTL &= ~0x08;             // Disable Timer 3 overflow interrupts    
    T3CTL |= 0x03;              // Timer 3 mode = 3 - Up/Down    
    T3CTL |= 0xA0;              // 32分频    
    
    T3CCTL0 &= ~0x40;           // Disable channel 0 interrupts    
    T3CCTL0 |= 0x04;            // Compare mode    
    T3CCTL0 |= 0x10;            // Ch0 output compare mode = toggle on compare  
    
    T3CC0 = 0;                  // 次数 * 2   每次为 1us  如：设置T3CC0 = 10  即20us    
    
    T3CTL |= 0x10;             // start timer 3    

//  IEN1 |= 0x02;               // Enable T1 cpu interrupt
}

//red， green， blue 的值必须是 1~375, 其他值无效
void PWM_Pulse(uint8 data)
{

  // Set up the timer registers

  T3CC0 = (uint8)data;

  //避免比较值为0时，输出一直为高
  if(data!=0){
    T3CCTL0 &= ~0x40;           // Disable channel 0 interrupts    
    T3CCTL0 |= 0x04;            // Compare mode    
    T3CCTL0 |= 0x10;            // Ch0 output compare mode = toggle on compare  
  }else{
    T3CCTL0 = 0x00;
  }  
}
int count =0,num = 0;
#if 1 
#pragma vector = T3_VECTOR
__interrupt void pwmISR (void) 
{
//   IRCON &= ~0x08;       //定时器3中断标志位清0

//   TIMIF &= ~0x01;       //定时器3溢出中断标志位清0
//  IRCON &= ~0x08;

//  TIMIF &= ~0x11;
  
//  T3CTL |= 0x10;        //再次启动定时器3
  
   if(++count>5000)       //256us *5000 = 1.28s周期闪烁
  {
  // LED3 = ~LED3;
   count = 0;              // 计数清零
   num++;                                  //周期切换PWM的脉宽
   }
   if(num==3)
   {
    T3CC0 = 200;        //200us  亮脉宽多，LED1应该亮些

   }
   else if(num ==6 )
   {
    num = 0;
    T3CC0 = 10;          //10us 亮脉宽少。LED1应该会暗些

   }
    
}
#endif