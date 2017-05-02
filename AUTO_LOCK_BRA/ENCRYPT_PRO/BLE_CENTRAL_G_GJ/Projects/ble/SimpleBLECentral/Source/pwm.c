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

void Buzzer_Init(void)
{
  //设置pwm端口为输出
  P0DIR|= BV(3);
  //设置pwm端口为外设端口，非gpio
  P0SEL &= ~(BV(3));
  
  P0 &= ~(BV(3));
}

pwm_rgbw PWM_rgbw;

void PWM_Init()
{
//  EA = 0 ;

  //设置pwm端口为输出
  P0DIR|= BV(3);
  //设置pwm端口为外设端口，非gpio
  P0SEL|= BV(3);

  
  //由于uart等会占用我们当前使用的pwm端口，因此需要将uart等重映射到别的端口去。
  PERCFG |= 0x03;             // Move USART1&2 to alternate2 location so that T1 is visible
//  PERCFG |= 0x03;             // Move USART1&2 to alternate2 location so that T1 is visible
    
//  PERCFG = (PERCFG & ~0x40) | 0x03; // Select Timer 1 Alternative 0 location, set U1CFG and U0CFG to Alternative 1
//  Initialize Timer 1
  T1CTL = 0x0A;               // Div = 128, CLR, MODE = Suspended          
  T1CCTL1 = 0x00;             // IM = 0; CMP = Clear output on compare; Mode = Compare

  T1CNTL = 0x00;                 // Reset timer to 0;
  T1CNTH = 0x00;
    //必须设置，否则定时器不工作
  T1CCTL0 = 0x4C;            // IM = 1, CMP = Clear output on compare; Mode = Compare

  //设置周期的tick为375, 也就是1.5ms
#if 1
  T1CC0H = 0x00;              // Ticks = 375 (1.5ms initial duty cycle)
  T1CC0L = 0xFA;              //             
  T1CC1H = 0x00;              // Ticks = 375 (1.5ms initial duty cycle)
  T1CC1L = 0x00;

#else//以下设置会是1khz
#define VALUE_H     0x00
#define VALUE_L     0x10
  T1CC0H = VALUE_H;    
  T1CC0L = VALUE_L;    
  T1CC1H = VALUE_H;    
  T1CC1L = VALUE_L;
  T1CC2H = VALUE_H;    
  T1CC2L = VALUE_L;
  T1CC3H = VALUE_H;    
  T1CC3L = VALUE_L;  
#endif 

//  EA=1;
//  IEN1 |= 0x02;               // Enable T1 cpu interrupt
}

//red， green， blue 的值必须是 1~375, 其他值无效
void PWM_Pulse(uint16 red )
{
  uint16 r;

  r = red;
  // stop,注意，不能加这句，加了周期偏差十几倍，具体原因未查明
//  T1CTL &= BV(0)|BV(1); 

  T1CC1L = (uint8)r;
  T1CC1H = (uint8)(r >> 8);
  //避免比较值为0时，输出一直为高
  if(r!=0){
    T1CCTL1 = 0x24;
  }else{
    T1CCTL1 = 0x00;
  }

  // Reset timer
//  T1CNTL = 0x0;
//  T1CNTH = 0;

  // Start timer in modulo mode.
//  T1CTL |= 0x02;   
}



//#pragma register_bank=2
#pragma vector = T1_VECTOR
__interrupt void pwmISR (void) 
{
    uint8 flags = T1STAT;
#if 1
    // T1 ch 0
    if (flags & 0x01){          
      
      // Stop Timer 1

      T1CNTL = 0x00;
//      T1CNTH = 0x00;
      
    }
#else
#endif
   
//    PWM_Pulse(PWM_rgbw.gRed,PWM_rgbw.gGreen,PWM_rgbw.gBlue,PWM_rgbw.gWhite);
    T1STAT = ~ flags;
}
