#include <ioCC2541.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "pwm.h"


#include "hal_mcu.h"
//������Ϊamomcu�ο������ϵ�����

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
    T3CTL |= 0xA0;              // 32��Ƶ    
    
    T3CCTL0 &= ~0x40;           // Disable channel 0 interrupts    
    T3CCTL0 |= 0x04;            // Compare mode    
    T3CCTL0 |= 0x10;            // Ch0 output compare mode = toggle on compare  
    
    T3CC0 = 0;                  // ���� * 2   ÿ��Ϊ 1us  �磺����T3CC0 = 10  ��20us    
    
    T3CTL |= 0x10;             // start timer 3    

//  IEN1 |= 0x02;               // Enable T1 cpu interrupt
}

//red�� green�� blue ��ֵ������ 1~375, ����ֵ��Ч
void PWM_Pulse(uint8 data)
{

  // Set up the timer registers

  T3CC0 = (uint8)data;

  //����Ƚ�ֵΪ0ʱ�����һֱΪ��
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
//   IRCON &= ~0x08;       //��ʱ��3�жϱ�־λ��0

//   TIMIF &= ~0x01;       //��ʱ��3����жϱ�־λ��0
//  IRCON &= ~0x08;

//  TIMIF &= ~0x11;
  
//  T3CTL |= 0x10;        //�ٴ�������ʱ��3
  
   if(++count>5000)       //256us *5000 = 1.28s������˸
  {
  // LED3 = ~LED3;
   count = 0;              // ��������
   num++;                                  //�����л�PWM������
   }
   if(num==3)
   {
    T3CC0 = 200;        //200us  ������࣬LED1Ӧ����Щ

   }
   else if(num ==6 )
   {
    num = 0;
    T3CC0 = 10;          //10us �������١�LED1Ӧ�ûᰵЩ

   }
    
}
#endif