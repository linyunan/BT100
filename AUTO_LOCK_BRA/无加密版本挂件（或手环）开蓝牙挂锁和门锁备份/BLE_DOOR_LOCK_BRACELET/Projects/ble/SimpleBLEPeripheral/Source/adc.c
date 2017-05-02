//******************************************************************************
// 文件名   ：adc.c
// 创建时间 ：2014-04-22
// 当前版本 ：V1.0
// 功能描述 ：采样ADC数据
//******************************************************************************

//******************************************************************************
// 头文件
#include "adc.h"
#include "hal_adc.h"
#include "bledoorlock.h"
//#include "doorGATTprofile.h"
//******************************************************************************
// 宏定义

//******************************************************************************
// 函数名   ：ADCInit
// 函数功能 ：ADC初始化
// 输入变量 ：
// 输出变量 ：
//******************************************************************************
void ADCInit(void)
{
  //P0_7做为 ADC设置
  APCFG |= BV(7);    //analog io enabled 
  P0SEL |= BV(7);   //peripheral function  
  P0DIR &= ~(BV(7));  //input  
//  P0INP |= BV(7);    //3 state  
  //P1_0做为控制是否读取电量
  P1DIR |= BV(0); // Port 1.0 as output,

  P1SEL &= ~(BV(0)); // Configure Port 1.0 as GPIO
  
  P1 &= ~(BV(0));   //初始化为不读取电量
}

void Adc_Stop(void)
{
	P1 &= ~(BV(0));
}

void Adc_Start(void)
{
	P1 |= BV(0);
}
//******************************************************************************
// 函数名   ：ADC Measure
// 函数功能 ：ADC采样
// 输入变量 ：
// 输出变量 ：adc
//******************************************************************************
uint16 BatADMeasure( void )
{
  // Configure ADC and perform a read
 // HalAdcSetReference( HAL_ADC_REF_AVDD );HAL_ADC_REF_125V
  HalAdcSetReference( HAL_ADC_REF_125V );
  return HalAdcRead( HAL_ADC_CHANNEL_7, HAL_ADC_RESOLUTION_12 );
}


uint16 BatMeasure(void)
{
  /*
  uint16 BatAD = 0 ;6
  uint16 BatValue ;
  BatAD = BatADMeasure();
  BatValue = BatAD *RefVoltage /4096 * 600 ; // 便于测试 多+10
  return (uint16)BatValue ;
  */
//  uint8 res[4] = {0};
  uint16 BatAD = 0 ;
  uint16 BatValue ;
  BatAD = BatADMeasure();

//  res[0] = BatAD/1000;
//  res[1] = BatAD%1000/100;
//  res[2] = BatAD%1000%100/10;
//  res[3] = BatAD%1000%100%10;
  

 // BatValue =(uint16) (((BatAD *RefVoltage /2048)* (1499/499))*100) ; 
  BatValue =(uint16) ((BatAD *1.25 /2048)*(4600/1000)* 1000) ; 
//  BatValue =(uint16) ((BatAD *1.25 /2048)* 1000) ;
//  BatValue =(uint16) ((BatAD *3.28 /2048)*(95/20)*100) ;   //  3.3v
  //(1499/499))*100)
  return (uint16)BatValue;
  
}


