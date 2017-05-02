//******************************************************************************
// 文件名   ：adc.h
// 创建时间 ：2016-04-22
// 当前版本 ：V1.0
// 功能描述 ：采样ADC数据
//******************************************************************************
#ifndef ADC_H
#define ADC_H

//******************************************************************************
// 头文件
#include "hal_types.h"

//******************************************************************************
// 功能函数
// ADC Measure
extern void ADCInit(void);
extern uint16 BatMeasure( void );
extern uint16 BatADMeasure( void );


extern void Adc_Stop(void);
extern void Adc_Start(void);

#define RefVoltage   3.3  //参考电压
#endif // ADC_H

