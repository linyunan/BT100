#ifndef __tmcar__h_
#define __tmcar__h__

#include "OnBoard.h"
#include "hal_types.h"

#define DATA_BIT   P0_6

extern void delay_us(unsigned int k);//usÑÓÊ±º¯Êý
extern void delay_ms(unsigned int k);
extern bool Read1990a(uint8 *pt);   
#if 0
static   bool   Data_Crc(uint8   *src ,uint8   crc) ;
#else
static   bool   Data_Crc(uint8   indata ,uint8   crc) ;
#endif
static uint8 InByte(void);
bool TouchReset(void);

extern void Tmcar_Input(void);
extern void Tmcar_Output(void);

extern void TmLed_Init(void);
extern void TmLed_True(void);
extern void TmLed_False(void);
extern void TmLed_Close(void);

#endif