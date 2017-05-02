#ifndef __motor__h_
#define __motor__h__

#include "OnBoard.h"

extern void Motor_Init(void);
extern void Motor_Foreward(void);
extern void Motor_Backward(void);
extern void Motor_Stop(void);

void KeyLock_InMode(void);      // ��λ���� ����
void KeyLock_OutMode(void);     // ��λ���� ���

void Buzzer_Init(void);
void Buzzer_Close(void);
void Buzzer_Open(void);
#endif