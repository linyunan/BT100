#include "motor.h"
#include "hal_sensor.h"

void Motor_Init(void)
{
  P1SEL &= ~(BV(1)); // Configure Port 1 as GPIO
  P1SEL &= ~(BV(2)); // Configure Port 1 as GPIO
  P1DIR |= BV(1); // All port 1 pins (P1.2) as output
  P1DIR |= BV(2); // All port 1 pins (P1.3) as output

  P1 &= ~(BV(1));   // All pins on port 1 to low
  P1 &= ~(BV(2));   // All pins on port 1 to low
}

#if 1
void Motor_Foreward(void)
{
  
  P1 |= (BV(1));   // All pins on port 1.2 to High
  P1 &= ~(BV(2));   // All pins on port 1.3 to low
}

void Motor_Backward(void)
{
  P1 &= ~(BV(1));   // All pins on port 1.2 to low
  P1 |= (BV(2));   // All pins on port 1.3 to High
}
#else
void Motor_Backward(void)
{
  
  P1 |= (BV(1));   // All pins on port 1.2 to High
  P1 &= ~(BV(2));   // All pins on port 1.3 to low
}

void Motor_Foreward(void)
{
  P1 &= ~(BV(1));   // All pins on port 1.2 to low
  P1 |= (BV(2));   // All pins on port 1.3 to High
}  
#endif

void Motor_Stop(void)
{
  P1 &= ~(BV(1));   // All pins on port 1 to low
  P1 &= ~(BV(2));   // All pins on port 1 to low
}


void KeyLock_InMode(void)
{
  P0SEL &= ~(BV(5));
  P0DIR &= ~(BV(5));

}

void KeyLock_OutMode(void)
{
  P0SEL &= ~(BV(5));
  P0DIR |= (BV(5));

}

void Buzzer_Init(void)
{
  P1SEL &= ~(BV(3));
  P1DIR |= BV(3);
  P1 |= (BV(3)); // 初始化输出0
}

void Buzzer_Open(void)
{
  P1 &= ~(BV(3)); // 初始化输出0
}

void Buzzer_Close(void)
{
  P1 |= (BV(3)); // 初始化输出1
  
}
