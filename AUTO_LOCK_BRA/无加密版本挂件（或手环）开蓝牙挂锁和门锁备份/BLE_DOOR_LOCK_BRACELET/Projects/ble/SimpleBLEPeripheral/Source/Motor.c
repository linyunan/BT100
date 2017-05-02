#include "motor.h"


void Motor_Init(void)
{
  P1SEL &= ~(BV(2)); // Configure Port 1 as GPIO
  P1SEL &= ~(BV(3)); // Configure Port 1 as GPIO
  P1DIR |= BV(2); // All port 1 pins (P1.2) as output
  P1DIR |= BV(3); // All port 1 pins (P1.3) as output

  P1 &= ~(BV(2));   // All pins on port 1 to low
  P1 &= ~(BV(3));   // All pins on port 1 to low
}


void Motor_Foreward(void)
{
  
  P1 |= (BV(2));   // All pins on port 1.2 to High
  P1 &= ~(BV(3));   // All pins on port 1.3 to low
}

void Motor_Backward(void)
{
  P1 &= ~(BV(2));   // All pins on port 1.2 to low
  P1 |= (BV(3));   // All pins on port 1.3 to High
}

void Motor_Stop(void)
{
  P1 &= ~(BV(2));   // All pins on port 1 to low
  P1 &= ~(BV(3));   // All pins on port 1 to low
}

