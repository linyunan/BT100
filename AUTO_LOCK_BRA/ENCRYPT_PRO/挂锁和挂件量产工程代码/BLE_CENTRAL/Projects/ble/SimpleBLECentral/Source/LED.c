#include "led.h"


void Led_Init(void)
{
  P1SEL &= ~(BV(5)); // Configure Port 1 as GPIO

  P1DIR |= BV(5); // All port 1 pins (P1.2) as output

  P1 &= ~(BV(5));   // All pins on port 1 to low

}


void LED_ON(void)
{
  
  P1 |= (BV(5));   // All pins on port 1.2 to High
}

void LED_OFF(void)
{
  P1 &= ~(BV(5));   // All pins on port 1.2 to low
}


