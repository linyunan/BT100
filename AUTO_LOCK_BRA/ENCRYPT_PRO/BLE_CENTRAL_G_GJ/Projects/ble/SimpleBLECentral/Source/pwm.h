#ifndef _PWM_H_
#define _PWM_H_

typedef struct
{
    uint16 gRed ;
    uint16 gGreen ;
    uint16 gBlue ;
    uint16 gWhite ;
} pwm_rgbw;

void PWM_Init();

void PWM_Pulse(uint16 red);
  
void Buzzer_Init(void);    
#endif