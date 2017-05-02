//#include "main.h"
#include "tmcar.h"
#include "nv080c.h"
void Init_NV080C()
{
   P0DIR |= 0x01;

}

void NV080C_Set(uint8 addr)
{
  uint8 i = 0;   
    nvc_sda=0;								
    delay_ms(5);                                                                 /*5ms */
    
    for(i=0;i<8;i++)
    {
      nvc_sda=1;
      if(addr & 1)
      {                    					
        delay_us(3517);                                              /* 1200us */
        nvc_sda=0;                   						
        delay_us(1172);                                              /* 400us */
      }
      else
      {
        delay_us(1172);                                             // 400us
        nvc_sda=0;
        delay_us(3517);                                                  //  1200
      }
      addr>>=1;
    }
    nvc_sda=1;  
    delay_ms(50);
}

void send_line(uint8 addr)
{
//    NV080C_Set(0xFE);
    NV080C_Set(addr);
}
