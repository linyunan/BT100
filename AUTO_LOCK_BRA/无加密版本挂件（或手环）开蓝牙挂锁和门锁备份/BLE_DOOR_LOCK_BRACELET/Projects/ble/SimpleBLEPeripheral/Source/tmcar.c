#include "tmcar.h"
#include "hal_lcd.h"

uint8 TmCarNum=0;


void delay_us(unsigned int k)//us��ʱ����
{
  while(--k);
//  for(uint8 i=0;i<k;i++);
//    for()
//  asm("nop");
//  asm("nop");
//  asm("nop");
  /*
    T1CC0L = 0x0A; 
    T1CC0H = 0x00; 
    T1CTL = 0x09; 
    while(k)
    { 
        while(!(T1CNTL >= 0x0A));
        k--;
    }
    T1CTL = 0x00;  //�رն�ʱ��
  */
}

void delay_ms(unsigned int k)
{
    T1CC0L = 0xe8;
    T1CC0H = 0x03;
    T1CTL = 0x0a; //ģģʽ 32��Ƶ
    while(k)
    {
        while(!((T1CNTL >= 0xe8)&&(T1CNTH >= 0x03)));
        k--;
    }
    T1CTL = 0x00; //�رն�ʱ��
}

void Tmcar_Input(void)
{
  P0DIR &= ~(BV(6));
}

void Tmcar_Output(void)
{
  P0DIR |= BV(6);
}    
    
void RST1990()
{ 
  DATA_BIT=0;
  delay_us(50);       
  DATA_BIT=1;	
  delay_ms(2);
}

void TmLed_Init(void)
{
  P0SEL &= ~(BV(4));    // Configure Port 0.4 as GPIO
  P0SEL &= ~(BV(5));    // Configure Port 0.5 as GPIO
  P0SEL &= ~(BV(6));    // Configure Port 0.6 as GPIO
  P0DIR |= BV(4);       // port 0 pins (P0.4) as output
  P0DIR |= BV(5);       // port 0 pins (P0.5) as output
  P0DIR |= BV(6);       // port 0 pins (P0.6) as output
  P0 &= ~(BV(4));       // pin0.4 on port 0 to low
  P0 &= ~(BV(5));       // pin0.5 on port 0 to low
  P0 &= ~(BV(6));       // pin0.6 on port 0 to low
}

void TmLed_True(void)
{
  P0 &= ~(BV(4));   // All pins on port 0.4 to low
  P0 |= (BV(5));   // All pins on port 0.5 to high
}

void TmLed_False(void)
{
  P0 |= (BV(4));   // All pins on port 0.4 to high
  P0 &= ~(BV(5));   // All pins on port 0.5 to low
}

void TmLed_Close(void)
{
  P0 &= ~(BV(4));   // All pins on port 0.4 to low
  P0 &= ~(BV(5));   // All pins on port 0.5 to low
}

//*************************************************
//������λ����
//���:��
//����:0=ʧ��,1=�ɹ�,ʧ�ܱ�ʾ���߱�ǯ�ڵ͵�ƽ
bool TouchReset(void)
{
  uint32 i = 0;
//  bool a;
  Tmcar_Output();       //����Ϊ���

  DATA_BIT=0;          //��������
  delay_us(830);       //���߱���Ϊ�� 500us
  DATA_BIT=1;	        //��������
  delay_us(2);        // ���� 15 - 60us    //30
  Tmcar_Input();        //����Ϊ����
#if 1
  while(i<3000)
  {
    i++;
    asm("nop");
    asm("nop");

    if(!DATA_BIT)
    {
      break;
    }
  }
  if(i>=2999)
  {
    return (0);
  }
  i = 0;
  while(i<3000)
  {
    i++;
    asm("nop");
    asm("nop");
    asm("nop");
    if(DATA_BIT)
    {
      break;
    }
  }
  if(i>=2999)
  {
    return (0);
  }
  delay_us(400);
#else
//  i = 0;
  while(DATA_BIT);
  while(!DATA_BIT);
#endif
  return (DATA_BIT);

}
#if 0
//*****************************************************   
//��д1λ,��ʱ��>=65us   
//���:��CY���ݲ���   
//����:��CY���ݲ���   
static void TouchBit(void)   
{   
 // uint8 i;   
  Tmcar_Output();
  DATA_BIT=1;
  delay_us(2);
  
  DATA_BIT=0;                      //��������   
  delay_us(2);                            //��ʱ4us   
  
  DATA_BIT=CY;                     //д���ƽ   
  delay_us(2);
  Tmcar_Input();
  delay_us(2);                          //��ʱ6us   
  CY=DATA_BIT;                     //�����ƽ   
  delay_us(80);
  Tmcar_Output();               //��ʱ50us,  base=11.059Mhz(��ȷ����)  
                      
  DATA_BIT=1;                      //�ͷ�����   
//  Tmcar_Input();
}   
//*****************************************************   
#endif
 
//*****************************************************   
//��1-wire����д��1���ֽ�   
//���:1�ֽ�����   
//����:��   
static void OutByte(uint8 indata)   
{   
#if 0
  uint8 i;  
//  Tmcar_Output(); 
  for (i=8;i>0;--i)              //��д8λ   
  {   
    indata>>=1;           //��λ��д   
    TouchBit();           //д1λ   
  }   
#else
  uint8 temp=0;
  temp = indata;
  Tmcar_Output();
  for(uint8 i=0;i<8;i++)
  {
    
    DATA_BIT=1;
    delay_us(2);          // delay 4us
    DATA_BIT=0;
    delay_us(20);          // delay 4us
    DATA_BIT=(temp&0x01);
    temp>>=1;
    delay_us(80);         // delay 60us
  }
  DATA_BIT=1;
#endif
}  


//*****************************************************   
//��1-wire��������1���ֽ�   
//���:��   
//����:1�ֽ�����   
static   uint8   InByte(void)   
{   

  uint8 temp = 0;
  TmCarNum = 0;
  
  for(uint8 i=0;i<8;i++)
  {
    Tmcar_Output();
    DATA_BIT=1;
//    asm("nop");//asm("nop");asm("nop");asm("nop");
    DATA_BIT=0;
    asm("nop");asm("nop");asm("nop");asm("nop");
    DATA_BIT=1;
    asm("nop");asm("nop");asm("nop");asm("nop");          // delay 4us
    Tmcar_Input();
    asm("nop");//asm("nop");
    temp=DATA_BIT&0x01;
    
    TmCarNum |= (temp<<i);
    delay_us(80);         // delay 60us
//    Tmcar_Output();
//    DATA_BIT=1;    
//    asm("nop");asm("nop");//asm("nop");asm("nop");          // delay 4us
      
  }
  
  return(TmCarNum);


     
}   
//*************************************************   
   
   
//*************************************************   
//8λCRCУ��,���ɶ���ʽx8+x5+x4+1   
//���:��У���ֽ�,CRC���   
//����:���ε�CRC���   
#if 0
static   bool   Data_Crc(uint8   *src ,uint8   crc)   
{   
  uint8 i;   
  uint8 bb; 
  for(uint8 j=0;j<7;j++)
  {
    bb = src[j];
    for(i=8;i>0;--i)   
    {   
      
  #if 0
      if(b=(bool)((indata^crc)&0x01))      //�ж���x7���Ľ��(x8)   
        crc^=0x18;               //������x5   x4   
      crc>>=1;                     //��λ   
      if(b)   
        crc|=0x80;               //x7���Ľ����x0  
  #else 
      if(((bb^crc)&0x01))      //�ж���x7���Ľ��(x8)   
      {
        crc^=0x18;               //������x5   x4   
        crc>>=1;                     //��λ   
        crc|=0x80;               //x7���Ľ����x0   
      }
      else
      {
        crc>>=1;
      }  
        
  #endif 
      bb>>=1;   
    }   
  }
  if(crc == src[7])  return 1;
  else  return 0;
//  return(crc);   
}   
#else

static   bool   Data_Crc(uint8   indata ,uint8   crc)   
{   
  uint8 i; 
  bool b;  

  for(i=8;i>0;--i)   
  {   

    if(b=(bool)((indata^crc)&0x01))      //�ж���x7���Ľ��(x8)   
      crc^=0x18;               //������x5   x4   
    crc>>=1;                     //��λ   
    if(b)   
      crc|=0x80;               //x7���Ľ����x0  

    indata>>=1;   
  }   

  return(crc);   
} 
#endif
//*************************************************   
   
   
   
//*************************************************   
//����DS1990A��SN,Ҫ��鸴λ�����CRC,�ܹ���10��   
//���:ָ���ڲ�RAM��ָ��   
//����:����0��ʾû�гɹ���ȡ,ָ���ֵ����,   
//����1��ʾ�ɹ���ȡ,ָ��ָ�����һ���ֽ�   
bool Read1990a(uint8 *pt)   
{   
  uint8 i,crc;   

  if(TouchReset())                        //��鸴λ�Ƿ���ȷ   
  {   
    
    OutByte(0x33);                  //������ROM����   
    crc=0;                        //CRCУ�鸳��ֵ   
//      InByte();
    for(i=0;i<7;i++)            //����ǰ��7���ֽ�   
    {   
      pt[i] = InByte(); 
      crc=Data_Crc(pt[i],crc);      //����У��  
//        HalLcdWriteValue(pt[i], 10, HAL_LCD_LINE_6);  
    }  
    
    pt[7]=InByte();                     //����Ibutton�е�CRCֵ  
//      HalLcdWriteValue(pt[7], 10, HAL_LCD_LINE_7);   
    if(crc==pt[7])                     //�Ƚ�   
      return(1);               //�ɹ�����1   
  }
 
  return(0);                                    //ʧ�ܷ���0   
}   
//*************************************************   
