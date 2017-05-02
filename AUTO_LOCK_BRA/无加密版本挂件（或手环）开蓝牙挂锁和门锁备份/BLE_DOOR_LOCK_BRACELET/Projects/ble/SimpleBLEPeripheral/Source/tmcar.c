#include "tmcar.h"
#include "hal_lcd.h"

uint8 TmCarNum=0;


void delay_us(unsigned int k)//us延时函数
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
    T1CTL = 0x00;  //关闭定时器
  */
}

void delay_ms(unsigned int k)
{
    T1CC0L = 0xe8;
    T1CC0H = 0x03;
    T1CTL = 0x0a; //模模式 32分频
    while(k)
    {
        while(!((T1CNTL >= 0xe8)&&(T1CNTH >= 0x03)));
        k--;
    }
    T1CTL = 0x00; //关闭定时器
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
//发出复位脉冲
//入口:无
//出口:0=失败,1=成功,失败表示总线被钳在低电平
bool TouchReset(void)
{
  uint32 i = 0;
//  bool a;
  Tmcar_Output();       //设置为输出

  DATA_BIT=0;          //拉低总线
  delay_us(830);       //总线保持为低 500us
  DATA_BIT=1;	        //总线拉高
  delay_us(2);        // 拉高 15 - 60us    //30
  Tmcar_Input();        //设置为输入
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
//读写1位,总时间>=65us   
//入口:以CY传递参数   
//出口:以CY传递参数   
static void TouchBit(void)   
{   
 // uint8 i;   
  Tmcar_Output();
  DATA_BIT=1;
  delay_us(2);
  
  DATA_BIT=0;                      //总线拉低   
  delay_us(2);                            //延时4us   
  
  DATA_BIT=CY;                     //写入电平   
  delay_us(2);
  Tmcar_Input();
  delay_us(2);                          //延时6us   
  CY=DATA_BIT;                     //读入电平   
  delay_us(80);
  Tmcar_Output();               //延时50us,  base=11.059Mhz(精确控制)  
                      
  DATA_BIT=1;                      //释放总线   
//  Tmcar_Input();
}   
//*****************************************************   
#endif
 
//*****************************************************   
//向1-wire器件写入1个字节   
//入口:1字节数据   
//出口:无   
static void OutByte(uint8 indata)   
{   
#if 0
  uint8 i;  
//  Tmcar_Output(); 
  for (i=8;i>0;--i)              //共写8位   
  {   
    indata>>=1;           //低位先写   
    TouchBit();           //写1位   
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
//从1-wire器件读入1个字节   
//入口:无   
//出口:1字节数据   
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
//8位CRC校验,生成多项式x8+x5+x4+1   
//入口:待校验字节,CRC结果   
//出口:当次的CRC结果   
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
      if(b=(bool)((indata^crc)&0x01))      //判断与x7异或的结果(x8)   
        crc^=0x18;               //反馈到x5   x4   
      crc>>=1;                     //移位   
      if(b)   
        crc|=0x80;               //x7异或的结果送x0  
  #else 
      if(((bb^crc)&0x01))      //判断与x7异或的结果(x8)   
      {
        crc^=0x18;               //反馈到x5   x4   
        crc>>=1;                     //移位   
        crc|=0x80;               //x7异或的结果送x0   
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

    if(b=(bool)((indata^crc)&0x01))      //判断与x7异或的结果(x8)   
      crc^=0x18;               //反馈到x5   x4   
    crc>>=1;                     //移位   
    if(b)   
      crc|=0x80;               //x7异或的结果送x0  

    indata>>=1;   
  }   

  return(crc);   
} 
#endif
//*************************************************   
   
   
   
//*************************************************   
//读入DS1990A的SN,要检查复位情况和CRC,总共读10次   
//入口:指向内部RAM的指针   
//出口:返回0表示没有成功读取,指针的值不变,   
//返回1表示成功读取,指针指向最后一个字节   
bool Read1990a(uint8 *pt)   
{   
  uint8 i,crc;   

  if(TouchReset())                        //检查复位是否正确   
  {   
    
    OutByte(0x33);                  //发出读ROM命令   
    crc=0;                        //CRC校验赋初值   
//      InByte();
    for(i=0;i<7;i++)            //读出前面7个字节   
    {   
      pt[i] = InByte(); 
      crc=Data_Crc(pt[i],crc);      //进行校验  
//        HalLcdWriteValue(pt[i], 10, HAL_LCD_LINE_6);  
    }  
    
    pt[7]=InByte();                     //读出Ibutton中的CRC值  
//      HalLcdWriteValue(pt[7], 10, HAL_LCD_LINE_7);   
    if(crc==pt[7])                     //比较   
      return(1);               //成功返回1   
  }
 
  return(0);                                    //失败返回0   
}   
//*************************************************   
