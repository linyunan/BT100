/**************************************************************************************************
  Filename:       hal_sensor.c
  Revised:        $Date: 2012-09-21 06:30:38 -0700 (Fri, 21 Sep 2012) $
  Revision:       $Revision: 31581 $

  Description:    This file contains code that is common to all sensor drivers.


  Copyright 2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------------------------
*/
#include "hal_sensor.h"
#include "hal_i2c.h"
#include "hal_led.h"
#include "tmcar.h"

/* ------------------------------------------------------------------------------------------------
*                                           Macros and constants
* ------------------------------------------------------------------------------------------------
*/


/* ------------------------------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------------------------
*/
static uint8 buffer[24];
uint8 bbb[7];


/*********************************************************************
 * Globalvariables
 */
 
uint8 writeBuffer[2];
uint8 buttonStatusBackUp = 0;
uint16 buttonStatus = 0;


void DelayMS(uint16 microSecs)  //��ʱ microSecs * 1ms
{   
     uint16 i = 0,j = 0;
     //while(microSecs--) 
     for(j = microSecs; j>0; j--)
     {     
         /* 32 NOPs == 1 usecs */  
       for(i=656;i>0;i--)
       {
           asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");     
           asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");     
           asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");     
           asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");     
           asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");     
           asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");     
           asm("nop"); asm("nop");         
       }
        
     }

 }


//��ʼ���� dev_addr iic �豸��ַ
void   HalSensorInit(uint8 dev_addr )
{
//  static uint8 controlLEDFlag = 0;
  HalI2CInit(dev_addr, i2cClock_123KHZ);

   
}
 
/**************************************************************************************************
 * @fn          MBR3_I2C_Read
 *
 * @brief        дN���ֽڵ�MBR3������Ӧ�Ĵ�����
 *               
 *
 * @param       addr - which register to read
 * @param       buf - pointer to buffer to place data
 * @param       len - numbver of bytes to read
 *
 * @return      0:��    -1:��
 
 **************************************************************************************************/
uint8 MBR3_I2C_Read( uint8 addr, uint8 len, uint8 *buf)
{
    bool ret;

    ret = HalSensorReadReg( addr, buf, len );

    return ret ? 0 : -1;
}

/**************************************************************************************************
 * @fn          MBR3_I2C_Write
 *
 * @brief        дN���ֽڵ�MBR3������Ӧ�Ĵ�����
 *               
 *
 * @param       addr - which register to read
 * @param       buf - pointer to buffer to place data
 * @param       len - numbver of bytes to read
 *
 * @return      0:��    -1:��
 
 **************************************************************************************************/
uint8 MBR3_I2C_Write( uint8 addr, uint8 len, uint8 *buf)
{		
    bool ret;
    
    ret = HalSensorWriteReg(addr, buf, len);

    return ret ? 0 : -1;
}


/**************************************************************************************************
 * @fn          HalSensorReadReg
 *
 * @brief       This function implements the I2C protocol to read from a sensor. The sensor must
 *              be selected before this routine is called.
 *
 * @param       addr - which register to read
 * @param       pBuf - pointer to buffer to place data
 * @param       nBytes - numbver of bytes to read
 *
 * @return      TRUE if the required number of bytes are reveived
 **************************************************************************************************/
bool HalSensorReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes)
{
  uint8 i = 0;

  /* Send address we're reading from */
  if (HalI2CWrite(1,&addr) == 1)
  {
    /* Now read data */
    i = HalI2CRead(nBytes,pBuf);
  }

  return i == nBytes;
}

/**************************************************************************************************
* @fn          HalSensorWriteReg
* @brief       This function implements the I2C protocol to write to a sensor. he sensor must
*              be selected before this routine is called.
*
* @param       addr - which register to write
* @param       pBuf - pointer to buffer containing data to be written
* @param       nBytes - number of bytes to write
*
* @return      TRUE if successful write
*/
bool HalSensorWriteReg(uint8 addr, uint8 *pBuf, uint8 nBytes)
{
  uint8 i;
  uint8 *p = buffer;

  /* Copy address and data to local buffer for burst write */
  *p++ = addr;
  for (i = 0; i < nBytes; i++)
  {
    *p++ = *pBuf++;
  }
  nBytes++;

  /* Send address and data */
  i = HalI2CWrite(nBytes, buffer);
  //if ( i!= nBytes)
  //  HAL_TOGGLE_LED2();

  return (i == nBytes);
}


/**************************************************************************************************
 * @fn          MBR3_RESET
 *
 * @brief       ͨ��дMBR3��Ӧ�Ĵ�������λ MBR3����оƬ
 *
 * input parameters
 *
 * None
 *
 * output parameters
 *
 * None.
 *
 * @return      
 * None.
 */

uint8 MBR3_RESET(void)
{
    uint8 j = 0;
    uint8 writeBuffer[1] = {0};
      /*��λMBR3 */
   // writeBuffer[0] = CTRL_CMD;
    writeBuffer[0] = 0xff;  
    HalSensorInit(MBR3_DEV_ADDR);
//    MBR3_I2C_Configuration( ); //MBR3Ѱַ����
    while(  ( MBR3_I2C_Write( CTRL_CMD, 1, &writeBuffer[0]) ) && j<4 )
    {      
       j++; 
    }		
    if(j>=4)return 0; 
    
    //DelayMS(200); //��ʱ200MS���ȴ�оƬ�ȶ�
    
    return 1;
          
}


/**************************************************************************************************
 * @fn          ReadButtonStatus
 *
 * @brief       ��ȡ����״̬�Ĵ���ֵ�����Ѷ�����ֵ��ֵ��ȫ�ֱ���
 *
 * input parameters
 *
 * None
 *
 * output parameters
 *
 * None.
 *
 * @return      
 * None.
 */

int8  ReadButtonStatus(void)
{ 
    uint8 BUTNUM = 0xff; //��ʼ��Ϊ0xff δ���»��ɿ����ֵ
    uint8 readBuffer[2];
    uint8 j = 0;
    readBuffer[0] = 0;
    readBuffer[1] = 0;
  
    HalSensorInit(MBR3_DEV_ADDR);
//    MBR3_I2C_Configuration( ); //MBR3Ѱַ����
    
    buttonStatus = 0;//��ȡ����֮ǰ������״ֵ̬����
  

    while(   ( MBR3_I2C_Read( BUTTON_STATUS,  2 ,  &readBuffer[0] )  )  && j<4 ) j++;
    if( j>=4)
    {
        MBR3_RESET(); 
        return -1;//-1 : ��ʾʧ��;   0 ,1,2,3,4****:  ��Ȼ����ʾ�ɹ���������
    }
/*
    if(( MBR3_I2C_Read( BUTTON_STATUS,  2 ,  &readBuffer[0] )))
    {
      return (-1);
    }
*/
    /* Store the status in a global variable */

    buttonStatus =  0x1fff & ( ( readBuffer[1]<<8 ) | readBuffer[0]  ); //1111 1111 1111 1111




   switch( buttonStatus )
   {
   	case 0x0400: //����0
		BUTNUM = 0x30;
	break;

   	case 0x0020: //����1
		BUTNUM = 0x31;
	break;	
	
   	case 0x0010: //����2
		BUTNUM = 0x32;
	break;

   	case 0x0004: //����3
		BUTNUM = 0x33;
	break;
	
   	case 0x0080: //����4
		BUTNUM = 0x34;
	break;

   	case 0x0040: //����5
		BUTNUM = 0x35;
	break;
	
   	case 0x0002: //����6
		BUTNUM = 0x36;
	break;

   	case 0x0100: //����7
		BUTNUM = 0x37;
	break;
	
   	case 0x0800: //����8
		BUTNUM = 0x38;
	break;

   	case 0x0008: //����9
		BUTNUM = 0x39;
	break;
	
   	case 0x0200: //����*
		BUTNUM = 0x0a;
	break;

   	case 0x1000: //����#
		BUTNUM = 0x0b;
	break;
   	case 0x0000:   //����δ���»��ɿ��󣬼Ĵ�����ֵΪ0x0000
		BUTNUM = 0xff;
	break;	
	default:		 
		break;                 


   }
   
   return BUTNUM; 

}

/**************************************************************************************************
 * @fn          BackLight_ON
 *
 * @brief       MBR3������� ������ ����
 *
 * input parameters
 *
 * None
 *
 * output parameters
 *
 * None.
 *
 * @return      
 * None.
 */

void BackLight_ON( void )
{
    uint8 j = 0;
    uint8 writeBuffer[1] = {0};        
    writeBuffer[0] |= BV(6); //0100 0000

//    MBR3_I2C_Configuration( ); //MBR3Ѱַ����
    HalSensorInit(MBR3_DEV_ADDR);
    while( ( MBR3_I2C_Write(GPO_OUTPUT_STATE, 1, &writeBuffer[0])  )  && j<10 ) j++;
    if( j>=10)
    {
        MBR3_RESET(); 
    }
	
}

/**************************************************************************************************
 * @fn          BackLight_OFF
 *
 * @brief       MBR3������� ������ Ϩ��
 *
 * input parameters
 *
 * None
 *
 * output parameters
 *
 * None.
 *
 * @return      
 * None.
 */

void BackLight_OFF( void )
{
    uint8 j = 0;
    uint8 writeBuffer[1] = {0};      
    writeBuffer[0] &= ~( BV(6) ); //0100 0000
    
//    MBR3_I2C_Configuration( ); //MBR3Ѱַ����
    HalSensorInit(MBR3_DEV_ADDR);
    while( ( MBR3_I2C_Write(GPO_OUTPUT_STATE, 1, &writeBuffer[0])  )  && j<10 ) j++;
    if( j>=10)
    {
        MBR3_RESET(); 
    } 
   


}





/*********************************************************************
*********************************************************************/





/*********************************************************************
*********************************************************************/

