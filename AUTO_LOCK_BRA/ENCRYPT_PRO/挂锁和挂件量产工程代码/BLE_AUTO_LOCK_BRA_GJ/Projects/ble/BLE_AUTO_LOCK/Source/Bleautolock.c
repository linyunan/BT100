/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_sensor.h"
#include "hal_i2c.h"
#include "hal_lcd.h"
#include "hal_aes.h"
#include "hal_dma.h"
#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"

#include "lockGATTprofile.h"

#if defined( CC2540_MINIDK )
//  #include "simplekeys.h"
#endif

#include "peripheral.h"

#include "gapbondmgr.h"

#include "Bleautolock.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif


//#ifdef wdog
  #include "wdog.h"
//#endif

#include "stdio.h"
#include "osal_clock.h"
#include "OSAL_Timers.h"
#include <string.h>
#include "osal_snv.h"
#include "motor.h"
#include <math.h>
#include <stdlib.h>
#include "adc.h"
#include "hal_crc.h"

#include "simpleble.h"

#include "ds1302.h"
    
#include "osal_snv.h"
//#include "pwm.h"
#include "OLED_MENU.h"
#include "phy.h"
uint8 key[] = { 0x01, 0x03, 0x05, 0x07, 0xFF, 0xFE, 0x77, 0x88, 0x12, 0x99, 0x32, 0x41, 0x14, 0x24, 0x25, 0x36}; 
//                          0x67, 0x00, 0xCE, 0xAA, 0xAB, 0xBB, 0xDF};

#if 0
#pragma constseg = "MY_MAC_CODE"
__no_init uint8 myCode[6] @0xFFF9;
#pragma constseg = default


#pragma constseg = "FLASH_LOCK_BITS"
__no_init uint8 myCode[16] @0x7FFF0;
#endif
#if 0
#pragma location = "FLASH_LOCK_BITS" 
//__no_init uint8 arry[16] @0x7FFF0;
__root const char lockbits[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define DOOR_PERIODIC_EVT_PERIOD                   300


// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160       // 广播间隔， 数值越大功耗越低但是广播的包的时间间隔就太长

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8//4//80//  连接间隔与数据发送量有关， 连接间隔越短， 单位时间内就能发送越多的数据

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     10//10//800//  连接间隔与数据发送量有关， 连接间隔越短， 单位时间内就能发送越多的数据

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000//1000  -各种原因断开连接后，超时并重新广播的时间:  100 = 1s

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         1

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 bleAutoLock_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;



/*********************************************************************
 * globle variable don't need save
 *
 */
uint8 Button_Num = 0;

enum POWER_ONOFF {  PowerOff =0,  PowerOn};   //添加系统状态位
bool Power_OnOff = PowerOn ;

enum GAP_CONNECT {  Gapconnecting =0,  Gapconnected};   //权限模式下的连接状态
bool Gap_Connect = Gapconnecting ;

//enum OPTION_DOOR {  Auto_Close =0,  Auto_Open};   //开关门状态位
bool Option_Auto = FALSE;

enum KEY_ISR {  Key_Onetouch =0,  Key_Sectouch};   //按键触发
bool Key_Isr = Key_Onetouch;




#if 0
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData2[] =
{
  // complete name
  0x0D,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,

  0x50,   // 'P'
  0x49,   // 'I'
  0x53,   // 'S'
  0x43,   // 'C'
  0x45,   // 'E'
  0x53,   // 'S'
  0x2D,   // '-'
  0x42,   // 'B'
  0x54,   // 'T'
  0x31,   // '1'
  0x30,   // '0'
  0x30,   // '0'
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};
#endif
// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
//  0x05,   // length of this data
  0x02,
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
//  0x42,   // 'B'
//  0x4a,   // 'J'
//  0x51,   // 'C'

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( DOORPROFILE_SERV_UUID ),
  HI_UINT16( DOORPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "**PISCES-BT100**";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void bleAutoLock_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void lockStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void lockProfileChangeCB( uint8 paramID );
//static void doorLockRssiCB( int8 newRSSI )  ;
static uint8 appData_Crc(uint8  *src,uint8 crc, uint8 len);   

static void bleAutoLock_HandleKeys( uint8 shift, uint8 keys );

static void Bat_AdcAuto(void);
static uint8 SendOrGetBatvalue(uint8 value);    

//static void Bat_AdcAuto(void);
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
//static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)


void ReadMac(unsigned char * TempMacAddress)  // Len 一定是6
{
#if 0
  TempMacAddress[5]  =  XREG((BLE_REG_BASE_ADDR  + 22));  // BLE_ASI_OWNADDR
  TempMacAddress[4]  =  XREG((BLE_REG_BASE_ADDR  + 23));  // BLE_ASI_OWNADDR
  TempMacAddress[3]  =  XREG((BLE_REG_BASE_ADDR  + 24));  // BLE_ASI_OWNADDR
  TempMacAddress[2]  =  XREG((BLE_REG_BASE_ADDR  + 25));  // BLE_ASI_OWNADDR
  TempMacAddress[1]  =  XREG((BLE_REG_BASE_ADDR  + 26));  // BLE_ASI_OWNADDR
  TempMacAddress[0]  =  XREG((BLE_REG_BASE_ADDR  + 27));  // BLE_ASI_OWNADDR
#else
  TempMacAddress[5]=*(unsigned char *)(0x780E); // 直接指向指针内容
  TempMacAddress[4]=*(unsigned char *)(0x780F);
  TempMacAddress[3]=*(unsigned char *)(0x7810);
  TempMacAddress[2]=XREG(0x7811);                // define 函数直接读出数据
  TempMacAddress[1]=XREG(0x7812);
  TempMacAddress[0]=XREG(0x7813);
#endif
}


/*******************************************************************************
 * 函数名称 :   RandomNumberGenerator                                                                  
 * 描述     :   生产随机数     （利用单片机自带的硬件随机数生成器）                                                    
 *                                                                               
 * 输   入  :  void                                                                   
 * 输   出  :  生产的16位随机数
 * 返   回  :                                                            
 * 修改时间 : 2014?ê3??9è?                                                                    
 *******************************************************************************/
uint16  RandomNumberGenerator(void)
{
  uint16 RN_H = RNDH;
  uint16 RN_L = RNDL;
  ADCCON1 = 0x37 ;
  
  return (RN_H <<8 |RN_L);
}


void Encrty_Addr(unsigned char * Dest)
{
  uint8 * Src = osal_mem_alloc(16);
  ReadMac(Src);
  Src[6] = 16;
  Src[7] = 7;
  Src[8] = 19;
  Src[9] = 10;
  Src[10] = 50;
  Src[11] = 20;
  Src[12] = 2;
  Src[13] = 0x42;
  Src[14] = 0x4A;
  Src[15] = 67;
  
  LL_Encrypt(key,Src,Dest);
  
  osal_mem_free(Src);
}

void Adv_Name_Init(uint8 * Str )
{
  static uint8 scanRspData[] = {0x0D,   // length of this data
                                    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
                                    0x50,   // 'P'
                                    0x49,   // 'I'
                                    0x53,   // 'S'
                                    0x43,   // 'C'
                                    0x45,   // 'E'
                                    0x53,   // 'S'
                                    0x2D,   // '-'
                                    0x42,   // 'B'
                                    0x54,   // 'T'
                                    0x31,   // '1'
                                    0x30,   // '0'
                                    0x30,   // '0'
                                    0x05,   // length of this data
                                    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
                                    };
  uint8 *scanRspData1 = osal_mem_alloc(16);

  LL_Encrypt( key, scanRspData, scanRspData1 );

  uint8 scanRspData3[] =
  {
    scanRspData1[0],   // length of this data
    scanRspData1[1],   // GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    
    scanRspData1[2],   // 'P'
    scanRspData1[3],   // 'I'
    scanRspData1[4],   // 'S'
    scanRspData1[5],   // 'C'
    scanRspData1[6],   // 'E'
    scanRspData1[7],   // 'S'
    scanRspData1[8],   // '-'
    scanRspData1[9],   // 'B'
    scanRspData1[10],   // 'T'
    scanRspData1[11],   // '1'
    scanRspData1[12],   // '0'
    scanRspData1[13],   // '0'
    scanRspData1[14],   // length of this data
    scanRspData1[15],   // GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    
    LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
    HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
    LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
    HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

    // Tx power level
    0x02,   // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0       // 0dBm
  };
  
  osal_mem_free(scanRspData1);
  osal_memcpy(Str, scanRspData3, 23);
}

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t bleAutoLock_PeripheralCBs =
{
  lockStateNotificationCB,  // Profile State Change Callbacks
  NULL//doorLockRssiCB//NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t bleAutoLock_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static lockProfileCBs_t bleAutoLock_AutoProfileCBs =
{
  lockProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */


char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}

void Init_TestCode(void)
{
//  uint8 time_char[6] = {0};
  uint8 IC_TEST[1] = {10};
  if(TESTCODE == 0)
  {
    if(sys_config.Recode_Addr_Fir == 0 && sys_config.Recode_Addr_Sec == 0)
    {
      for(uint8 i=0;i<11;i++)
      {
        switch(i)
        {
          case 0:
            #if defined ( POWER_SAVING )
               osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // Revert to battery mode after LED off    PWRMGR_ALWAYS_ON  PWRMGR_BATTERY
            #endif
            OLED_DISPLAY_ONOFF(TRUE);
            DelayMS(300);
            feetdog();
            break;
          case 1:
            Clear_Screen(0);
            OLED_Welcom();
            DelayMS(300);
            break;
  /*          
          case 2:
          {
            uint8 uiTempDat;
            Clear_Screen(0);
            feetdog();
            
            uiTempDat=RDS1302(0x8C|0x01);			  
            time_char[5]=(uiTempDat>>4)*16+(uiTempDat&0x0f);

            uiTempDat=RDS1302(0x88|0x01);
            time_char[4]=((uiTempDat&0x1f)>>4)*16+(uiTempDat&0x0f);	

            uiTempDat=RDS1302(0x86|0x01);
            time_char[3]=((uiTempDat&0x3f)>>4)*16+(uiTempDat&0x0f);

            uiTempDat=RDS1302(0x84|0x01);
            time_char[2]=((uiTempDat&0x3f)>>4)*16+(uiTempDat&0x0f);

            uiTempDat=RDS1302(0x82|0x01);
            time_char[1]=((uiTempDat&0x7f)>>4)*16+(uiTempDat&0x0f);

            uiTempDat=RDS1302(0x80|0x01);
            time_char[0]=((uiTempDat&0x7f)>>4)*16+(uiTempDat&0x0f);
            
            LCD_P8x16Str(0, 0, (uint8*)bdAddr2Str(time_char));
            
    //          DelayMS(500);
            feetdog();
          }
            break;
          case 3:
            LCD_P8x16Str(0, 0, (uint8*)bdAddr2Str(time_char));
            DelayMS(500);
            break;
          case 4:
          {
            uint8 uiTempDat;
    //          Clear_Screen(0);
            feetdog();
            
            uiTempDat=RDS1302(0x8C|0x01);			  
            time_char[5]=(uiTempDat>>4)*16+(uiTempDat&0x0f);

            uiTempDat=RDS1302(0x88|0x01);
            time_char[4]=((uiTempDat&0x1f)>>4)*16+(uiTempDat&0x0f);	

            uiTempDat=RDS1302(0x86|0x01);
            time_char[3]=((uiTempDat&0x3f)>>4)*16+(uiTempDat&0x0f);

            uiTempDat=RDS1302(0x84|0x01);
            time_char[2]=((uiTempDat&0x3f)>>4)*16+(uiTempDat&0x0f);

            uiTempDat=RDS1302(0x82|0x01);
            time_char[1]=((uiTempDat&0x7f)>>4)*16+(uiTempDat&0x0f);

            uiTempDat=RDS1302(0x80|0x01);
            time_char[0]=((uiTempDat&0x7f)>>4)*16+(uiTempDat&0x0f);
            
            LCD_P8x16Str(0, 0, (uint8*)bdAddr2Str(time_char));
            
            DelayMS(500);
            feetdog();
          }
            break;
          case 5:
            LCD_P8x16Str(0, 0, (uint8*)bdAddr2Str(time_char));
            DelayMS(500);
            break;
          case 6:
          {
            uint8 uiTempDat;
    //          Clear_Screen(0);
            feetdog();
            
            uiTempDat=RDS1302(0x8C|0x01);			  
            time_char[5]=(uiTempDat>>4)*16+(uiTempDat&0x0f);

            uiTempDat=RDS1302(0x88|0x01);
            time_char[4]=((uiTempDat&0x1f)>>4)*16+(uiTempDat&0x0f);	

            uiTempDat=RDS1302(0x86|0x01);
            time_char[3]=((uiTempDat&0x3f)>>4)*16+(uiTempDat&0x0f);

            uiTempDat=RDS1302(0x84|0x01);
            time_char[2]=((uiTempDat&0x3f)>>4)*16+(uiTempDat&0x0f);

            uiTempDat=RDS1302(0x82|0x01);
            time_char[1]=((uiTempDat&0x7f)>>4)*16+(uiTempDat&0x0f);

            uiTempDat=RDS1302(0x80|0x01);
            time_char[0]=((uiTempDat&0x7f)>>4)*16+(uiTempDat&0x0f);
            
            LCD_P8x16Str(0, 0, (uint8*)bdAddr2Str(time_char));
            
            DelayMS(500);
            feetdog();
          }
            break;
            */
          case 2:
          case 3:  
          case 4:  
          case 5:  
          case 6:  
            break;
          case 7:
  //          LCD_P8x16Str(0, 0, (uint8*)bdAddr2Str(time_char));
  //          DelayMS(500);
            AT24C64_I2C_Write(0x1FF8, 1, IC_TEST);
            DelayMS(50);  
            feetdog();
            break;
          case 8:
            IC_TEST[0] = 0;
            AT24C64_I2C_Read(0x1FF8, 1, IC_TEST);
            if(IC_TEST[0] != 10)
            {
              HAL_SYSTEM_RESET();
            }
            break;
          case 9:
            {
              Clear_Screen(2);
              LCD_P8x16Str(0, 2, "TEST OVER !!!");
              DelayMS(500); 
              feetdog();
//              HAL_SYSTEM_RESET();
            }

            break;
          case 10:
            {
//            OLED_DISPLAY_ONOFF(FALSE);
              uint8 completKeyCode[16] = {0};    
              uint16 temp1 = 0, temp2 = 0;
              uint8 test[1] = {0};
                
              Init_GlobleVar();
              Init_SysConf();
              Init_Sys24c64();
              Init_Recode();
              
              Encrty_Addr(completKeyCode);
              osal_memcpy(sys_config.MY_KEYCODE, completKeyCode, 16);
              osal_mem_free(completKeyCode);
              HAL_DISABLE_INTERRUPTS();
              temp1 = RandomNumberGenerator();
              while( (temp1 + 714 ) > 0x1FF0 )
              {
                temp1 = RandomNumberGenerator();
              }
              sys_config.Recode_Addr_Fir = temp1;

              temp2 = RandomNumberGenerator();
              while( ((temp1<temp2+244)&&(temp2+244<temp1+714)) || ((temp1+714<temp2)&&(temp2+244>0x1FF0)) 
                    ||((temp1<244)&&(temp2<temp1)) || ((temp2<temp1+714)&&((temp2+244)>(temp1+714))) )
              {
                temp2 = RandomNumberGenerator();

              }
              sys_config.Recode_Addr_Sec = temp2;  
              
              osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
              
              AT24C64_I2C_Write(sys_config.Recode_Addr_Fir, sizeof(SYS_24C64), &sys_24c64);
              DelayMS(10);
              AT24C64_I2C_Write(sys_config.Recode_Addr_Sec, sizeof(SYS_RECODE), &sys_recode);
              AT24C64_I2C_Write(0x1FF0, 1, test);
              AT24C64_I2C_Write(0x1FFE, 1, test);
              AT24C64_I2C_Write(0x1FFF, 1, test);
              HAL_ENABLE_INTERRUPTS();
              HAL_SYSTEM_RESET();              
              
            }

          default:
            break;
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */

void BleAutoLock_Init( uint8 task_id )
{
//  uint8 data[16]={0x00};
//  uint8 data1[16]={0x00};
//  uint8 test[6] = {0x31,0x31,0x31,0x31,0x31,0x31};
//  uint8 crc[2] = {0};
//  static uint8 scanRspData2[23] = {0};
//  uint8 time_char[6] = {0};
//  char * time_scr;
  static uint8 scanRspData[] = {0x0D,   // length of this data
                                    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
                                    0x50,   // 'P'
                                    0x49,   // 'I'
                                    0x53,   // 'S'
                                    0x43,   // 'C'
                                    0x45,   // 'E'
                                    0x53,   // 'S'
                                    0x2D,   // '-'
                                    0x42,   // 'B'
                                    0x54,   // 'T'
                                    0x31,   // '1'
                                    0x30,   // '0'
                                    0x30,   // '0'
                                    0x05,   // length of this data
                                    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
                                    };
  uint8 *scanRspData1 = osal_mem_alloc(16);

  LL_Encrypt( key, scanRspData, scanRspData1 );

  uint8 scanRspData2[] =
  {
    scanRspData1[0],   // length of this data
    scanRspData1[1],   // GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    
    scanRspData1[2],   // 'P'
    scanRspData1[3],   // 'I'
    scanRspData1[4],   // 'S'
    scanRspData1[5],   // 'C'
    scanRspData1[6],   // 'E'
    scanRspData1[7],   // 'S'
    scanRspData1[8],   // '-'
    scanRspData1[9],   // 'B'
    scanRspData1[10],   // 'T'
    scanRspData1[11],   // '1'
    scanRspData1[12],   // '0'
    scanRspData1[13],   // '0'
    scanRspData1[14],   // length of this data
    scanRspData1[15],   // GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    
    LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
    HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
    LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
    HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

    // Tx power level
    0x02,   // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0       // 0dBm
  };
  
  osal_mem_free(scanRspData1);
  
  bleAutoLock_TaskID = task_id;
  
//  Adv_Name_Init(scanRspData2);
  
  
//  ssp_HW_KeyInit( key );
//  crc[1] = appData_Crc(test,crc[0],3);
  
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

//  GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, 100);  //central mode
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;//TRUEFALSE
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
	
    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData2 ), scanRspData2 );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {   

    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;//GAPBOND_PAIRING_MODE_INITIATE ;//GAPBOND_PAIRING_MODE_NO_PAIRING;GAPBOND_PAIRING_MODE_WAIT_FOR_REQ
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;//;GAPBOND_IO_CAP_DISPLAY_ONLY;GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
//  Batt_AddService( );     // Battery Service
  DoorProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( bleAutoLock_TaskID );   //  一定需要添加这个， 否则按键不起作用


//#ifdef wdog  
  //看门狗初始化
  watchdog_init();
//#endif

  ADCInit();
  
  Adc_Stop();

  Motor_Init();
  
//  Motor_Foreward();
  
//  P2INP &= ~(BV(6));
  KeyLock_OutMode();

  Ds1302_Init();
  DS1302_InitConf();
  DS1302_InitData();
  Buzzer_Init();
  OLED_Init();

//  OLED_DISPLAY_ONOFF(TRUE);
  
  Init_TestCode();
//   DelayMS(500);
  
  
#if 1  
  {
    uint8 res[1] ={0};
    
    AT24C64_I2C_Read(0x1FF0, 1, res); 
    
    if(res[0] != 0xAA )
    {
//      SendOrGetBatvalue(0);
      Bat_AdcAuto();
//      Battery_Auto();
    }
    
  }
#endif
  


//  OLED_Welcom();
//  Motor_Foreward();
  // Register callback with SimpleGATTprofile
  VOID AutoProfile_RegisterAppCBs( &bleAutoLock_AutoProfileCBs );
  
  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);//HCI_EXT_TX_POWER_MINUS_23_DBM,HCI_EXT_TX_POWER_0_DBM

//turn on overlapped processing
  HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_DISABLE);
  HCI_EXT_OverlappedProcessingCmd(HCI_EXT_ENABLE_OVERLAPPED_PROCESSING);
  // Set
//  HCI_EXT_SetTxPowerCmd
  // Setup a delayed profile startup
  osal_set_event( bleAutoLock_TaskID, DOOR_START_DEVICE_EVT );

  #if defined ( POWER_SAVING )
     osal_pwrmgr_device( PWRMGR_BATTERY ); // Revert to battery mode after LED off    PWRMGR_ALWAYS_ON  PWRMGR_BATTERY
  #endif

}




/********************************************************************
*   电池检测
*   时间: 2015-07-28
*   作者：林玉阑
*********************************************************************
* For a coin cell battery 3.0v = 100%.  The minimum operating
   * voltage of the CC2541 is 2.0v so 2.0v = 0%.
   *
   * To convert a voltage to an ADC value use: 12bite
   *
   *   (v/3)/1.25 * 2048 = adc
   *
   * 3.0v = 1638 ADC
   * 2.0v = 1092 ADC
   * 2.1v = 1147 ADC
   * We need to map ADC values from 409-273 to 100%-0%.
   *
   * Normalize the ADC values to zero:
   *
   * 1637 - 1092 = 545
   *
   * And convert ADC range to percentage range:
   *
   * percent/adc = 100/545 = 20/109
   *
   * Resulting in the final equation, with round:
   *
   * percent = ((adc - 1092) * 20) / 109
********************************************************************/
// value
// 0: get
// 1: send 
static uint8 SendOrGetBatvalue(uint8 value)    
{
  uint16 BatValue0,BatValue1,BatValue2 ;
  uint16 temp;
  uint8 res[3] = {0};
//    float temp1 = 0;
//  uint8 responseData[8] = {0x00};
    uint8 * responseData = osal_mem_alloc(16);

//  if( value < 3  )
  {
      HAL_DISABLE_INTERRUPTS();
      AT24C64_I2C_Read(0x1FFE, 1, &res[0]);  // +
      AT24C64_I2C_Read(0x1FFF, 1, &res[1]);  // -
      AT24C64_I2C_Read(0x1FF0, 1, &res[2]);
      HAL_ENABLE_INTERRUPTS();
  }

  BatValue0    = BatADMeasure();
  BatValue1    = BatADMeasure();
  BatValue2    = BatADMeasure(); 

  global.batteryADC = (uint16) ( BatValue0 < BatValue1 ?
 ((BatValue1 < BatValue2) ? BatValue1 : (BatValue0 < BatValue2 ? BatValue2 : BatValue0)) :
 ((BatValue1 > BatValue2) ? BatValue1 : (BatValue0 > BatValue2 ? BatValue2 : BatValue0)));
 

    
#if 1      

#else
//      responseData[3] = global.batteryADC;         
      responseData[3] = (global.batteryADC)/1000;
      responseData[4] = (global.batteryADC)%1000/100;
      responseData[5] = (global.batteryADC)%1000%100/10;
      responseData[6] = (global.batteryADC)%10;
//#else
//      responseData[3] = res[0];
//      responseData[4] = res[1]; 
//      responseData[5] = res[2];
#endif


  if(res[0])
  {
    global.batteryADC = global.batteryADC - res[0];//1623 - 20
  }
  else if(res[1])
  {
    global.batteryADC = global.batteryADC + res[1];//1623 - 20
  }
  if( global.batteryADC > 1450 )
  {
    temp =(uint16) global.batteryADC - 1450; 
    global.batteryPer = (uint8)( temp*100/443 ) ;
    
  }
  else
  {
    global.batteryPer = 0;
  }
  

  if( (gapProfileState == GAPROLE_CONNECTED) && (value == 1)) 
//  if( value == 1 )
  {
    responseData[0] = 0x31;
    
    if((global.batteryPer /10) >= 10)
    {
      responseData[1] = 0x01;
      responseData[2] = 0x00;
    }
    else
    {
      responseData[1] = 0x00;
      responseData[2] = ((uint8)global.batteryPer /10 *16) + ((uint8)global.batteryPer %10);
    }
#if 1
      DoorProfile_SetParameter( DOORPROFILE_CHAR2, 16, responseData );
#else
      DoorProfile_SetParameter( DOORPROFILE_CHAR2, 7, responseData );
#endif
  }
  osal_mem_free(responseData);
  return 100;//global.batteryPer;
}

/*********************************************************************
 * @fn      BleDoorLock_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 BleAutoLock_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( bleAutoLock_TaskID )) != NULL )
    {
      bleAutoLock_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & DOOR_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &bleAutoLock_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &bleAutoLock_BondMgrCBs );

    // Set timer for first periodic event
    osal_set_event( bleAutoLock_TaskID, DOOR_POWERONOFF_EVT );

    return ( events ^ DOOR_START_DEVICE_EVT );
  }

  
//作为系统实时性任务检测
  if ( events & DOOR_PERIODIC_EVT )
  {
    if( DOOR_PERIODIC_EVT_PERIOD )
    {
      // Perform periodic application task
      
      osal_start_timerEx( bleAutoLock_TaskID, DOOR_PERIODIC_EVT, DOOR_PERIODIC_EVT_PERIOD );
    }
    performPeriodicTask();
    
    return (events ^ DOOR_PERIODIC_EVT);
  }

//  开机 待机任务
  if( events & DOOR_POWERONOFF_EVT)
  {
      Power_OnOff = !Power_OnOff; 

      
      if( Power_OnOff == PowerOn )					// 唤醒开机
      {
        
        #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // 唤醒
        #endif
//        watchdog_init();
        Write_Command(0xAF); //Set Display On 
        OLED_INIT_SHOW();
//        OLED_DISPLAY_ONOFF(TRUE);
        osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config );
#if 1
        if( global.MY_KEY_CODE )
        {
          HAL_DISABLE_INTERRUPTS();
    //      AT24C64_I2C_Read(0x0000 , 1, res);
          AT24C64_I2C_Read(sys_config.Recode_Addr_Fir, sizeof(SYS_24C64), (uint8*)&sys_24c64);
          
          AT24C64_I2C_Read(sys_config.Recode_Addr_Sec, sizeof(SYS_RECODE), &sys_recode);
          HAL_ENABLE_INTERRUPTS(); 
        }
#endif
        DsGet_Time();

        
        osal_set_event( bleAutoLock_TaskID, DOOR_PERIODIC_EVT );    // 主循环

        osal_start_timerEx(bleAutoLock_TaskID, DOOR_ONCEPSWD_EVT,100);				//  唤醒一次执行一次一次性密码时间比较
        
        osal_set_event( bleAutoLock_TaskID, DOOR_BATTERY_EVT );
        
        osal_start_timerEx(bleAutoLock_TaskID, DOOR_POWERONOFF_EVT, 10000);		//  15秒无操作进入睡眠

        
      }
#if 1
      else											// 时间到关机
      {
        uint8 initial_advertising_enable;

        initial_advertising_enable = FALSE;
        
        // 断开所有连接
        GAPRole_TerminateConnection();

         // 设置蓝牙广播状态
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );  

//        GAP_UpdateAdvertisingData( bleAutoLock_TaskID, FALSE, sizeof(attDeviceName), attDeviceName );
        Key_Isr = Key_Onetouch;
//        ADCCON1 = 0x3F ;
        OLED_DISPLAY_ONOFF(FALSE);
        
        if(global.MY_KEY_CODE )
        {
          global.menu_Num = 0;
          global.mainMenuUpDown = 2;
          global.longButtonFlag = FALSE;
          global.delKeysNum = 0;
          global.delPhoneNum= 1;
          global.openDoorTurn = 0;
          global.MenuDelay = 0;
          global.Choice_Flag = 0;
          osal_memcpyz(global.keyRecode_Temp , 0xBB, 8);
          global.authoryMode = 0;
          global.AddPhoneFlag = 0;
        }
        else
        {
          Init_GlobleVar();
        }
        Motor_Stop();

        KeyLock_OutMode();
        Ds1302_Init();
        if( global.MY_KEY_CODE  )
        {
          if(global.sys_UpdataStatus || global.app_DownChange == 0x01)
          {
            HAL_DISABLE_INTERRUPTS();							//  进入睡眠后处理被更新的内容
            osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
            HAL_ENABLE_INTERRUPTS();
          }
          
          if(global.sys_UpdataStatus ||global.app_DownChange == 0x02)
          {
            HAL_DISABLE_INTERRUPTS();							//  进入睡眠后处理被更新的内容
            AT24C64_I2C_Write(sys_config.Recode_Addr_Fir, sizeof(SYS_24C64), (uint8*)&sys_24c64);  
            DelayMS(10);
            AT24C64_I2C_Write(sys_config.Recode_Addr_Sec, sizeof(SYS_RECODE), &sys_recode);
            HAL_ENABLE_INTERRUPTS();
          }
        }

        osal_stop_timerEx( bleAutoLock_TaskID, DOOR_PHONE_EVT);                 //  手机
        osal_stop_timerEx( bleAutoLock_TaskID, DOOR_ENCRYPT_EVT);               //  秘钥检测任务
        osal_stop_timerEx( bleAutoLock_TaskID, DOOR_ADMININ_EVT);               //  进入退出管理员
        osal_stop_timerEx( bleAutoLock_TaskID, DOOR_ADMIN_PRO_EVT);             //  管理员操作
        osal_stop_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT);                //  超时
        osal_stop_timerEx( bleAutoLock_TaskID, DOOR_PSWD_OPEN_EVT);             //  密码配对
        osal_stop_timerEx( bleAutoLock_TaskID, DOOR_ONCEPSWD_EVT);              //  一次性密码
        osal_stop_timerEx( bleAutoLock_TaskID, DOOR_OPENDOOR_EVT);              //  开门
        osal_stop_timerEx(bleAutoLock_TaskID, DOOR_BATTERY_EVT);                //  电池检测
        osal_stop_timerEx(bleAutoLock_TaskID,DOOR_PERIODIC_EVT);                //  主任务
        osal_stop_timerEx(bleAutoLock_TaskID, DOOR_POWERONOFF_EVT);		//  15秒无操作进入睡眠
        #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_BATTERY ); // 进入低功耗
        #endif
        
      }
#endif
    return ( events ^ DOOR_POWERONOFF_EVT );
  }
 
  // 按键进入管理员操作任务
  if ( events & DOOR_ADMININ_EVT)
  {
    static uint8 scanRspData2[23] = {0};
    osal_stop_timerEx(bleAutoLock_TaskID, DOOR_ADMININ_EVT);
    osal_stop_timerEx(bleAutoLock_TaskID, DOOR_ADMIN_PRO_EVT);
    global.longButtonFlag = !global.longButtonFlag;
    Button_Num = 0;
    global.menu_Index = 0;
    global.menu_Num = 0;
    global.mainMenuUpDown = 2;
    if(global.longButtonFlag)
    {
      uint8 initial_advertising_enable;

      initial_advertising_enable = FALSE;
      
      // 断开所有连接
      GAPRole_TerminateConnection();

       // 设置蓝牙广播状态
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );  
      
      OLED_INPUTADMIN_PSWD();//请输入管理员密码
      
      osal_memcpyz(global.MvInPswd, 0xFF, 15);
    }
    else
    {
      uint8 initial_advertising_enable;

      initial_advertising_enable = TRUE;
      
      // 断开所有连接
      GAPRole_TerminateConnection();

       // 设置蓝牙广播状态
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );  
  
      Adv_Name_Init(scanRspData2);
      
      GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData2 ), scanRspData2 );
      
      osal_start_timerEx( bleAutoLock_TaskID, DOOR_PERIODIC_EVT, DOOR_PERIODIC_EVT_PERIOD );    // 退出按键管理员模式时候 开启主循环
      
      OLED_OUTADMIN_MODE();//退出管理员模式
      
      osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 2000);
    }

    return (events ^ DOOR_ADMININ_EVT);
  }

// 进入管理员模式后层级处理
  if ( events & DOOR_ADMIN_PRO_EVT)
  {
    static uint8 scanRspData2[23] = {0};
    global.menu_Index = 0;
    osal_stop_timerEx(bleAutoLock_TaskID, DOOR_ADMIN_PRO_EVT);

    switch( global.menu_Num )
    {
      
      case 0:			//输入管理员密码
        if(osal_memcmpn(sys_config.Admin_Pswd, global.MvInPswd, 8))
        {
          OLED_INADMIN_MODE();
          OLED_MAINMENU_NUM(global.mainMenuUpDown);
          global.menu_Num = 1;
        }
        else
        {
               //密码错误重新输入
          global.menu_Num = 0;     
          OLED_INPUT_FALSE();
          global.safety_Flag ++;
          if(global.safety_Flag >= 6)
          {
            DsGet_Time();
            global.time_Recode = Ds1302_ConverUTCSecs(&systime);
            osal_set_event(bleAutoLock_TaskID, DOOR_POWERONOFF_EVT);
          }
        }

        break;
      case 1:			//选项
      {
        switch(global.mainMenuUpDown)
        {
          case 2:
            {
              if( sys_24c64.Phone_Num >= 10  && sys_24c64.Phone_Num!= 0xBB )  // 用户已满
              {
                OLED_USER_OVER();
                // 2 s 后返回
                osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 1000);
              }
              else
              {
                uint8 initial_advertising_enable = TRUE;

                global.AddPhoneFlag = 1;
                     
                // 断开所有连接
                GAPRole_TerminateConnection();
                // 设置蓝牙广播状态
                GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );

                Adv_Name_Init(scanRspData2);
      
                GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData2 ), scanRspData2 );
                
                OLED_INADDPHONE_MODE();
                OLED_ADDPHONE_CANCEL();
                global.menu_Num = 2;
  //              osal_start_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT, 4000);
              }
            }
            break;
          case 3:
            {
              global.menu_Num = 3;
              OLED_ONENEWADMIN_PSWD();
            }
            break;
          case 4:
            {
              if( sys_24c64.Keys_num >= 10 && sys_24c64.Keys_num != 0xBB )  // 用户已满
              {
                OLED_USER_OVER();
                // 2 s 后返回
                osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 1000);
              }
              else
              {
                global.menu_Num = 4;
                OLED_ONENEWGENERAL_PSWD();
              }
            }
            break;
          case 5:
            {
              uint8 i = 0;
              if( sys_24c64.Phone_Num == 0xBB )   // 未添加用户
              {
                OLED_USER_EMPTY();
                osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 1000);
              }
              else
              {
                 global.menu_Num = 5;
                 for(i=0;i<10;i++)
                 {
                    if(sys_config.Phone_User[i][0] != 0xFF)
                    {
                       global.delPhoneNum= i;
                       OLED_DELPHONE_NUM(i);
                       break;
                    }
                 }
                 
              }
              
            }
            break;
          case 6:
            {
              uint8 i = 0;
              if( sys_24c64.Keys_num == 0xBB )   // 未添加用户
              {
                OLED_USER_EMPTY();
                osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 1000);
              }
              else
              {
                global.menu_Num = 6;
                for(i=0;i<10;i++)
                {
                  if(sys_24c64.General_Pswd[i][0] != 0xBB)
                  {
                     global.delKeysNum = i;
                     OLED_DELKEYS_NUM(i);
                     break;
                  }
                }
                
              }
            }
            break;
          case 7:
            {
              OLED_CHINA_ENGLISH();
              global.menu_Num = 8;
            }
            break;
          default:
            break;
        }
        
      }
        break;
      case 2:       // 添加手机用户
        {
          uint8 *replyUpdata = osal_mem_alloc(16);
          uint8 crc = 0, i = 0;
          uint8 initial_advertising_enable;
          
          if(gapProfileState == GAPROLE_CONNECTED)
          {
            while(sys_config.Phone_User[i][0] != 0xFF)
            {
              i++;
            }

            if( i == 0 )
            {
                global.appRecode_Temp[6] |= 0x11;                                                //管理员权限
              global.sys_UpdataStatus |= 0x80;
            }
            else
            {
                global.appRecode_Temp[6]  |= 0x21;                                                //普通权限
                global.sys_UpdataStatus |= 0x41;
            }
            sys_config.Phone_User[i][0] = i;			//把序号赋值到列表信息
            osal_memcpy(&sys_config.Phone_User[i][1], global.appRecode_Temp, 7);
            crc = appData_Crc(&global.appRecode_Temp[0], crc, 6);
            global.MenuDelay = 1;
            OLED_ADDPHONE_SUCC(i);
            // 2 s 后返回
            osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 2000);
            if(sys_24c64.Phone_Num == 0xBB)
            {
              sys_24c64.Phone_Num = 1;
            }
            else
            {
              sys_24c64.Phone_Num ++;
            }

            

            replyUpdata[0] = 0x2F;
            replyUpdata[1] = i;
            replyUpdata[2] = global.appRecode_Temp[6];  //权限位
            replyUpdata[3] = crc;
              DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
            osal_memcpyz(global.appRecode_Temp, 0, 7);
            global.menu_Num = 1;
          }
          else
          {
          
          }
          initial_advertising_enable = FALSE;
          
          // 断开所有连接
          GAPRole_TerminateConnection();

           // 设置蓝牙广播状态
          GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
          osal_mem_free(replyUpdata);
          global.AddPhoneFlag = 0;
          
        }
        break;
      case 3:   //  修改管理员密码
        {
          if( osal_memcmpn(global.MvInPswd, global.keyRecode_Temp, 8) )
          {
            osal_memcpy(sys_config.Admin_Pswd, global.keyRecode_Temp, 8);
            OLED_NEWADMIN_SUCC();
            global.sys_UpdataStatus |= 0x41;     //修改管理员密码 0x01
            //  2 秒后返回
            osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 1000);
          }
          else
          {
            OLDE_NEWADMIN_FAIL();
            //  2 秒后返回
            osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 1000);
          }
          global.MenuDelay = 1;
          global.menu_Num = 1;
          osal_memcpyz(global.keyRecode_Temp, 0xBB, 8);
        }
        break;
       case 4:  // 添加按键
        {
          uint8 i = 0;
          if( osal_memcmpn(global.MvInPswd, global.keyRecode_Temp, 8) )
          {
            // 存在就显示下一个直到找到为空的用户xx退出while循环
            while(sys_24c64.General_Pswd[i][0] != 0xBB)
            {
              i++;
            }
            sys_24c64.General_Pswd[i][0] = i ;
            osal_memcpy(&sys_24c64.General_Pswd[i][1], global.keyRecode_Temp, 8);
            OLED_ADDGENERAL_SUCC(i );
            if(sys_24c64.Keys_num == 0xBB)
            {
              sys_24c64.Keys_num = 1;
            }
            else
            {
              sys_24c64.Keys_num ++;
            }
            global.sys_UpdataStatus |= 0x44;     //多次添加手机且未上传到管理员手机时，只加一次 0x02
            //  2 秒后返回
            osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 1000);
          }
          else
          {
            
            OLED_ADDGENERAL_FAIL();
            //  2 秒后返回
            osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 1000);
          }
          global.delPhoneNum = 1;
          global.MenuDelay = 1;
          global.menu_Num = 1;
          osal_memcpyz(global.keyRecode_Temp, 0xBB, 8);
        }
        break;
      case 5:     // 删除手机
        {
          sys_config.Phone_User[global.delPhoneNum][0] = 0xFF;
          osal_memcpyz(&sys_config.Phone_User[global.delPhoneNum][1], 0, 7);
          sys_24c64.Phone_Num --;
          osal_memcpy(&global.Del_Phone_User[global.Del_Phone_Num][0], &sys_config.Phone_User[global.delPhoneNum][1], 6);
          global.Del_Phone_Num ++;
          if( global.Del_Phone_Num >= 9)
          {
            global.Del_Phone_Num = 0;
          }
          if(sys_24c64.Phone_Num == 0)
          {
            sys_24c64.Phone_Num = 0xBB;
          }
          global.sys_UpdataStatus |= 0x42;     //多次添加手机且未上传到管理员手机时，只加一次 0x02
          osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
          global.MenuDelay = 1;
          global.menu_Num = 1;
          OLED_DELPHONE_SUCC();
          //  2 秒后返回
          osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 1000);
        }
        
        break;
      case 6:   // 删除按键
        {         
          osal_memcpyz(&sys_24c64.General_Pswd[global.delKeysNum][0], 0xBB, 9);
          sys_24c64.Keys_num--;
          if(sys_24c64.Keys_num == 0)
          {
            sys_24c64.Keys_num = 0xBB;
          }
          global.sys_UpdataStatus |= 0x44;     //多次添加手机且未上传到管理员手机时，只加一次 0x02
//          global.delKeysNum = 0;
          global.MenuDelay = 1;
          global.menu_Num = 1;
          OLED_DELKEYS_SUCC();
          //  2 秒后返回
          osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 1000);
        }
        
        break;
      case 7:
        {

        }
        break;
      case 8:
        {

        }
        break;
      default:
        
        break;
    }
    if( global.menu_Num > 1)
    {
        if(global.AddPhoneFlag == 1)
        {
          osal_start_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT, 6000);    //       4秒无操作超时返回主菜单
        }
        else
        {
          osal_start_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT, 4000);    //       4秒无操作超时返回主菜单
        }
    }
    global.menu_Index = 0;
    
    osal_memcpyz(global.MvInPswd, 0xFF, 15);
    
    return (events ^ DOOR_ADMIN_PRO_EVT);
  }

 // 进入手机模式任务
  if ( events & DOOR_PHONE_EVT)
  {
    static uint8 replyUpdata[] = {0x02, 0x53, 0x4A,0x00};
    switch(global.authoryMode)
    {
      case 1:
        if( gapProfileState == GAPROLE_CONNECTED )
        { 
          if(global.sys_UpdataStatus && !global.longButtonFlag && ((global.sys_UpdataStatus&0x80)!= 0x80 ))		// 若是第一次设置管理员手机，更新标志位为0xFF ，要上传信息不能再下位机管理员模式情况下
          {
 	         global.sys_UpdataStatus |= 0x40;
            global.appUpdata_CRC = appData_Crc(replyUpdata, global.appUpdata_CRC, 3);
            DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,replyUpdata);		//     通知要上传用户信息
          }
//          osal_start_timerEx(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT, 10000);		//  10秒无操作进入睡眠
        }
      break;
      case 2:

      break;
      default:
      break;

    }
    
    
    return (events ^ DOOR_PHONE_EVT);
  }
// 按键密码开门
  if ( events & DOOR_PSWD_OPEN_EVT)
  {                            
//    static uint8 openDoor[] = {0x1F, 0x4B, 0x4F};

    osal_stop_timerEx(bleAutoLock_TaskID, DOOR_PSWD_OPEN_EVT);
    #if 0
    for(uint8 i=0;i<100;i++)                                                         
    {
      if(osal_memcmpn(&sys_24c64.General_Pswd[i][1], global.MvInPswd, 6))      //  普通密码
      {
//        global.genPswdFlag = 0;							 // 状态位清零
        sys_24c64.open_DoorRec[1] = 1;
        sys_24c64.open_DoorRec[2] = i;
        osal_memcpy(global.MvInPswd, 0xFF, 10);                                     // 清空临时 10位虚位密码     
        osal_set_event(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT);
        DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,openDoor);		         // 通知APP  门已开
        break;
      }
      else
      {
        if(i == 99)                                                              //防乱按
        {
          global.safety_Flag ++;
          if(global.safety_Flag>=6)
          {
            DsGet_Time();
            global.time_Recode = Ds1302_ConverUTCSecs(&systime);
            osal_set_event(bleAutoLock_TaskID, DOOR_POWERONOFF_EVT);
          }
        }
      }
    }
    #endif
   if( global.MY_KEY_CODE  )
   {
      for(uint8 i=0;i<10;i++)                                                         
      {
        if(osal_memcmpn(&sys_24c64.Once_Single[i][1], global.MvInPswd, 8))   // 一次性密码 单次
        {
          Option_Auto = TRUE;//Auto_Open;
          KeyLock_InMode();
          sys_recode.open_DoorRec[1] = 4;
          sys_recode.open_DoorRec[2] = i;
          sys_24c64.Once_Single[i][0] = 0x00;
          osal_memcpyz(&sys_24c64.Once_Single[i][1], 0xBB, 16);                             // 同时删除一次性密码
          osal_set_event(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT);

          break;
        }
        else if(osal_memcmpn(&sys_24c64.General_Pswd[i][1], global.MvInPswd, 8))      //  普通密码
        {
          Option_Auto = TRUE;//Auto_Open;
          KeyLock_InMode();
          sys_recode.open_DoorRec[1] = 1;
          sys_recode.open_DoorRec[2] = i;
          osal_set_event(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT);

          break;
        }
        
        else if(osal_memcmpn(&sys_24c64.Once_OneDay[i][1], global.MvInPswd, 8))   // 一次性密码 一天
        {
          Option_Auto = TRUE;//Auto_Open;
          KeyLock_InMode();
          sys_recode.open_DoorRec[1] = 5;
          sys_recode.open_DoorRec[2] = i;

          osal_set_event(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT);

          break;
        }
        else if(osal_memcmpn(&sys_24c64.Once_OneWeek[i][1], global.MvInPswd, 8))   // 一次性密码 一星期
        {
          Option_Auto = TRUE;//Auto_Open;
          KeyLock_InMode();
          sys_recode.open_DoorRec[1] = 6;
          sys_recode.open_DoorRec[2] = i;
          osal_set_event(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT);

          break;
        }
        else if(osal_memcmpn(&sys_24c64.Once_AMonth[i][1], global.MvInPswd, 8))   // 一次性密码 一个月
        {
          Option_Auto = TRUE;//Auto_Open;
          KeyLock_InMode();
          sys_recode.open_DoorRec[1] = 7;
          sys_recode.open_DoorRec[2] = i;
          osal_set_event(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT);

          break;
        }
        else
        {
          if(i == 9)                                                              //防乱按
          {
            global.safety_Flag ++;
            if(global.safety_Flag>=6)
            {
            DsGet_Time();
              global.time_Recode = Ds1302_ConverUTCSecs(&systime);
              osal_set_event(bleAutoLock_TaskID, DOOR_POWERONOFF_EVT);
            }
            OLED_INPUT_FALSE();
          }
        }
        feetdog();
      }
    }
    else
    {
      OLED_INPUT_FALSE();
    }
    osal_memcpyz(global.MvInPswd, 0xFF, 15);                                     // 清空临时 15 位虚位密码
    return (events ^ DOOR_PSWD_OPEN_EVT);
  }
  

  // 一次性密码管理任务
  if ( events & DOOR_ONCEPSWD_EVT)                                              // 唤醒一次执行一次
  {
//    osal_ConvertUTCTime(&UTCTimeSys, osal_getClock());
//    uint32 currenTime = osal_ConvertUTCSecs(&UTCTimeSys);
    DsGet_Time();
    uint32 currenTime = Ds1302_ConverUTCSecs(&systime);
    
    for(uint8 i=0;i<10;i++)
    {
   
      if(currenTime >=( sys_24c64.Once_DayLimit[i][0]+86400))
      {
        sys_24c64.Once_DayLimit[i][0] = 0x00;
        osal_memcpyz(&sys_24c64.Once_OneDay[i][0], 0xBB, 9);  			//若超过一天则自动清除一天权限的一次性密码
      }
      if(currenTime >=( sys_24c64.Once_WeekLimit[i][0]+604800))
      {
        sys_24c64.Once_WeekLimit[i][0] = 0x00;
        osal_memcpyz(&sys_24c64.Once_OneWeek[i][0], 0xBB, 9);  		        //若超过一天则自动清除一星期权限的一次性密码
      }
      if(currenTime >=( sys_24c64.Once_MonthLimit[i][0]+2592000))
      {
        sys_24c64.Once_MonthLimit[i][0]= 0x00;
        osal_memcpyz(&sys_24c64.Once_AMonth[i][0], 0xBB, 9);  			//若超过一天则自动清除一个月权限的一次性密码
      }
//      feetdog();
    }
    osal_stop_timerEx(bleAutoLock_TaskID, DOOR_ONCEPSWD_EVT);
    return (events ^ DOOR_ONCEPSWD_EVT);
  }


  
  if ( events & DOOR_ENCRYPT_EVT )
  {
//    if(gapProfileState == GAPROLE_CONNECTED)
    {
      //advertising enable 
//      uint8 initial_advertising_enable = TRUE;

      // Terminate all connection
      VOID GAPRole_TerminateConnection();

       // adverty state
//      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );  

//      GAP_UpdateAdvertisingData( bleAutoLock_TaskID, FALSE, sizeof(attDeviceName), attDeviceName );
      
//      osal_start_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT, 10 ); 
    } 
    osal_stop_timerEx( bleAutoLock_TaskID, DOOR_ENCRYPT_EVT ); 

    return ( events ^ DOOR_ENCRYPT_EVT );
  }
  
  #if 1
  //恢复出厂设置
  if( events & DOOR_INITCONF_EVT )
  {
    osal_stop_timerEx(bleAutoLock_TaskID, DOOR_INITCONF_EVT);
    uint8 test[1] = {0x00};
    Buzzer_Open();
    Init_GlobleVar();
    Init_Sys24c64();
    Init_Recode();
    Init_SysConf();
//      sys_24c64.At24c64_WR = 0;
    HAL_DISABLE_INTERRUPTS(); 
    osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
    
    AT24C64_I2C_Write(sys_config.Recode_Addr_Fir, sizeof(SYS_24C64), &sys_24c64);
    DelayMS(10);
    AT24C64_I2C_Write(sys_config.Recode_Addr_Sec, sizeof(SYS_RECODE), &sys_recode);
    
    AT24C64_I2C_Write(0x1FF0, 1, test);
    AT24C64_I2C_Write(0x1FFE, 1, test);
    AT24C64_I2C_Write(0x1FFF, 1, test);
    
    HAL_ENABLE_INTERRUPTS();
    
    HAL_SYSTEM_RESET();

//    return ( events ^ DOOR_INITCONF_EVT );
  }

#endif

  // 开关门任务 
  if ( events & DOOR_OPENDOOR_EVT)
  {
//    osal_stop_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT);    //       4秒无操作超时返回主菜单
    global.openDoorTurn ++;
#if 0
    if( global.batteryNum > 0)
    {
      osal_stop_timerEx(bleAutoLock_TaskID, DOOR_BATTERY_EVT);
    }
#endif
    switch(global.openDoorTurn)
    {
    case 1:
      
//      HalKeyConfig( HAL_KEY_INTERRUPT_ENABLE, OnBoard_KeyCallback);//HAL_KEY_INTERRUPT_DISABLE
      Motor_Foreward();
      OLED_OPENING_LOCK();
      Buzzer_Open();
      feetdog();
      osal_stop_timerEx(bleAutoLock_TaskID, DOOR_ADMININ_EVT);
      osal_stop_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT);    //       4秒无操作超时返回主菜单
      osal_stop_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT);    //       
      osal_start_timerEx(bleAutoLock_TaskID, DOOR_POWERONOFF_EVT, 10000);
      osal_stop_timerEx(bleAutoLock_TaskID, DOOR_PERIODIC_EVT);
      osal_start_timerEx(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT, 300);
      break;
    case 2:
      Buzzer_Close();
      feetdog();
      osal_start_timerEx(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT, 100);
      break;
    case 3:
      Buzzer_Open();
      osal_start_timerEx(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT, 300);
      break;
    case 4:
      Buzzer_Close();
      feetdog();
      osal_start_timerEx(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT, 100);
      break;
    case 5:
      Buzzer_Open();
      osal_start_timerEx(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT, 100);
      break;
    case 6:
      Buzzer_Close();
      osal_start_timerEx(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT, 100);
      break;
    case 7:
      Buzzer_Open();
      osal_start_timerEx(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT, 100);
      break;
    case 8:
      Buzzer_Close();
//      global.openDoorTurn = 0;
      DsGet_Time();

      global.sys_UpdataStatus |= 0x50;
      
      if(sys_recode.Open_Count >=16)
      {
        sys_recode.Open_Count = 15;
        sys_recode.open_DoorRec[0] = sys_recode.Open_Count;
        Array_Stack();
      }
      
      sys_recode.Open_Recode[sys_recode.open_DoorRec[0]-1][0] = sys_recode.open_DoorRec[0];
      sys_recode.Open_Recode[sys_recode.open_DoorRec[0]-1][1] = sys_recode.open_DoorRec[1];
      sys_recode.Open_Recode[sys_recode.open_DoorRec[0]-1][2] = sys_recode.open_DoorRec[2];
      
      sys_recode.Open_Recode[sys_recode.open_DoorRec[0]-1][3] = systime.cYear;
      sys_recode.Open_Recode[sys_recode.open_DoorRec[0]-1][4] = systime.cMon;
      sys_recode.Open_Recode[sys_recode.open_DoorRec[0]-1][5] = systime.cDay;
      sys_recode.Open_Recode[sys_recode.open_DoorRec[0]-1][6] = systime.cHour;
      sys_recode.Open_Recode[sys_recode.open_DoorRec[0]-1][7] = systime.cMin;
      sys_recode.Open_Recode[sys_recode.open_DoorRec[0]-1][8] = systime.cSec;
      
      sys_recode.Open_Count ++;
      sys_recode.open_DoorRec[0] = sys_recode.Open_Count;
//      if(sys_recode.open_DoorRec[0] >=21)
//      {
//        sys_recode.open_DoorRec[0] = 1;
//      }
      global.safety_Flag = 0;
      osal_stop_timerEx(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT);
      osal_set_event(bleAutoLock_TaskID, DOOR_PERIODIC_EVT);
      osal_start_timerEx(bleAutoLock_TaskID, DOOR_POWERONOFF_EVT, 5000);       //关门后7秒内无操作进入睡眠状态
      #if 0
      if( global.batteryNum > 0)
      {
        osal_set_event(bleAutoLock_TaskID, DOOR_BATTERY_EVT);
      }
      #endif
      break;
    default:
      break;
      
    }

      return (events ^ DOOR_OPENDOOR_EVT);
  }
    



  
  //keys option timeout
  if( events & DOOR_OUTTIME_EVT )
  {
    uint8 initial_advertising_enable = FALSE;
    osal_stop_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT);
    if( !global.longButtonFlag && global.menu_Num == 0)
    {
      OLED_Welcom();
    }
    else
    {
      if( global.menu_Num == 2 )
      {
        // 断开所有连接
        GAPRole_TerminateConnection();

         // 设置蓝牙广播状态
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable ); 
      }
      global.menu_Num = 1;
        
      //操作超时返回主菜单
      osal_memcpyz(global.MvInPswd, 0xFF, 15);
      OLED_INADMIN_MODE();
      OLED_MAINMENU_NUM( global.mainMenuUpDown );
    }
    
    global.menu_Index = 0;
    
    return ( events ^ DOOR_OUTTIME_EVT );
  }

  
  if( events & DOOR_BATTERY_EVT )
  {
    uint8 temp = 0;
    static uint8 scanRspData2[23] = {0};
    global.batteryNum ++ ;
    if( global.batteryNum > 3 && global.batteryNum < 10)
    {
      temp = osal_array(global.batteryPower);
      Adc_Stop();
      if(temp <= 5)
      {
//        global.Battery_Empty = 1;
//        osal_stop_timerEx(bleDoorLock_TaskID, DOOR_PERIODIC_EVT);
        
        if( global.batteryNum %2 == 0)
        {
          Buzzer_Open();
          OLED_BATTERY_EMPTY();
        }
        else
        {
          Buzzer_Close();
        }
        osal_start_timerEx(bleAutoLock_TaskID, DOOR_BATTERY_EVT,50);
      }
      else
      {
        uint8 initial_advertising_enable = TRUE;
         
        OLED_DISPLAY_ONOFF(TRUE);
        // 断开所有连接
        GAPRole_TerminateConnection();
         // 设置蓝牙广播状态
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable ); 
        
        Adv_Name_Init(scanRspData2);
      
        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData2 ), scanRspData2 );
//        GAP_UpdateAdvertisingData( bleAutoLock_TaskID, FALSE, sizeof(attDeviceName), attDeviceName );
        
        global.batteryNum = 0;
        global.Battery_Empty = 0;
        osal_memcpyz(global.batteryPower , 0, 3);
        osal_stop_timerEx(bleAutoLock_TaskID, DOOR_BATTERY_EVT);
        return ( events ^ DOOR_BATTERY_EVT );
      }
      
    }
    else if( global.batteryNum <= 3 && global.batteryNum > 0)
    {
      Adc_Start();
      SendOrGetBatvalue(0);
//      OLED_INIT_SHOW();
//      if(global.batteryNum > 1)//第一次读不要
      {
        global.batteryPower[global.batteryNum-1] = global.batteryPer;
      }
      
      osal_start_timerEx(bleAutoLock_TaskID, DOOR_BATTERY_EVT,20);
      
    }
    else if(global.batteryNum >= 10)
    {
      uint8 initial_advertising_enable = TRUE;
       // 断开所有连接
      GAPRole_TerminateConnection();
       // 设置蓝牙广播状态
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable ); 
      
      Adv_Name_Init(scanRspData2);
      
      GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData2 ), scanRspData2 );
//      GAP_UpdateAdvertisingData( bleAutoLock_TaskID, FALSE, sizeof(attDeviceName), attDeviceName );
      
      global.batteryNum = 0;
//      Adc_Stop();
      osal_memcpyz(global.batteryPower , 0, 3);
      osal_stop_timerEx(bleAutoLock_TaskID, DOOR_BATTERY_EVT);
//      osal_set_event(bleAutoLock_TaskID, DOOR_PERIODIC_EVT);
      return ( events ^ DOOR_BATTERY_EVT );
    }
    
    return ( events ^ DOOR_BATTERY_EVT );
  }
  
  //  2秒后 显示  智能锁 博佳琴电子科技
  if( events & DOOR_SHOWCO_EVT )
  {
    osal_stop_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT);
    if( !global.longButtonFlag && global.menu_Num == 0)
    {
      OLED_Welcom();
    }
    else
    {
      global.MenuDelay = 0;
      global.menu_Num = 1;
      OLED_INADMIN_MODE();
      OLED_MAINMENU_NUM( global.mainMenuUpDown );
      osal_stop_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT);
      global.menu_Index = 0;
    }
    return ( events ^ DOOR_SHOWCO_EVT );
  }
  
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */

static void bleAutoLock_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
 // #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      bleAutoLock_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
 // #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}



/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void bleAutoLock_HandleKeys( uint8 shift, uint8 keys )
{
  VOID shift;
//  static uint8 Button_Num = 0;
  Key_Isr = !Key_Isr;

  if(Power_OnOff == PowerOn )
  {
    osal_start_timerEx(bleAutoLock_TaskID, DOOR_POWERONOFF_EVT, 10000);
//    osal_stop_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT);    //       4秒无操作超时返回主菜单
//    if(global.openDoorTurn == 0 && global.batteryNum == 0)
//    {
      switch( keys )
      {
        case HAL_KEY_SW_1:            // "*"
          Button_Num = 0x0A;
        break;
        case HAL_KEY_SW_2:            //  "0"
          Button_Num = 0x30;
        break;
        case HAL_KEY_SW_3:            // "#"
           Button_Num = 0x0B;
        break;
        case HAL_KEY_SW_4:            // "3"
          Button_Num = 0x33;
        break;
        case HAL_KEY_SW_5:            // "2"
          Button_Num = 0x32;
        break;
        case HAL_KEY_SW_6:           // "1" 
          Button_Num = 0x31;
        break;
        case HAL_KEY_SW_7:
  /*        
        if(Option_Auto == TRUE && Key_Motor != 0 )
        {
          Option_Auto = FALSE;
  //        Buzzer_Close();
          OLED_Welcom();
          Motor_Stop();
          KeyLock_OutMode();
          global.openDoorTurn = 0;
        }
  */       
        break;
        default:
#if 0        
        if( global.batteryNum > 0 )
        {
          osal_set_event(bleAutoLock_TaskID, DOOR_BATTERY_EVT);
        }
#endif
        break;
      }
    osal_stop_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT);    //       4秒无操作超时返回主菜单
    if(global.openDoorTurn == 0 && global.batteryNum == 0)
    {
  //    if( Button_Num && global.MenuDelay == 0)
      if(  Button_Num && !global.longButtonFlag)   //  global.MenuDelay == 0 &&
      {        
//        osal_stop_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT);    //       4秒无操作超时返回主菜单
        osal_stop_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT);
      }
      if( keys & HAL_KEY_SW_3 )
      {
        osal_start_timerEx(bleAutoLock_TaskID, DOOR_ADMININ_EVT, 2000);  // 长按
      }
      else
      {
        osal_stop_timerEx(bleAutoLock_TaskID, DOOR_ADMININ_EVT);
      }
      if( Button_Num && !Key_Isr)               // 有按键值进入
      {
        global.MvInPswd[global.menu_Index] = Button_Num;
        global.menu_Index++;
/*      
        if( global.menu_Index >= 15 )
        {
          global.menu_Index = 0;
        }
  
        if( global.batteryNum > 0 )
        {
          osal_stop_timerEx(bleAutoLock_TaskID, DOOR_BATTERY_EVT);
        }
*/        
        // 非管理员模式下不加入  # 按键，
        if(!global.longButtonFlag )                        
        {
          osal_start_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT, 4000);    //       4秒无操作超时返回主菜单 20160919
          if( global.menu_Index == 1 && Button_Num != 0x0B )
          {
            
            Clear_Screen(2);
            OLED_INPUT_UNLOCK();
          }
          if( Button_Num != 0x0B   )
          { 
            if(Button_Num == 0x0A)
            {
              global.menu_Index = 0;
              Clear_Screen(2);
              osal_memcpyz(global.MvInPswd, 0xFF, 15);
            }
            else
            {
              OLED_INHIDDEN_PSWD( global.menu_Index - 1 );
            }
            
          }
          else if( Button_Num == 0x0B  )   // # 号键进入密码比较
          {
            if(global.menu_Index <= 8)
            {
              OLED_INPUT_FALSE();
              global.menu_Index = 0;
            }
            else
            {
              global.menu_Index = 0;
  //            global.menu_Index = 0;
              osal_set_event(bleAutoLock_TaskID, DOOR_PSWD_OPEN_EVT);
            }
          }

          if(global.menu_Index >= 15)
          {
            global.menu_Index = 0;
            osal_set_event(bleAutoLock_TaskID, DOOR_PSWD_OPEN_EVT);
          }
        }
        else
        {
          if(global.MenuDelay == 0)
          {
            
            switch(global.menu_Num)
            {
            case 0:                    
              OLED_INHIDDEN_PSWD( global.menu_Index - 1 );
              if(global.MvInPswd[global.menu_Index-1] == 0x0B || global.menu_Index >=15)               //如果收到"#" 进入比较
              {
    //                global.menu_Index = 0;
                osal_set_event(bleAutoLock_TaskID, DOOR_ADMIN_PRO_EVT);
              }
              
              else if( Button_Num == 0x0A )
              {
                Clear_Screen(2);
                osal_memcpyz(global.MvInPswd, 0xFF, 15);
                global.menu_Index = 0;
              }
              break;
              
            case 1:// 进入一级管理   
              if( Button_Num == 0x30 )              //  下
              {
                global.mainMenuUpDown ++;
                if( global.mainMenuUpDown >= 8 )
                {
                  global.mainMenuUpDown = 2;
                }
                global.menu_Index = 0;
                OLED_MAINMENU_NUM( global.mainMenuUpDown );
              }
              else if( Button_Num == 0x32)          //  上
              {
                global.mainMenuUpDown --;
                if( global.mainMenuUpDown <= 1 )
                {
                  global.mainMenuUpDown = 7;
                }
                global.menu_Index = 0;
                OLED_MAINMENU_NUM( global.mainMenuUpDown );
              }
              else if( Button_Num == 0x0B)          //  #
              {
    //            global.menu_Num = global.mainMenuUpDown;   //按确定键 确定了菜单栏选项
//                osal_start_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT, 4000);    //       4秒无操作超时返回主菜单
                osal_set_event(bleAutoLock_TaskID, DOOR_ADMIN_PRO_EVT);
              }
              
              if(global.MvInPswd[0] == 0x0A && global.MvInPswd[1] == 0x0A && global.menu_Index == 3)
              {
                global.menu_Num = 7;
       //         global.menu_Index = 0;
              }
              if(global.menu_Index >= 3 && global.MvInPswd[1] != 0x0A)
              {
                global.menu_Index = 0;
              }
              break;
              
            case 2:               // 添加手机用户？
             {
              
              if( Button_Num == 0x0B && gapProfileState == GAPROLE_CONNECTED)                 // 确认添加手机用户
              {
                OLED_ADDPHONE_SUCC(sys_24c64.Phone_Num);     // 添加了用户xx
                // 2 秒后跳回 主菜单界面
                osal_set_event(bleAutoLock_TaskID, DOOR_ADMIN_PRO_EVT);
              }
              else if( Button_Num == 0x0A )                             // 取消添加
              {
                OLED_INADMIN_MODE();
                OLED_MAINMENU_NUM( global.mainMenuUpDown );
                global.menu_Num = 1;
                global.menu_Index = 0;
                osal_memcpyz(global.MvInPswd, 0xFF, 15);
              }

             }
             break;
              
            case 3:               // 修改管理员密码     输入新管理员密码
              {
                if( Button_Num == 0x0A )            // 取消
                {
                  OLED_INADMIN_MODE();
                  OLED_MAINMENU_NUM( global.mainMenuUpDown );
                  osal_memcpyz(global.keyRecode_Temp, 0xBB, 8);
                  global.menu_Num = 1;
                  global.menu_Index = 0;
                  osal_memcpyz(global.MvInPswd, 0xFF, 15);
                }
                else if( Button_Num == 0x0B )
                {
                  if( global.menu_Index >=8 && global.keyRecode_Temp[7] == 0xBB )// 输入新密码
                  {
                    OLED_SECNEWADMIN_PSWD();
                    global.menu_Index = 0;
                    osal_memcpy(global.keyRecode_Temp, global.MvInPswd, 8); // 拷贝第一次输入密码
                  }
                  else if( global.menu_Index >=8 && global.keyRecode_Temp[7] != 0xBB )// 再次输入新密码
                  {
                    osal_set_event(bleAutoLock_TaskID, DOOR_ADMIN_PRO_EVT);
                  }
                  else if( global.menu_Index < 9 )
                  {
                    global.menu_Num = 1;
                    OLED_ADDGENERAL_FAIL();
                    //  2 秒后返回
                    osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 1000);
//                    OLED_INADMIN_MODE();                    
                    osal_memcpyz(global.keyRecode_Temp , 0xBB, 8);
//                    OLED_MAINMENU_NUM( global.mainMenuUpDown );
                    global.menu_Index = 0;
                  }
                }
                else if(global.menu_Index > 8)
                {
                  global.menu_Index = 9;
                }
                else
                {
                  OLED_INDISPLAY_PSWD(global.menu_Index - 1, Button_Num - 0x30 );
                }
              }
              
              break;
              
            case 4:              // 添加按键用户  输入密码
              {
                if( Button_Num == 0x0A )            // 取消
                {
                  OLED_INADMIN_MODE();
                  OLED_MAINMENU_NUM( global.mainMenuUpDown );
                  osal_memcpyz(global.keyRecode_Temp, 0xBB, 8);
                  global.menu_Num = 1;
                  global.menu_Index = 0;
                  osal_memcpyz(global.MvInPswd, 0xFF, 15);
                }
                else if( Button_Num == 0x0B )
                {
                  if( global.menu_Index >=8 && global.keyRecode_Temp[7] == 0xBB )// 输入新密码
                  {
                    OLED_SECNEWADMIN_PSWD();
                    global.menu_Index = 0;
                    osal_memcpy(global.keyRecode_Temp, global.MvInPswd, 8); // 拷贝第一次输入密码
                  }
                  else if( global.menu_Index >=8 && global.keyRecode_Temp[7] != 0xBB )// 再次输入新密码
                  {
                    osal_set_event(bleAutoLock_TaskID, DOOR_ADMIN_PRO_EVT);
                  }
                  else if( global.menu_Index < 9 )
                  {
                    global.menu_Num = 1;
                    OLED_ADDGENERAL_FAIL();
                    //  2 秒后返回
                    osal_start_timerEx(bleAutoLock_TaskID, DOOR_SHOWCO_EVT, 1000);
//                    OLED_INADMIN_MODE();                    
                    osal_memcpyz(global.keyRecode_Temp , 0xBB, 8);
//                    OLED_MAINMENU_NUM( global.mainMenuUpDown );
                    global.menu_Index = 0;
                  }
                }
                else if(global.menu_Index > 8)
                {
                  global.menu_Index = 9;
                }
                else
                {
                  OLED_INDISPLAY_PSWD(global.menu_Index - 1, Button_Num - 0x30 );
                }
                
              }  
                
              break;
              
            case 5:              // 删除手机用户xx?
              {
                
                if( Button_Num == 0x30 )    // 下
                {
                  // 为空就显示下一个直到存在的用户xx退出while循环
                  for(uint8 i=global.delPhoneNum;i<20;i++)
                  {
                    global.delPhoneNum ++;
                    if(global.delPhoneNum > 9)
                    {
                      global.delPhoneNum = 0;
                    }
                    if( sys_config.Phone_User[global.delPhoneNum][0] != 0xFF )
                    {
                      OLED_DELPHONE_NUM(global.delPhoneNum);
                      break;
                    }
                  }
                }
                else if( Button_Num == 0x32 ) // 上
                {
                  for(uint8 i=global.delPhoneNum;i<20;i++)
                  {
                    global.delPhoneNum --;
                    if(global.delPhoneNum > 9)
                    {
                      global.delPhoneNum = 9;
                    }
                    if( sys_config.Phone_User[global.delPhoneNum][0] != 0xFF )
                    {
                      OLED_DELPHONE_NUM(global.delPhoneNum);
                      break;
                    }
                  }
                }
                else if( Button_Num == 0x0B )  // #
                {                 
                  osal_set_event(bleAutoLock_TaskID, DOOR_ADMIN_PRO_EVT);
                }
                else if( Button_Num == 0x0A )       // *
                {
                  OLED_INADMIN_MODE();
                  OLED_MAINMENU_NUM( global.mainMenuUpDown );
                  global.menu_Num = 1;
                  osal_memcpyz(global.MvInPswd, 0xFF, 15);
                }
                global.menu_Index = 0;
//                OLED_DELPHONE_NUM(global.delPhoneNum);
              }
              break;
              
            case 6:              // 删除按键用户xx?
              {
                if( Button_Num == 0x30 )    // 下
                {
                  // 为空就显示下一个直到存在的用户xx退出while循环
                  for(uint8 i=global.delKeysNum;i<20;i++)
                  {
                    global.delKeysNum ++;
                    if(global.delKeysNum > 9)
                    {
                      global.delKeysNum = 0;
                    }
                    if( sys_24c64.General_Pswd[global.delKeysNum][0] != 0xBB )
                    {
                      OLED_DELKEYS_NUM(global.delKeysNum);
                      break;
                    }
                  } 
                }
                else if( Button_Num == 0x32 ) // 上
                {

                  for(uint8 i=global.delKeysNum;i<20;i++)
                  {
                    global.delKeysNum --;
                    if(global.delKeysNum > 9)
                    {
                      global.delKeysNum = 9;
                    }
                    if( sys_24c64.General_Pswd[global.delKeysNum][0] != 0xBB )
                    {
                      OLED_DELKEYS_NUM(global.delKeysNum);
                      break;
                    }
                  } 
                }
                else if( Button_Num == 0x0B )  // #
                {                  
                  osal_set_event(bleAutoLock_TaskID, DOOR_ADMIN_PRO_EVT);
                }
                else if( Button_Num == 0x0A )       // *
                {
                  OLED_INADMIN_MODE();
                  OLED_MAINMENU_NUM( global.mainMenuUpDown );
                  global.menu_Num = 1;
                  osal_memcpyz(global.MvInPswd, 0xFF, 15);
                }
                global.menu_Index = 0;
              }
              break;
              
            case 7:          
              {

                if( Button_Num == 0x0B )
                {
                  if(global.MvInPswd[2] == 0x33 && global.MvInPswd[3] == 0x31 && global.MvInPswd[4] == 0x32)
                  {
                    global.menu_Index = 0;
                    OLED_DISPLAY_ONOFF(FALSE);
                    osal_set_event(bleAutoLock_TaskID,DOOR_INITCONF_EVT);
                  }
                }
                else if( Button_Num == 0x0A )
                {
                  global.menu_Index = 0;
                  global.menu_Num = 1;
                  OLED_INADMIN_MODE();
                  OLED_MAINMENU_NUM( global.mainMenuUpDown );
                  osal_memcpyz(global.MvInPswd, 0xFF, 15);
                }
                else if(global.menu_Index >= 6 &&  Button_Num != 0x0B)
                {
                  global.menu_Index = 0;
                  global.menu_Num = 1;
                  OLED_INADMIN_MODE();
                  OLED_MAINMENU_NUM( global.mainMenuUpDown );
                  osal_memcpyz(global.MvInPswd, 0xFF, 15);
                }
                
              }
              
              break;
            case 8:               
              {
                if( Button_Num == 0x30 )    // 下
                {
                  sys_config.Language_Choice = !sys_config.Language_Choice;
                  OLED_CHINA_ENGLISH();
                  global.Choice_Flag ++;
                }
                else if( Button_Num == 0x32 ) // 上
                {
                  sys_config.Language_Choice = !sys_config.Language_Choice;
                  OLED_CHINA_ENGLISH();
                  global.Choice_Flag ++;
                }
                else if( Button_Num == 0x0B )  // #
                {                  
                  if( sys_config.Language_Choice )
                  {
                    sys_config.ChiOrEng = 0xFF;
                  }
                  else
                  {
                    sys_config.ChiOrEng = 0;
                  }
                  global.Choice_Flag = 0;
                  OLED_INADMIN_MODE();
                  OLED_MAINMENU_NUM( global.mainMenuUpDown );
                  global.menu_Num = 1;
                  osal_memcpyz(global.MvInPswd, 0xFF, 15);
                  global.menu_Index = 0;
                }
                else if( Button_Num == 0x0A )       // *
                {
                  if(global.Choice_Flag %2 != 0)
                  {
                    sys_config.Language_Choice = !sys_config.Language_Choice;
                  }
                  global.Choice_Flag = 0;
                  OLED_INADMIN_MODE();
                  OLED_MAINMENU_NUM( global.mainMenuUpDown );
                  global.menu_Num = 1;
                  osal_memcpyz(global.MvInPswd, 0xFF, 15);
                }
                global.menu_Index = 0;
              }
              break;
              
            default:
              break;
            }
//            if( global.menu_Num > 1)
//            {
//              osal_start_timerEx(bleAutoLock_TaskID, DOOR_OUTTIME_EVT, 4000);    //       4秒无操作超时返回主菜单
//            }
          }
        }
        Button_Num = 0;
      }
    }
    else
    {
      global.menu_Index = 0;
      Button_Num = 0;
    }
  }
  else									//非唤醒状态则进入唤醒任务。状态转换
  {
    uint32 res = 0;

    global.menu_Index = 0;
    
    Button_Num = 0;

    if(global.safety_Flag >= 6)                                           // 是否被乱按
    {
        DsGet_Time();
      res = Ds1302_ConverUTCSecs(&systime);
      if(res >= global.time_Recode + 600)                                 // 10分钟标志 时间到就可以唤醒否则无作用
      {
        global.safety_Flag = 0;
        global.time_Recode = 0;
        osal_start_timerEx(bleAutoLock_TaskID, DOOR_POWERONOFF_EVT,50);
      }
      else
      {
        return;
      }
    }
    else
    {
//        global.safety_Flag = 0;
      osal_set_event(bleAutoLock_TaskID, DOOR_POWERONOFF_EVT);
    }
  }
  
}


/*********************************************************************
 * @fn      doorLockRssiCB
 *
 * @brief   show Rssi value.
 *
 * @param   newRSSI - new Rssi
 *
 * @return  none
 */
#if 0
static void doorLockRssiCB( int8 newRSSI )  
{ 
/* 
  int8 iRssi;
  char  s[10];
  float power;
  float RSI;
  
  
  
  iRssi = abs(newRSSI); 
  
  power = (iRssi-59)/(10*2.0); 
  
  RSI = pow(10, power); 
  
  sprintf(s, "%f", RSI);
  
  HalLcdWriteString(s, HAL_LCD_LINE_6);
  */
//    HalLcdWriteStringValue( "RSSI -dB:", (uint8) (-newRSSI), 10, HAL_LCD_LINE_7 );  
    
}
#endif
/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void lockStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
//  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
//  static uint8 first_conn_flag = 0;
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

      }
      break;

    case GAPROLE_ADVERTISING:
      {
        uint8 initial_advertising_enable = FALSE;
//        global.authoryMode = 0;
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        osal_stop_timerEx( bleAutoLock_TaskID, DOOR_ENCRYPT_EVT ); 
        if (Power_OnOff == PowerOff)
        {
          // 断开所有连接
          GAPRole_TerminateConnection();

          // 设置蓝牙广播状态
          GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable ); 
        }

      }
      break;

    case GAPROLE_CONNECTED:
      {       
#if 1        
//        if ( first_conn_flag == 0 && global.authoryMode==1)   //连接时候上传一次 管理员手机
//        {
//          first_conn_flag = 1;
//          Adc_Start();
//          SendOrGetBatvalue(1);  // 1: send  0:get
//          Adc_Stop();
//        }
        if(global.AddPhoneFlag == 1)
        {
          uint8 * addPhoneMode = osal_mem_alloc(16);
//          uint8 addPhoneMode[16] = {0x01, 0x4F, 0x4B, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
          addPhoneMode[0] = 0x01;
          addPhoneMode[1] = 0x4F;
          addPhoneMode[2] = 0x4B;
//          LL_Encrypt(key, addPhoneMode, addPhoneMode);
          DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, addPhoneMode);	    // 通知APP 进入添加模式
//          OLED_ADDPHONE_ENTER();
//          osal_set_event(bleAutoLock_TaskID, DOOR_ADMIN_PRO_EVT);
          osal_stop_timerEx( bleAutoLock_TaskID, DOOR_ENCRYPT_EVT);                 //
          osal_mem_free(addPhoneMode);
        }
        
        if(!global.authoryMode && global.AddPhoneFlag == 0)       // 无权限断开连接
        {
          osal_start_timerEx( bleAutoLock_TaskID, DOOR_ENCRYPT_EVT, 1000 ); 
        }
        else      // 有权限后关闭验证
        {
          osal_stop_timerEx( bleAutoLock_TaskID, DOOR_ENCRYPT_EVT);                 //
        }
        #endif
      }
    
      break; 
      
    case GAPROLE_WAITING:
      {
//        global.authoryMode = 0;

      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
          
        // Reset flag for next connection.
//        first_conn_flag = 0;

      }
      break;

    case GAPROLE_ERROR:
      {

      }
      break;

    default:
      {

      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
/*
  if(Key_Motor == 0 && Option_Auto == TRUE )
  {
    Option_Auto = FALSE;
    Motor_Stop();
    osal_set_event(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT);
    
  }
*/    
  if( Key_Motor != 0 && Option_Auto == TRUE )
  {
    Motor_Stop();
    Option_Auto = FALSE;
//        Buzzer_Close();
    OLED_Welcom();
    KeyLock_OutMode();
    global.openDoorTurn = 0;
  }
/*
  if(global.authoryMode != 0)                                                   //     APP自动模式开门
  {
//    uint8 * openDoor = osal_mem_alloc(16);
//    openDoor[0] = 0x1F;
//    openDoor[1] = 0x4B;
//    openDoor[2] = 0x4F;
//    if(global.safety_Rssi[0] <= global.safety_Rssi[1])				//     只有安全距离下才可以开门
    {
      global.authoryMode = 0;
      osal_set_event(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT);
      Option_Auto = TRUE;
//      DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, openDoor);		         //     通知APP  门已开
    }
//    osal_mem_free(openDoor);

  }
  */
/*  
  else if(global.authoryMode ==2)                                                   //     APP点亮屏幕开启模式开门
  {
    uint8 * openDoor = osal_mem_alloc(3);
    uint8 * openDoor2 = osal_mem_alloc(3);
    openDoor[0] = 0x1F;
    openDoor[1] = 0x4B;
    openDoor[2] = 0x4F;
    if(global.safety_Rssi[0] <= global.safety_Rssi[1])				//     只有安全距离下才可以开门
    {
      global.authoryMode = 0;
      osal_set_event(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT);
      LL_Encrypt(key,openDoor,openDoor2);
      DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,openDoor2);		         //     通知APP  门已开
    }
    osal_mem_free(openDoor);
    osal_mem_free(openDoor2);
    
  }

*/
//   OLED_MENUMAIN();
//  ADCInit();
//  Adc_Start();

//  SendOrGetBatvalue(1);  // 1: send  0:get
  

}

/*********************************************************************
 * @fn      doorProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void lockProfileChangeCB( uint8 paramID )
{
  static uint8 responseData[DOORPROFILE_CHAR1_LEN]={0};
  static uint8 responseData2[DOORPROFILE_CHAR1_LEN]={0};
//  uint8 * responseData = osal_mem_alloc(DOORPROFILE_CHAR1_LEN);
//  uint8 * responseData2 = osal_mem_alloc(DOORPROFILE_CHAR1_LEN);
//  static uint8 replyUpdata[4] = {0};
  uint8 * replyUpdata = osal_mem_alloc(16);
  uint8 initial_advertising_enable;
//  uint8 timeReq[6];
  
  static uint8 reqStatus[16] = {0x4F, 0x4B, 0};

  switch( paramID )
  {
    case DOORPROFILE_CHAR1:
      DoorProfile_GetParameter( DOORPROFILE_CHAR1, responseData2 );
      LL_EXT_Decrypt(key,responseData2,responseData);
      osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config);
      switch(responseData[0])
      {
        case 0x01 :
        {
#if 1
          for(uint8 i=0; i<10; i++)
          {
            if(osal_memcmpn(&sys_config.Phone_User[i][1],&responseData[1],6))           // admin
            {
              
              Gap_Connect = Gapconnected;
              sys_recode.open_DoorRec[1] = 2;
              sys_recode.open_DoorRec[2] = i;
              osal_stop_timerEx( bleAutoLock_TaskID, DOOR_ENCRYPT_EVT );   
#if 0              
//              Adc_Start();
//              SendOrGetBatvalue(1);  // 1: send  0:get
//              Adc_Stop();
//              osal_stop_timerEx( bleAutoLock_TaskID, DOOR_ENCRYPT_EVT );   
              if( i == 0 )
              {
                global.authoryMode = 1;
//                osal_set_event( bleAutoLock_TaskID, DOOR_PHONE_EVT);		//进入管理员
              }
              else
              {
                global.authoryMode = 2;
//                osal_set_event( bleAutoLock_TaskID, DOOR_PHONE_EVT);		//进入普通手机用户
              }
              
              if((responseData[7] & 0x0F) != (sys_config.Phone_User[i][7] & 0x0F))       //手机APP更新了开门模式  开门模式若改变则重新保存
              {
                sys_config.Phone_User[i][7] = responseData[7];

              }
              if((responseData[7] & 0x0F) == 0x01)                              // open mode  自动
              {
                global.openDoorMode = 1;
                global.AppOpenStatus = 1;
              }
              else if((responseData[7] & 0x0F) == 0x02)			// 点亮屏幕开门
              {
                global.openDoorMode = 2;
              }
              if((responseData[7] & 0xF0) == 0x00 )                                //APP 重装
              {
                replyUpdata[0] = 0x21;
                if( i == 0 )
                {
                  replyUpdata[1] = 0x11;                                                 // 管理员权限
                }
                else
                {
                  replyUpdata[1] = 0x21;                                                 // 普通权限
                } 

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
                
              }
#endif
              if(global.AddPhoneFlag == 1)
              {
                global.menu_Num = 1;
                global.menu_Index = 0;
                OLED_INADMIN_MODE();
                OLED_MAINMENU_NUM( global.mainMenuUpDown );
                initial_advertising_enable = FALSE;
                
                // 断开所有连接
                GAPRole_TerminateConnection();

                 // 设置蓝牙广播状态
                GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );  
                
                osal_mem_free(replyUpdata);
                return;
              }
              if((responseData[7] & 0xF0) == 0x30 )                        //  手机用户删除自己本身
              {
                 sys_config.Phone_User[i][0] = 0xFF;
                 osal_memcpyz(&sys_config.Phone_User[i][1], 0, 7);
              }
              
//              global.authoryMode = 0;
              osal_set_event(bleAutoLock_TaskID, DOOR_OPENDOOR_EVT);
              Option_Auto = TRUE;
              break;
            }
            
            else
            {
//              Gap_Connect = Gapconnecting;
              if(i >= 9)
              {
                if( global.AddPhoneFlag == 1)
                {
                   osal_memcpy(global.appRecode_Temp, &responseData[1], 7);       // 如果没权限的手机则临时存储8字节应答数据
                   OLED_ADDPHONE_ENTER();
//                   osal_set_event(bleAutoLock_TaskID, DOOR_ADMIN_PRO_EVT);
                }

              }
            }
            if(osal_memcmpn(&global.Del_Phone_User[i][0],&responseData[1],6))
            {
              replyUpdata[0] = 0xCC;
              replyUpdata[1] = 0xCC;
              replyUpdata[2] = 0xCC;
              
              DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
              osal_memcpyz(&global.Del_Phone_User[i][0], 0, 6);
            }

          }

#else
	if(responseData[1] == 0x01)
	{
          DoorProfile_SetParameter(DOORPROFILE_CHAR2, 6, &sys_24c64.Tm_User[0][0]);
	}
	else if(responseData[1] == 0x02)
	{
          Motor_Backward();
//          osal_start_timerEx(bleAutoLock_TaskID, DOOR_PASSWORD_MANAGE_EVT, 500);
	}
	else if(responseData[1] == 0x03)
        {
//          osal_memcpy(&sys_24c64.Once_DayLimit[0][0],&UTCTimeSys,6);
        }
#endif

        }

        break;
        
        case 0x02:
          {
            static uint8 varUpdata[16] = {0, 0x53, 0x4A};
            uint8 * varUpdata2 = osal_mem_alloc(16);
            static uint8 falseReq[16] = {0, 0x43, 0x57};
            uint8 * tempReq = osal_mem_alloc(16);
            uint8 * tempReq2 = osal_mem_alloc(16);
            static uint8 sysUpdata_temp = 0;
            switch(global.sys_CurrentUp )
            {
                case 0x01:
                  falseReq[0] = 0x41;
                  global.sys_UpdataStatus |= 0x01;
                break;
                case 0x02:
                  falseReq[0] = 0x42;
                  global.sys_UpdataStatus |= 0x02;
                break;
                case 0x04:
                  falseReq[0] = 0x43;
                  global.sys_UpdataStatus |= 0x04;
                break;
                case 0x08:
                  falseReq[0] = 0x44;
                  global.sys_UpdataStatus |= 0x08;
                break;
                case 0x10:
                  falseReq[0] = 0x45;
                  global.sys_UpdataStatus |= 0x10;
                break;
                case 0x20:
                  falseReq[0] = 0x46;
                  global.sys_UpdataStatus |= 0x20;
                break;
                default:
//                       falseReq[0] = 0x47;
                break;
            }
            sysUpdata_temp = global.sys_UpdataStatus;
            reqStatus[2] = global.appUpdata_CRC;
            if(global.authoryMode == 1)
            {
              if(osal_memcmpn(reqStatus, &responseData[1], 3)  )
              {
                global.appUpdata_CRC = 0;
                for(uint8 i=0;i<7;i++)
                {
                  if(sysUpdata_temp&0x01)
                  {
                    switch(i)
                    {
                    case 0x00:                                                      //  上传手机       
                      varUpdata[0] = 0x41;
                      global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);

                      DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, varUpdata);

                      break;
                    case 0x01:                                                      //  管理员密码
                      varUpdata[0] = 0x42;
                      global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);

                      DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, varUpdata);
                      break;
                    case 0x02:                                                      //  普通按键密码
                      varUpdata[0] = 0x43;
                      global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);

                      DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, varUpdata);
                      break;
                    case 0x03:                                                      //  TM 卡信息        
                      varUpdata[0] = 0x44;
                      global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);

                      DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, varUpdata);

                      break;
                    case 0x04:                                                      // 开门记录
                      varUpdata[0] = 0x45;
                      global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);

                      DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, varUpdata);
                      break;
                    case 0x05:                                                      // 期限密码单次
                      varUpdata[0] = 0x46;
                      global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);

                      DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, varUpdata);
                      break;
                    case 0x06:                                                     // 结束
                      varUpdata[0] = 0x47;
                      global.sys_OptionFlag = 0;
                      global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);

                      DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, varUpdata);
                      
                      break;
                    default:
                      break;                  
                    }
                  }
                  sysUpdata_temp >>=1;
                }
              }
              else
              {
                if( global.sys_CurrentUp )
                {
                  global.appUpdata_CRC = 0;
                  switch(global.sys_CurrentUp )
                  {
                      case 0x01:
                        falseReq[0] = 0x41;
                      break;
                      case 0x02:
                        falseReq[0] = 0x42;
                      break;
                      case 0x04:
                        falseReq[0] = 0x43;
                      break;
//                      case 0x08:
//                        falseReq[0] = 0x44;
//                      break;
                      case 0x10:
                        falseReq[0] = 0x45;
                      break;
                      case 0x20:
                        falseReq[0] = 0x46;
                      break;
                      default:
 //                       falseReq[0] = 0x47;
                      break;
                  }

                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, falseReq);		//     上传错误信息
                  global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
                }
              }
            }

          }
        
        break;
        
        case 0x03:              //上传手机用户信息
        {
//          static uint8 replyUpdata[] = {0x02, 0x53, 0x4A,0x00};
          static uint8 falseReq[16] = {0, 0x43, 0x57};

          if( osal_memcmpn(reqStatus, &responseData[1], 2) && global.authoryMode == 1 )
          {     
            switch(responseData[4])
            {
            case 0x41:                                                          //  手机用户
              if(global.appUpdata_CRC == responseData[3])
              {
                global.appUpdata_CRC = 0;
                for(uint8 i=1; i<10; i++)
                {
                  global.appUpdata_CRC = appData_Crc(&sys_config.Phone_User[1][0], global.appUpdata_CRC, 8);        //获取校验码 CRC
                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, &sys_config.Phone_User[i][0]);
                }
                global.sys_CurrentUp = 0x01;
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x41;

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16,falseReq);		//     上传错误信息
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
              break;
              
            case 0x42:                                                          //  管理员密码
              if(global.appUpdata_CRC == responseData[3])
              {
                global.sys_UpdataStatus &= ~(BV(1));
                global.appUpdata_CRC = 0;
                global.appUpdata_CRC = appData_Crc(sys_config.Admin_Pswd, global.appUpdata_CRC, 8);               //获取校验码 CRC

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, sys_config.Admin_Pswd);
                global.sys_CurrentUp = 0x02;
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x42;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, falseReq);		//     上传错误信息
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
              break;
              
            case 0x43:                                                          //  普通按键密码
              if(global.appUpdata_CRC == responseData[3])
              {
                global.sys_UpdataStatus &= ~(BV(0));
                global.appUpdata_CRC = 0;
                for(uint8 i=0; i<10; i++)
                {
                  global.appUpdata_CRC = appData_Crc(&sys_24c64.General_Pswd[i][0], global.appUpdata_CRC, 9);               //获取校验码 CRC
                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, &sys_24c64.General_Pswd[i][0]);
                }
                global.sys_CurrentUp = 0x04;
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x43;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2,16,falseReq);		//     上传错误信息
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
              break;
               
            case 0x44:                                                          //  TM 卡信息
  
//              break;
              
            case 0x45:                                                          // 开门记录
              if(global.appUpdata_CRC == responseData[3])
              {
                global.sys_UpdataStatus &= ~(BV(3));
                global.appUpdata_CRC = 0;
                for(uint8 i=0; i<20; i++)
                {
                  global.appUpdata_CRC = appData_Crc(&sys_recode.Open_Recode[i][0], global.appUpdata_CRC, 9);               //获取校验码 CRC
                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, &sys_recode.Open_Recode[i][0]);
                }
                global.sys_CurrentUp = 0x10;
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x45;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, falseReq);		//     上传错误信息
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
              break;
              
            case 0x46:                                                          
              if(global.appUpdata_CRC == responseData[3])
              {
                global.sys_UpdataStatus &= ~(BV(4));
                global.appUpdata_CRC = 0;
                for(uint8 i=0; i<10; i++)
                {
                  global.appUpdata_CRC = appData_Crc(&sys_24c64.Once_Single[i][0], global.appUpdata_CRC, 7);               //获取校验码 CRC
                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, &sys_24c64.Once_Single[i][0]);
                }
                global.sys_CurrentUp = 0x20;
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x46;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, falseReq);		//     上传错误信息
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
              break;
            case 0x47:                                                          
              if(global.appUpdata_CRC == responseData[3])
              {
                global.appUpdata_CRC = 0;
                global.sys_UpdataStatus = 0;                                //  上传结束  更新标志位置 0
                global.sys_CurrentUp = 0x00;
                Init_Recode();//上传结束后清空开门记录。重新计算。
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x47;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, falseReq);		//     上传错误信息
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
              break;
              
            default:
              break;
            }
            
          }
          else
          {
            
          }
        }
        break;
		
        case 0x08:              //下发时间校准
        {	
          DsGet_Time();
          systime.cYear=  responseData[1];
          systime.cMon= responseData[2];
          systime.cDay= responseData[3];
          systime.cHour= responseData[4];
          systime.cMin= responseData[5];
          systime.cSec= responseData[6];
          DsSet_Time();

          replyUpdata[0] = 0x16;
          replyUpdata[1] = 0x4B;
          replyUpdata[2] = 0x4F;
          DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata); // 校准成功
//           osal_setClock(timeReq);
        }
        break;
        case 0x10: // 手机模式是点亮屏幕开启时
        {
          if((responseData[1]==0x4F)&&(responseData[2] == 0x50))
          {
            global.AppOpenStatus = 2;
            osal_set_event(bleAutoLock_TaskID, DOOR_PERIODIC_EVT);
          }
          
        }
        break;
/*        
        case 0x11: // 当手机有操作时候不进入低功耗，等待断开连接
        {
          
        }
        break;
*/
        default:
        // should not reach here!
        break;
	  
      }
      
      break;

    case DOORPROFILE_CHAR3:
//      osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config);
      DsGet_Time();
      global.appDedata_CRC = 0;
      DoorProfile_GetParameter( DOORPROFILE_CHAR3, responseData2 );
      LL_EXT_Decrypt(key,responseData2,responseData);
//      osal_mem_free(responseData2);
      switch(responseData[0])
      {
        case 0x01 :     // 删除手机用户
        {
          global.appDedata_CRC = appData_Crc(&sys_config.Phone_User[responseData[1]][0], global.appDedata_CRC, 8);
          if(global.authoryMode == 1 && global.appDedata_CRC == responseData[2])       //只有在管理员权限模式下才可以操作此项
          {
            global.app_DownChange = 0x01;
            osal_memcpy(&global.Del_Phone_User[global.Del_Phone_Num][0], &sys_config.Phone_User[responseData[1]][1], 6);
            global.Del_Phone_Num ++;
            if( global.Del_Phone_Num >= 9)
            {
              global.Del_Phone_Num = 0;
            }
            osal_memcpyz(&sys_config.Phone_User[responseData[1]][0], 0, 8);
            sys_config.Phone_User[responseData[1]][0] = 0xFF;
            replyUpdata[0] = 0x03;
            replyUpdata[1] = responseData[1];
            replyUpdata[2] = global.appDedata_CRC;

            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
          }
          else
          {
            replyUpdata[0] = 0x50;
            replyUpdata[1] = 0xFF;	     

            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
          }
        }

        break;
        
        case 0x02:      // 删除管理密码
        {
          static uint8 Init_Pswd[8] = "00000000";
          global.appDedata_CRC = appData_Crc(sys_config.Admin_Pswd, global.appDedata_CRC, 8);
          if(global.authoryMode == 1 && global.appDedata_CRC == responseData[1])       //只有在管理员权限模式下才可以操作此项
          {
            global.app_DownChange = 0x01;
            osal_memcpyz(sys_config.Admin_Pswd, 0, 8);
            osal_memcpy(sys_config.Admin_Pswd, Init_Pswd, 8);						//  被删除则恢复出厂设置
            replyUpdata[0] = 0x04;
            replyUpdata[1] = global.appDedata_CRC;

            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16,replyUpdata);
          }
          else
          {
            replyUpdata[0] = 0x51;
            replyUpdata[1] = 0xFF;	           

            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
          }
        }
        break;

        case 0x03:      // 删除用户密码
        {
          global.appDedata_CRC = appData_Crc(&sys_24c64.General_Pswd[responseData[1]][0], global.appDedata_CRC, 9);
          if(global.authoryMode == 1 && global.appDedata_CRC == responseData[2])       //只有在管理员权限模式下才可以操作此项
          {
            global.app_DownChange = 0x02;
            sys_24c64.General_Pswd[responseData[1]][0] = 0x00;
            osal_memcpyz(&sys_24c64.General_Pswd[responseData[1]][1], 0xBB, 9);
            replyUpdata[0] = 0x05;
            replyUpdata[1] = responseData[1];
            replyUpdata[2] = global.appDedata_CRC;

            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16,replyUpdata);
          }
          else
          {
            replyUpdata[0] = 0x52;
            replyUpdata[1] = 0xFF;	           

            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
          }
        }
        break;

        case 0x04:      // 删除TM卡
        {

        }
        break;
        
        case 0x05:      // 修改管理密码
        {
          global.appDedata_CRC = appData_Crc(&responseData[1], global.appDedata_CRC, 8);
          if(global.authoryMode == 1 && global.appDedata_CRC == responseData[9])       //只有在管理员权限模式下才可以操作此项
          {
            global.app_DownChange = 0x01;
            osal_memcpy(sys_config.Admin_Pswd, &responseData[1], 8);
            replyUpdata[0] = 0x07;
            replyUpdata[1] = 0x4B;          

            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16,replyUpdata);
          }
          else
          {
            replyUpdata[0] = 0x54;
            replyUpdata[1] = 0xFF;	           

            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
          }
        }
        break;
        
        case 0x06:      // 修改普通密码
        {
          global.appDedata_CRC = appData_Crc(&responseData[1], global.appDedata_CRC, 9);
          if(global.authoryMode == 1 && global.appDedata_CRC == responseData[10])       //只有在管理员权限模式下才可以操作此项
//          if(global.appDedata_CRC == responseData[8])
          {
            global.app_DownChange = 0x02;
            osal_memcpy(&sys_24c64.General_Pswd[responseData[1]][0], &responseData[1], 9);
            replyUpdata[0] = 0x08;
            replyUpdata[1] = 0x4B;	   
//            replyUpdata[2] = global.appDedata_CRC;

            DoorProfile_SetParameter(DOORPROFILE_CHAR2,16,replyUpdata);
          }
          else
          {
            replyUpdata[0] = 0x55;
            replyUpdata[1] = 0xFF;	           

            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
          }
        }
        break;
        
        case 0x07:      // 下发一次性密码
        {
          global.appDedata_CRC = appData_Crc(&responseData[1], global.appDedata_CRC, 10);
          if(global.authoryMode == 1 && global.appDedata_CRC == responseData[11])       //只有在管理员权限模式下才可以操作此项
//          if(global.appDedata_CRC == responseData[9])   //测试用
          {

            switch(responseData[1])
            {
              case 0x31:		//单次密码无需记录时间，用完立即清空
              {
                global.app_DownChange = 0x02;
                sys_24c64.Once_Single[responseData[2]][0] = responseData[2];
                osal_memcpy(&sys_24c64.Once_Single[responseData[2]][1], &responseData[3], 8);
                replyUpdata[0] = 0x09;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
              }
              break;
              
              case 0x32:
              {
                global.app_DownChange = 0x02;
                sys_24c64.Once_OneDay[responseData[2]][0] = responseData[2];
                osal_memcpy(&sys_24c64.Once_OneDay[responseData[2]][1], &responseData[3], 8);
                sys_24c64.Once_DayLimit[responseData[2]][0] = Ds1302_ConverUTCSecs(&systime);
//                  osal_memcpy(&sys_24c64.Once_DayLimit[responseData[2]][0],&UTCTimeSys,6);	//记录当前时间
                replyUpdata[0] = 0x09;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	        

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);                    
              }
              break;
              
              case 0x33:
              {
                global.app_DownChange = 0x02;
                sys_24c64.Once_OneWeek[responseData[2]][0] = responseData[2];
                osal_memcpy(&sys_24c64.Once_OneWeek[responseData[2]][1], &responseData[3], 8);
//                  osal_memcpy(&sys_24c64.Once_WeekLimit[responseData[2]][0],&UTCTimeSys,6);	//记录当前时间
                sys_24c64.Once_WeekLimit[responseData[2]][0] = Ds1302_ConverUTCSecs(&systime);
                replyUpdata[0] = 0x09;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	   

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
              }
              break;
              
              case 0x34:
              {
                global.app_DownChange = 0x02;
                sys_24c64.Once_AMonth[responseData[2]][0] = responseData[2];
                osal_memcpy(&sys_24c64.Once_AMonth[responseData[2]][1], &responseData[3], 8);
//                  osal_memcpy(&sys_24c64.Once_MonthLimit[responseData[2]][0],&UTCTimeSys,6);	//记录当前时间
                sys_24c64.Once_MonthLimit[responseData[2]][0] = Ds1302_ConverUTCSecs(&systime);
                replyUpdata[0] = 0x09;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	   

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
              }
              break;
                  
              default:
                break;

            }
          }
          else
          {
            replyUpdata[0] = 0x56;
            replyUpdata[1] = 0xFF;	            

            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
          }
        }
        break;
       case 0x08:																												//  删除一次性密码
        {
			
          if(global.authoryMode == 1 )       //只有在管理员权限模式下才可以操作此项
          {

            switch(responseData[1])
            {
            case 0x31:		//单次密码无需记录时间，用完立即清空
            {
              global.appDedata_CRC = appData_Crc(&sys_24c64.Once_Single[responseData[2]][0], global.appDedata_CRC, 9);
              if( global.appDedata_CRC == responseData[3] )
              {
                global.app_DownChange = 0x02;
                osal_memcpyz(&sys_24c64.Once_Single[responseData[2]][0], 0xBB, 16);		//  删除
                
                replyUpdata[0] = 0x0A;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
              }
              else
              {
                replyUpdata[0] = 0x57;
                replyUpdata[1] = 0xFF;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	       

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
              }
            }
            break;

            case 0x32:
            {
              global.appDedata_CRC = appData_Crc(&sys_24c64.Once_OneDay[responseData[2]][0], global.appDedata_CRC, 9);
              if( global.appDedata_CRC == responseData[3] )
              {
                global.app_DownChange = 0x02;
                osal_memcpyz(&sys_24c64.Once_OneDay[responseData[2]][0], 0xBB, 9);			// 删除
                sys_24c64.Once_DayLimit[responseData[2]][0] = 0;
                replyUpdata[0] = 0x0A;
                replyUpdata[1] = 0x4B;		   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	          

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);    
              }
              else
              {
                replyUpdata[0] = 0x57;
                replyUpdata[1] = 0xFF;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	          

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
              }
            }
            break;

            case 0x33:
            {
              global.appDedata_CRC = appData_Crc(&sys_24c64.Once_OneWeek[responseData[2]][0], global.appDedata_CRC, 9);
              if( global.appDedata_CRC == responseData[3] )
              {
                global.app_DownChange = 0x02;
                osal_memcpyz(&sys_24c64.Once_OneWeek[responseData[2]][0], 0xBB, 9);
                sys_24c64.Once_WeekLimit[responseData[2]][0] = 0;
                replyUpdata[0] = 0x0A;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	       

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
              }
              else
              {
                replyUpdata[0] = 0x57;
                replyUpdata[1] = 0xFF;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	        

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
              }
            }
            break;

            case 0x34:
            {
              global.appDedata_CRC = appData_Crc(&sys_24c64.Once_AMonth[responseData[2]][0], global.appDedata_CRC, 9);
              if( global.appDedata_CRC == responseData[3] )
              {
                global.app_DownChange = 0x02;
                osal_memcpyz(&sys_24c64.Once_AMonth[responseData[2]][0], 0xBB, 9);
                sys_24c64.Once_MonthLimit[responseData[2]][0] = 0;
                replyUpdata[0] = 0x0A;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	        

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
              }
              else
              {
                replyUpdata[0] = 0x57;
                replyUpdata[1] = 0xFF;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	       

                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 16, replyUpdata);
              }
            }
            break;

            default:
            break;

            }
          }
        }
        break;
        
       case 0x09:
         global.safety_Rssi[0] = responseData[1];
                
        break;
       case 0x0A:
         global.safety_Rssi[1] = responseData[1];
        break;

        default:
        // should not reach here!
//        osal_mem_free(replyUpdata);
        break;
          
      }


      break;

    default:
      // should not reach here!
      break;
  }
//  osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);		//修改完保存
//  osal_mem_free(responseData);
//  osal_mem_free(responseData2);

  osal_mem_free(replyUpdata);

}


static uint8 appData_Crc(uint8  *src,uint8 crc, uint8 len)   
{   
  uint8 i; 
  uint8 bb; 
  for(uint8 j=0;j<len;j++)
  {
    bb = src[j];
    for(i=8;i>0;--i)   
    {   
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
      bb>>=1;   
    }
  }   
  return(crc);   
} 

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

#if 1
static void Bat_AdcAuto(void)
{
  uint16 BatValue0,BatValue1,BatValue2 ;
//  uint16 temp;
  uint8 res[3] = {0,0,0};
  uint8 test[1] = {0xAA};
  Adc_Start();
  for(uint8 i=0;i<3;i++)
  {
    BatValue0    = BatADMeasure();
    BatValue1    = BatADMeasure();
    BatValue2    = BatADMeasure(); 

    global.batteryADC = ( BatValue0 < BatValue1 ?
   ((BatValue1 < BatValue2) ? BatValue1 : (BatValue0 < BatValue2 ? BatValue2 : BatValue0)) :
   ((BatValue1 > BatValue2) ? BatValue1 : (BatValue0 > BatValue2 ? BatValue2 : BatValue0)));
   
  
    if(global.batteryADC > 1980)  // 减小
    {
      res[0] = (uint8)(global.batteryADC - 1980);
      AT24C64_I2C_Write(0x1FFE, 1,(uint8 *) &res[0]);
      AT24C64_I2C_Write(0x1FF0, 1,test);
    }
    else if(global.batteryADC < 1980 && global.batteryADC > 1955) // 增加
    {
      res[1] = (uint8)(1980 - global.batteryADC);
      AT24C64_I2C_Write(0x1FFF, 1,(uint8 *) &res[1]);
      AT24C64_I2C_Write(0x1FF0, 1,test);
    }
    else if( global.batteryADC == 1980)
    {
      AT24C64_I2C_Write(0x1FF0, 1,test);
    }
    else
    {
  //    global.batteryADC = 0;
      res[2] = 0xBB;
      AT24C64_I2C_Write(0x1FF0, 1,(uint8 *) &res[2]);
    }
  }
  Adc_Stop();
}
#endif  

/*********************************************************************
*********************************************************************/
