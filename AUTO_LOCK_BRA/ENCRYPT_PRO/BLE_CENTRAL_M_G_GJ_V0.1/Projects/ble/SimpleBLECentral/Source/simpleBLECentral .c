/**************************************************************************************************
  Filename:       simpleBLECentral.c
  Revised:        $Date: 2011-06-20 11:57:59 -0700 (Mon, 20 Jun 2011) $
  Revision:       $Revision: 28 $

  Description:    This file contains the Simple BLE Central sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

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
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "simpleBLECentral.h"
#include "stdio.h"
#include <stdlib.h>
//#include <time.h>

#include "hal_aes.h"
#include "osal_snv.h"
#include "pwm.h"
#include "led.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */
#define SBP_PERIODIC_EVT_PERIOD              400//ms   发送数据的定时间隔， 可长可短， 建议 100~10000
#define SBP_PERIODIC_EVT_AUTO_CONNECT        1600//ms   连接从机的间隔， 不能太短， 请勿任意修改
// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 1000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      40

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      80

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR1,               // 特征值1 
  BLE_DISC_STATE_CHAR2,  
  BLE_DISC_STATE_CHAR3,  
  BLE_DISC_STATE_CHAR4,  
  BLE_DISC_STATE_CHAR5,  
  BLE_DISC_STATE_CHAR6,              // 我们的主从一体串口透传使用的  ---amomcu---
};

enum
{
  BLE_CHAR1 = 0,
  BLE_CHAR2,
  BLE_CHAR3,
  BLE_CHAR4,
  BLE_CHAR5,
  BLE_CHAR6,
  BLE_CHAR7,
};

enum POWER_ONOFF {  PowerOff =0,  PowerOn};   //添加系统状态位
bool Power_OnOff = PowerOn ;

enum LED_STATUS {  Led_Off =0,  Led_On };   // LED
bool Led_Status = Led_Off;

uint8 REQSCAN_CMP1[12] = {
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
};

uint8 REQSCAN_CMP2[12] = {
                            0x47,   // 'G'
                            0x65,   // 'e'
                            0x6D,   // 'm'
                            0x69,   // 'i'
                            0x6E,   // 'n'
                            0x69,   // 'i'
                            0x2D,   // '-'
                            0x42,   // 'B'
                            0x41,   // 'A'
                            0x31,   // '1'
                            0x30,   // '0'
                            0x30,   // '0'
};
//uint8 SEND_RSSI[2] = {0x09, 0x00};

uint8 PHONE_NUM[16] = {0x01, 0x13, 0x77, 0x99, 0x55, 0x64, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static uint8 key[] = { 0x01, 0x03, 0x05, 0x07, 0xFF, 0xFE, 0x77, 0x88, 0x12, 0x99, 0x32, 0x41, 0x14, 0x24, 0x25, 0x36}; 
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
SYS_CONFIG sys_config;

GOBAL_VAR gobal_var;
/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 simpleBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
//uint8 simpleBLEScanRes = 0;
static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
//static uint8 simpleBLEScanning = FALSE;

// RSSI polling state
//static uint8 simpleBLERssi = FALSE;


#if defined (MULT_PERIPHERAL_CONN)
#define MAX_PERIPHERAL_NUM 2
uint8 connectedPeripheralNum = 0;
#endif

// Connection handle of current connection 
#if defined (MULT_PERIPHERAL_CONN)
static uint16 simpleBLEConnHandle[MAX_PERIPHERAL_NUM] = {GAP_CONNHANDLE_INIT};
#else
static uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
#endif
// Application state
static uint8 simpleBLEState = BLE_STATE_IDLE;

// Discovery state
static uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16 simpleBLESvcStartHdl = 0;
static uint16 simpleBLESvcEndHdl = 0;

// Discovered characteristic handle
#if !defined(MULT_CHAR_DISC)
static uint16 simpleBLECharHdl = 0;
#else
static uint16 simpleBLECharHdl[4] = {0};
#endif
// Value to write
//static uint8 simpleBLECharVal = 0;

// Value read/write toggle
//static bool simpleBLEDoWrite = FALSE;

// GATT read/write procedure state
//static bool simpleBLEProcedureInProgress = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );

//static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );

static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
//static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
//                                        uint8 uiInputs, uint8 uiOutputs );
//static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void simpleBLECentralStartDiscovery( void );
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
char *bdAddr2Str ( uint8 *pAddr );
static void performPeriodicTask_AutoConnect( void ) ; // 执行动连接
void OneConnetedDevice_WriteCharX(uint8 index, uint8 charIdx, uint8 value[20], uint8 valueLen);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  NULL,//  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  NULL,//simpleBLECentralPasscodeCB,
  NULL,//simpleBLECentralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
static uint8 ADDMODE[3] = {0x01, 0x4F, 0x4B};
static uint8 DELPHONE[3] = {0xCC, 0xCC, 0xCC};
static uint8 Delettest[8] = {0};

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

void Rand_Phone(void)
{
  uint8 RANDOM_KEY[6] = {0};
  uint8 T_TIME = 0;
  sys_config.Phone_Num[0] = 0x01;
  
  ReadMac(RANDOM_KEY);
  T_TIME = appData_Crc(RANDOM_KEY, T_TIME, 6);
  
  srand(T_TIME); //用时间做种，每次产生随机数不一样
  
  for (uint8 i=1; i<7; i++)
  {

    sys_config.Phone_Num[i] = rand() % 101;
//    sys_config.Phone_Num[i] = rand() % temp;  //产生0-100的随机数
  }   
}

void Sys_Init_Var(void)
{
  sys_config.ADD_PHONE_MODE = FALSE;
  sys_config.Info_Name_Status = FALSE;
  sys_config.mac_index = 0;
  sys_config.mac_per_index = 0;
  
  for(uint8 i=0;i<10;i++)
  {
    osal_memcpy(&sys_config.MAC_RECODE[i][0], Delettest, 16);
  }
  
  sys_config.MAC_SAVE_STATUS = FALSE;
  sys_config.UUID_STATUSE = FALSE;
  sys_config.SYS_ONOFF = FALSE;
  
//  osal_memcpy(sys_config.Phone_Num, PHONE_NUM, 16);
  
  Rand_Phone();
}

void Gobal_Init_Var(void)
{
  gobal_var.buz_num = 0;
  gobal_var.led_num = 0;
  gobal_var.init_num = 0;
}

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */

void SimpleBLECentral_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;
 
  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );
  
  GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, 100); //断开后立马扫描时间
  
  GAP_SetParamValue(TGAP_CONN_EST_LATENCY, 0);						// 设置从机广播允许的丢包率为0

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );
  
  P0SEL = 0x00; // Configure Port 1 as GPIO
  P1SEL &= ~(0xCF); // Configure Port 1 as GPIO

  P0DIR |= 0xFF; // All port 1 pins (P1.2) as output
  P1DIR |= 0xCF; // All port 1 pins (P1.2) as output

  P0 = 0;   // All pins on port 1 to low
  
  P1 &= ~(0xCF);   // All pins on port 1 to low
  
  Led_Init();
  
  LED_OFF();
  
//  Buzzer_Init();
  
  PWM_Init();
  
//  PWM_Pulse(0);
  
//  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
  
  #if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_BATTERY ); // 睡眠
  #endif
}

/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );

    osal_set_event( simpleBLETaskId, SBP_POWERONOFF_EVT );
//    osal_set_event( simpleBLETaskId, SBP_LEDONOFF_EVT );
    return ( events ^ START_DEVICE_EVT );
  }
  
  if( events & SBP_POWERONOFF_EVT)
  {
      Power_OnOff = !Power_OnOff; 

      
      if( Power_OnOff == PowerOn )					// 唤醒开机
      {
        #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // 唤醒
        #endif
        
        PWM_Init();
        
//        LED_ON();
        osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config);
        osal_set_event( simpleBLETaskId, SBP_LEDONOFF_EVT );
        
        osal_set_event( simpleBLETaskId, SBP_BUZZER_EVT );
        
        simpleBLEScanRes = 0;       
        
//        VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );
        
        GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST );      
        
        osal_start_timerEx( simpleBLETaskId, SBP_POWERONOFF_EVT, 5000 );
      }
      else
      {
        
        osal_set_event( simpleBLETaskId, SBP_BUZZER_EVT );
//        osal_set_event( simpleBLETaskId, SBP_OUTPOWER_EVT );
      }
      return ( events ^ SBP_POWERONOFF_EVT );
  }
  
  
  if( events & SBP_OUTPOWER_EVT)
  {
    
    simpleBLEState = BLE_STATE_IDLE;
    
    gobal_var.buz_num = 0;
    HAL_DISABLE_INTERRUPTS();
    if( sys_config.SYS_ONOFF == TRUE )
    {
      osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
    }
    HAL_ENABLE_INTERRUPTS();
    
    
    Buzzer_Init();
#if 1    
    osal_stop_timerEx( simpleBLETaskId, SBP_OUTPOWER_EVT );  
    
    osal_stop_timerEx( simpleBLETaskId, SBP_LEDONOFF_EVT );
    
    LED_OFF();
    
    GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                   FALSE,
                                   DEFAULT_DISCOVERY_WHITE_LIST ); 
    
    GAPCentralRole_TerminateLink( simpleBLEConnHandle[0] );
    
    GAPCentralRole_CancelDiscovery();
    
    osal_stop_timerEx( simpleBLETaskId, SBP_BUZZER_EVT );  
    osal_stop_timerEx( simpleBLETaskId, SBP_SEND_RSSI_EVT );  
    osal_stop_timerEx( simpleBLETaskId, SBP_AUTO_CONNECT_EVT );
    osal_stop_timerEx( simpleBLETaskId, START_DISCOVERY_EVT );
    
    osal_stop_timerEx( simpleBLETaskId, SBP_POWERONOFF_EVT );
    
    #if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_BATTERY ); // 睡眠
    #endif
    return ( events ^ SBP_OUTPOWER_EVT );
    
#endif
  }
    
  if( events & SBP_LEDONOFF_EVT)
  {
    if( Power_OnOff == PowerOn )
    {
      gobal_var.led_num ++;
      #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // 唤醒
      #endif
      if ( simpleBLEState == BLE_STATE_CONNECTED )
      {
        LED_ON();
        osal_start_timerEx( simpleBLETaskId, SBP_LEDONOFF_EVT, 100 );
        
      }
#if 1
      else if ( simpleBLEState != BLE_STATE_CONNECTED )
      {
        
        Led_Status = !Led_Status;
        {
          if( Led_Status )
          {
            LED_ON();        
          }
          else
          {
            LED_OFF();       
          }
        }
        
        osal_start_timerEx( simpleBLETaskId, SBP_LEDONOFF_EVT, 200 );
      }
#endif      
    }
    return ( events ^ SBP_LEDONOFF_EVT );
  }
  
  if ( events & SBP_BUZZER_EVT )
  {
    gobal_var.buz_num ++;
    if( Power_OnOff == PowerOn )
    {
#if 1      
      switch(gobal_var.buz_num)
      {
      case 1:
        PWM_Pulse( 100 );
        osal_start_timerEx(simpleBLETaskId, SBP_BUZZER_EVT, 200);
        break;
      case 2:
        PWM_Pulse( 0 );
        gobal_var.buz_num = 0;
        PWM_Init();
        osal_stop_timerEx(simpleBLETaskId, SBP_BUZZER_EVT );
        break;
      default:
        PWM_Pulse( 0 );
        gobal_var.buz_num = 0;
        PWM_Init();
        osal_stop_timerEx(simpleBLETaskId, SBP_BUZZER_EVT );
        break;
      }
      
#endif      
    }
    else
    {
      PWM_Pulse( 100 );
#if 0
      #if defined ( POWER_SAVING )
      osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // 唤醒
      #endif
#endif
      osal_start_timerEx( simpleBLETaskId, SBP_OUTPOWER_EVT, 200 );
    }
    
    return ( events ^ SBP_BUZZER_EVT );
  }
  
  if ( events & SBP_AUTO_CONNECT_EVT )
  {
     // 以下都是处理用户按下 CENTER 按键后自动连接每一个扫描到的从机的控制
     if ( simpleBLEScanRes > 0 
        && simpleBLEScanIdx < simpleBLEScanRes )
     {       
        simpleBLEScanIdx++;
        if(simpleBLEScanIdx == simpleBLEScanRes)
        {
            simpleBLEScanIdx = 0;    
        }
        else
        {
            performPeriodicTask_AutoConnect();  // 执行动连接
            osal_start_timerEx( simpleBLETaskId, SBP_AUTO_CONNECT_EVT, SBP_PERIODIC_EVT_AUTO_CONNECT );
        }
     }

     return (events ^ SBP_AUTO_CONNECT_EVT);
  }
  
  if ( events & START_DISCOVERY_EVT )
  {
    simpleBLECentralStartDiscovery( );
    
    return ( events ^ START_DISCOVERY_EVT );
  }

  if ( events & SBP_SYSINIT_EVT )
  {
    if(sys_config.SYS_ONOFF == TRUE)
    {
      gobal_var.init_num ++;
      PWM_Pulse(150);
      if(gobal_var.init_num <= 1)
      {
        Sys_Init_Var();
   
        sys_config.SYS_ONOFF = TRUE;
        
        osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config );
        
        osal_start_timerEx( simpleBLETaskId, SBP_SYSINIT_EVT, 200 );
      }
      else
      {
        simpleBLEScanRes = 0;  
        osal_stop_timerEx( simpleBLETaskId, SBP_SYSINIT_EVT );
        HAL_SYSTEM_RESET();
      }
    }
    else
    {
      osal_stop_timerEx( simpleBLETaskId, SBP_SYSINIT_EVT );
    }
    
    
    return ( events ^ SBP_SYSINIT_EVT );
  }
  
/*
  if ( events & SBP_SEND_RSSI_EVT )
  {
    OneConnetedDevice_WriteCharX(simpleBLEScanIdx, BLE_CHAR4, (uint8*)SEND_RSSI, 2);
    osal_start_timerEx(simpleBLETaskId, SBP_SEND_RSSI_EVT, 500);
    return ( events ^ SBP_SEND_RSSI_EVT );
  }
  */
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      simpleBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}


// 发送特征值 的函数
/*
uint8 index,   从机标 0， 1， 2 等
uint8 charIdx,  特征值索引   CHAR1~CHAR7
uint8 value[20], 要发送的数据 buffer，
uint8 valueLen  要发送的数据长度， 要注意 CHAR1~CHAR4 都是只有一个字节的， 
                CHAR5是5个字节， CHAR6是19个字节， 那CHAR7多少， 自己找找吧  ---amomcu
*/
void OneConnetedDevice_WriteCharX(uint8 index, uint8 charIdx, uint8 value[20], uint8 valueLen)
{
  uint8 temp[15] = {0};
//    if ( simpleBLEState == BLE_STATE_CONNECTED &&
//    simpleBLECharHdl[charIdx] != 0 &&
//    /*p->simpleBLEProcedureInProgress == FALSE &&*/
//    simpleBLEConnHandle[charIdx] != GAP_CONNHANDLE_INIT)
    {
        uint8 status;

        // Do a read or write as long as no other read or write is in progress
//        if ( simpleBLEDoWrite )
        {
            // Do a write
            attWriteReq_t req;

            req.handle = simpleBLECharHdl[charIdx];

            req.len = valueLen;
            
            LL_Encrypt(key, value, temp);
            //req.value[0] = value;//p->simpleBLECharVal;
            osal_memcpy(req.value, temp, valueLen);
            req.sig = 0;
            req.cmd = 0;

            //NPI_PrintValue("DoWrite :", p->simpleBLECharVal, 10);
#if defined (MULT_PERIPHERAL_CONN)
            status = GATT_WriteCharValue( simpleBLEConnHandle[0], &req, simpleBLETaskId );
#else
            status = GATT_WriteCharValue( simpleBLEConnHandle, &req, simpleBLETaskId );         
#endif
//            status = GATT_WriteCharValue( p->simpleBLEConnHandle, &req, simpleBLETaskId ); // 发送数据
            if(SUCCESS != status)
            {
//                NPI_PrintValue("Write Error :", status, 10);
            }
        }  
        /*
        else// 以下是读取数据的 函数， 我们没用到， 你加上去就ok
        {
            // Do a read
            attReadReq_t req;
            req.handle = p->simpleBLECharHdl[charIdx];
            NPI_PrintValue("DoRead :", req.handle, 10);
            status = GATT_ReadCharValue( p->simpleBLEConnHandle, &req, simpleBLETaskId );  
        }
*/
//        simpleBLEProcedureInProgress = TRUE;
        
        // 请注意， 把以下这一句打开， 那么就会执行读数据了， 后面会执行写一次数据， 然后读一次数据
        //p->simpleBLEDoWrite = !(p->simpleBLEDoWrite);
    }  
}

/*********************************************************************
 * @fn      simpleBLECentral_HandleKeys
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
uint8 gStatus;
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
#if 1
  if ( keys & HAL_KEY_SW_5 )
  {
    osal_start_timerEx( simpleBLETaskId, SBP_SYSINIT_EVT, 5000);
  }
  else
  {
    osal_stop_timerEx( simpleBLETaskId, SBP_SYSINIT_EVT );
    osal_set_event( simpleBLETaskId, SBP_POWERONOFF_EVT );
  }
#else
  if ( keys & HAL_KEY_SW_5 )
  {
    osal_set_event( simpleBLETaskId, SBP_POWERONOFF_EVT );
  }
#endif
}

/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */

static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  attWriteReq_t req;
//  uint8 temp[8] = {0};
  uint8 temp2[16] = {0};
  uint8 temp3[16] = {0};
//  ssp_HW_KeyInit( key );
  if ( simpleBLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }
  
  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )
    {
#if !defined (CC2540_MINIDK)
//      uint8 status = pMsg->msg.errorRsp.errCode;
      
//      LCD_WRITE_STRING_VALUE( "Read Error", status, 10, HAL_LCD_LINE_1 );
#endif
    }
    else
    {
#if !defined (CC2540_MINIDK)
      // After a successful read, display the read value
//      uint8 valueRead = pMsg->msg.readRsp.value[0];

//      LCD_WRITE_STRING_VALUE( "Read rsp:", valueRead, 10, HAL_LCD_LINE_1 );
#endif    
    }
//    simpleBLEProcedureInProgress = FALSE;
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP )
    {
#if !defined (CC2540_MINIDK)
//      uint8 status = pMsg->msg.errorRsp.errCode;
      
//      LCD_WRITE_STRING_VALUE( "Write Error", status, 10, HAL_LCD_LINE_1 );
#endif
    }
    else
    {
      // After a succesful write, display the value that was written and increment value
//      LCD_WRITE_STRING_VALUE( "Write sent:", simpleBLECharVal++, 10, HAL_LCD_LINE_1 );      
    }
    
//    simpleBLEProcedureInProgress = FALSE;    

  }
  else if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
  {
    simpleBLEGATTDiscoveryEvent( pMsg );
  }
  else if ( ( pMsg->method == ATT_HANDLE_VALUE_NOTI ) )   //通知
  {
    if( pMsg->msg.handleValueNoti.handle == 0x0027)     //CHAR2
    {
//      uint8 temp2[16] = {0};
//      uint8 key[] = { 0x01, 0x03, 0x05, 0x07, 0xFF, 0xFE, 0x77, 0x88, 0x12, 0x99, 0x32, 0x41, 0x14, 0x24, 0x25, 0x36}; 
//        ssp_HW_KeyInit( key );
        // 根据 connHandle 查找对应的设备
        LL_EXT_Decrypt(key, pMsg->msg.handleValueNoti.value, temp2);
//        sspAesEncryptHW( key, temp2 );
//        sspAesDecryptHW(key, temp2);
        if(osal_memcmp(temp2, ADDMODE, 3 ))
        {
          sys_config.ADD_PHONE_MODE = TRUE;
          if(sys_config.mac_index == 10)     //最多设置10个
          {
             return;
          }
          if( sys_config.SYS_ONOFF == TRUE )
          {
            osal_memcpy(&sys_config.MAC_RECODE[sys_config.mac_index][0], &simpleBLEDevList[simpleBLEScanIdx-1].addr, 6);
            sys_config.mac_index ++;
          }
          LL_Encrypt(key,sys_config.Phone_Num,temp3);
 //         LL_EXT_Decrypt(key,temp,temp2);
          req.handle = 0x0025;

          req.len = 16;

          osal_memcpy(req.value, temp3, 16);
          req.sig = 0;
          req.cmd = 0;
          if( sys_config.SYS_ONOFF == TRUE )
          {
            GATT_WriteCharValue( 0, &req, simpleBLETaskId );//simpleBLEConnHandle[0]
          }
//          osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
        }
        else if( osal_memcmp(temp2, DELPHONE, 3 ) )
        {
          osal_memcpy(&sys_config.MAC_RECODE[sys_config.mac_index - 1][0], Delettest, 8);
        }

    }
    else if( pMsg->msg.handleValueNoti.handle == 0x0028)     //CHAR2
    {
        LL_EXT_Decrypt(key, pMsg->msg.handleValueNoti.value, temp2);

        if(osal_memcmp(temp2, ADDMODE, 3 ))
        {
          sys_config.ADD_PHONE_MODE = TRUE;
          if(sys_config.mac_index == 10)     //最多设置10个
          {
             return;
          }
          osal_memcpy(&sys_config.MAC_RECODE[sys_config.mac_index][0], &simpleBLEDevList[simpleBLEScanIdx-1].addr, 6);
          sys_config.mac_index ++;
          
          LL_Encrypt(key,sys_config.Phone_Num,temp3);
 //         LL_EXT_Decrypt(key,temp,temp2);
          req.handle = 0x0025;

          req.len = 16;

          osal_memcpy(req.value, temp3, 16);
          req.sig = 0;
          req.cmd = 0;
          GATT_WriteCharValue( simpleBLEConnHandle[0], &req, simpleBLETaskId );
//          osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
        }
        else if( osal_memcmp(temp2, DELPHONE, 3 ) )
        {
          osal_memcpy(&sys_config.MAC_RECODE[sys_config.mac_index - 1][0], Delettest, 8);
        }
    }
  }  
}

/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
#if 0
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{

  char str[32]={0};
  
  SEND_RSSI[1] =  (uint8) (-rssi);
#if 0        
  attWriteReq_t AttReq;       
  AttReq.handle = 0x002A;        
  AttReq.len = 2;
  AttReq.sig = 0;
  AttReq.cmd = 0;
  test1[1] =  rssi;
  
  osal_memcpy(AttReq.value, test1, 2);
  GATT_WriteCharValue( 0, &AttReq, simpleBLETaskId );

#else
//  OneConnetedDevice_WriteCharX(simpleBLEScanIdx, BLE_CHAR4, (uint8*)test1, 2);
#endif

  sprintf(str, "[%d].rssi = -%d db", connHandle, (uint8) (-rssi));
  LCD_WRITE_STRING(str, HAL_LCD_LINE_2+connHandle );

}
#endif
/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  uint8 request[16] = {0};
  uint8 request2[16] = {0};
  if( Power_OnOff == PowerOn )
  {
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {

      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        
        // if filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )       // UUID 和设备名称都对的时候做个临时拷贝
        {
          if ( simpleBLEFindSvcUuid( SIMPLEPROFILE_SERV_UUID,
                                     pEvent->deviceInfo.pEvtData,
                                     pEvent->deviceInfo.dataLen ) )  
          {
              sys_config.UUID_STATUSE = TRUE;
              GAPCentralRole_CancelDiscovery();
                
          }
        }
        if(pEvent->deviceInfo.eventType == GAP_ADTYPE_SCAN_RSP_IND)       //scan_rsp  
        {  
           VOID osal_memcpy( request2, pEvent->deviceInfo.pEvtData, 14  );
           LL_EXT_Decrypt(key, pEvent->deviceInfo.pEvtData, request);
//           VOID osal_memcpy( request, pEvent->deviceInfo.pEvtData, 14  );
           if( sys_config.UUID_STATUSE == TRUE )
           {
             if( osal_memcmp(&request[2], REQSCAN_CMP1, 12)  )
             {
                sys_config.UUID_STATUSE = FALSE;
  //              sys_config.Info_Name_Status = TRUE;
                simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
             }
             if (osal_memcmp(&request[2], REQSCAN_CMP2, 12))
             {
  //              sys_config.Info_Name_Status = FALSE;
                sys_config.UUID_STATUSE = FALSE;
  //              sys_config.Info_Name_Status = TRUE;
                simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
             }
           }
        } 
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
//       simpleBLEScanning = FALSE;

        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          simpleBLEScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }
//        if( Power_OnOff == PowerOn )
        {
          if( simpleBLEScanRes > 0 )
          {
            performPeriodicTask_AutoConnect();
            osal_start_timerEx( simpleBLETaskId, SBP_AUTO_CONNECT_EVT, SBP_PERIODIC_EVT_AUTO_CONNECT );
          }
          else
          {
            simpleBLEScanRes = 0;
            GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                           DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                           DEFAULT_DISCOVERY_WHITE_LIST );   
          }
        }
        // initialize scan index to last device
        simpleBLEScanIdx = simpleBLEScanRes;

      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        uint8 temp[16] = {0};
        if ( pEvent->gap.hdr.status == SUCCESS )
        {          
          simpleBLEState = BLE_STATE_CONNECTED;
//          osal_set_event( simpleBLETaskId, SBP_LEDONOFF_EVT );
#if defined (MULT_PERIPHERAL_CONN)
          simpleBLEConnHandle[connectedPeripheralNum] = pEvent->linkCmpl.connectionHandle;
          connectedPeripheralNum++;
          // Just for demo, we do not actually save both peripheral's characteristic values handles, 
          // after second peripheral is connected, clean simpleBLECharHdl, for second peripheral's service discovery
          simpleBLECharHdl[0] = 0;
          simpleBLECharHdl[1] = 0;
#else
          simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;
#endif
//          simpleBLEProcedureInProgress = TRUE;    
          
          // If service discovery not performed initiate service discovery
#if !defined(MULT_CHAR_DISC)
          if ( simpleBLECharHdl == 0 )
#else
          // There is no characteristic records in the list, do the service discovery.
          if ( simpleBLECharHdl[0] == 0 )
#endif
          {
            osal_start_timerEx( simpleBLETaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
          }

          for(uint8 i=0;i<10;i++)
          {
            if(osal_memcmp(&sys_config.MAC_RECODE[i][0], &simpleBLEDevList[simpleBLEScanIdx-1].addr, 6))
            {
          
              attWriteReq_t req;
              req.handle = 0x0025;

              req.len = 16;
              LL_Encrypt(key,(uint8 *)sys_config.Phone_Num,temp);
              osal_memcpy(req.value, temp, 16);
              req.sig = 0;
              req.cmd = 0;
              GATT_WriteCharValue( simpleBLEConnHandle[0], &req, simpleBLETaskId );
              
            }
          }  

        }
        else
        {
          simpleBLEState = BLE_STATE_IDLE;
#if defined (MULT_PERIPHERAL_CONN)
          simpleBLEConnHandle[connectedPeripheralNum] = GAP_CONNHANDLE_INIT;
#else
          simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
#endif
//          simpleBLERssi = FALSE;
          simpleBLEDiscState = BLE_DISC_STATE_IDLE;
          
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        simpleBLEState = BLE_STATE_IDLE;
//        osal_set_event( simpleBLETaskId, SBP_LEDONOFF_EVT );
#if defined (MULT_PERIPHERAL_CONN)
          connectedPeripheralNum--;
          // Make sure both links are disconnected.
          if (connectedPeripheralNum == 0)
          {
              simpleBLEConnHandle[0] = GAP_CONNHANDLE_INIT;
              simpleBLEConnHandle[1] = GAP_CONNHANDLE_INIT;

              simpleBLEDiscState = BLE_DISC_STATE_IDLE;
          }
          simpleBLEScanRes = 0;
          simpleBLEScanIdx = 0;
#else
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
#endif
//        simpleBLERssi = FALSE;
#if !defined (MULT_PERIPHERAL_CONN)
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
#endif

#if !defined(MULT_CHAR_DISC)
        simpleBLECharHdl = 0;
#else
        simpleBLECharHdl[0] = 0;
        simpleBLECharHdl[1] = 0;
        simpleBLECharHdl[2] = 0;
        simpleBLECharHdl[3] = 0;
#endif
        

      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
//        LCD_WRITE_STRING( "Param Update", HAL_LCD_LINE_1 );
      }
      break;
      
    default:
      break;
  }
  }
}
#if 0
/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
//    LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
//      LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
    }
    else
    {
//      LCD_WRITE_STRING_VALUE( "Pairing fail", status, 10, HAL_LCD_LINE_1 );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
//      LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8   str[7];

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}
#endif
/*********************************************************************
 * @fn      simpleBLECentralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void simpleBLECentralStartDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                   HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
  
  // Initialize cached handles

#if !defined(MULT_CHAR_DISC)
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;
#else
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = 0;
#endif
  simpleBLEDiscState = BLE_DISC_STATE_SVC;
  
  // Discovery simple BLE service
#if defined (MULT_PERIPHERAL_CONN)
  GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle[connectedPeripheralNum - 1],
#else
  GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle,
#endif   
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 simpleBLETaskId );
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */


static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{ 
  
   attReadByTypeReq_t req;  
   if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )   
   {    
      // Service found, store handles    
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&       
       pMsg->msg.findByTypeValueRsp.numInfo > 0 )   
      {      
        simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;  
        simpleBLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;    
       }    
  
      // If procedure complete    
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  &&           
      pMsg->hdr.status == bleProcedureComplete ) ||         
      ( pMsg->method == ATT_ERROR_RSP ) )   
      {      
        if ( simpleBLESvcStartHdl != 0 )      
        {        
          // Discover characteristic       
          simpleBLEDiscState = BLE_DISC_STATE_CHAR1;      
          req.startHandle = simpleBLESvcStartHdl;    
          req.endHandle = simpleBLESvcEndHdl;    
          req.type.len = ATT_BT_UUID_SIZE;
          req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);    
          req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);      
#if defined (MULT_PERIPHERAL_CONN)
          GATT_ReadUsingCharUUID( simpleBLEConnHandle[connectedPeripheralNum - 1], &req, simpleBLETaskId );     
#else
          GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );     
#endif
        }    
      }  
  }  
  
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR1 )  
  {    
    // Characteristic found, store handle    
     if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&      
        pMsg->msg.readByTypeRsp.numPairs > 0 )
        {      
           simpleBLECharHdl[0] = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],                         
           pMsg->msg.readByTypeRsp.dataList[1] );      
    
//           LCD_WRITE_STRING( "CHAR 1 Found", HAL_LCD_LINE_3 );      
//           simpleBLEProcedureInProgress = TRUE;      
       }
     else // pMsg->msg.readByTypeRsp.numPairs is 0.
     {
       simpleBLEDiscState = BLE_DISC_STATE_CHAR2;    
       req.startHandle = simpleBLESvcStartHdl;  
       req.endHandle = simpleBLESvcEndHdl;   
       req.type.len = ATT_BT_UUID_SIZE;    
       req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR2_UUID);   
       req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR2_UUID);    
#if defined (MULT_PERIPHERAL_CONN)
       GATT_ReadUsingCharUUID( simpleBLEConnHandle[connectedPeripheralNum - 1], &req, simpleBLETaskId );  
#else
       GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );  
#endif
     }
  }  
  
  else if (simpleBLEDiscState == BLE_DISC_STATE_CHAR2)  
  {     // Characteristic found, store handle    
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&     
    pMsg->msg.readByTypeRsp.numPairs > 0 )    
    {      
      simpleBLECharHdl[1] = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],                                      
      pMsg->msg.readByTypeRsp.dataList[1] );     
//      LCD_WRITE_STRING( "CHAR 2 Found", HAL_LCD_LINE_4 );      
//      simpleBLEProcedureInProgress = FALSE;  
    }    
    else // pMsg->msg.readByTypeRsp.numPairs is 0.
     {
       simpleBLEDiscState = BLE_DISC_STATE_CHAR4;    
       req.startHandle = simpleBLESvcStartHdl;  
       req.endHandle = simpleBLESvcEndHdl;   
       req.type.len = ATT_BT_UUID_SIZE;    
       req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR4_UUID);   
       req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR4_UUID);    
#if defined (MULT_PERIPHERAL_CONN)
       GATT_ReadUsingCharUUID( simpleBLEConnHandle[connectedPeripheralNum - 1], &req, simpleBLETaskId );  
#else
       GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );  
#endif
     }
  }  
  
  else if (simpleBLEDiscState == BLE_DISC_STATE_CHAR4)  
  {     // Characteristic found, store handle    
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&     
    pMsg->msg.readByTypeRsp.numPairs > 0 )    
    {      
      simpleBLECharHdl[3] = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],                                      
      pMsg->msg.readByTypeRsp.dataList[1] );     
//      LCD_WRITE_STRING( "CHAR 4 Found", HAL_LCD_LINE_5 );      
//      simpleBLEProcedureInProgress = FALSE;  
    } 
    simpleBLEDiscState = BLE_DISC_STATE_IDLE;
  }

}

/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
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


// 执行动连接
static void performPeriodicTask_AutoConnect( void )  // 执行动连接
{
    uint8 addrType;
    uint8 *peerAddr;

    if ( simpleBLEState == BLE_STATE_IDLE )
    {
      // if there is a scan result
//      if ( simpleBLEScanRes > 0 )
//        && simpleBLEScanIdx < simpleBLEScanRes )
      {
        // connect to current device in scan result
        peerAddr = simpleBLEDevList[simpleBLEScanIdx-1].addr;
        addrType = simpleBLEDevList[simpleBLEScanIdx-1].addrType;
      
        simpleBLEState = BLE_STATE_CONNECTING;
        //osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                      DEFAULT_LINK_WHITE_LIST,
                                      addrType, peerAddr );
  
//        LCD_WRITE_STRING( "Connecting", HAL_LCD_LINE_1 );
//        LCD_WRITE_STRING( bdAddr2Str( peerAddr ), HAL_LCD_LINE_2 ); 
      }
    }
    else if ( simpleBLEState == BLE_STATE_CONNECTING ||
              simpleBLEState == BLE_STATE_CONNECTED ||
              simpleBLEState == BLE_STATE_DISCONNECTING)
    {
      // disconnect
      simpleBLEState = BLE_STATE_DISCONNECTING;

      gStatus = GAPCentralRole_TerminateLink( simpleBLEConnHandle[simpleBLEScanIdx] );
      
//      LCD_WRITE_STRING( "Disconnecting", HAL_LCD_LINE_1 ); 
    }
}

/*********************************************************************
*********************************************************************/
