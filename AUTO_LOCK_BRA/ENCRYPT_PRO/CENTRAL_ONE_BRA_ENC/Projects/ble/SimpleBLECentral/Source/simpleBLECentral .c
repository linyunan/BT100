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
#include "hal_aes.h"
#include "osal_snv.h"
/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */
#define SBP_PERIODIC_EVT_PERIOD              400//ms   发送数据的定时间隔， 可长可短， 建议 100~10000
#define SBP_PERIODIC_EVT_AUTO_CONNECT       1600//ms   连接从机的间隔， 不能太短， 请勿任意修改
// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

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
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

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


uint8 REQSCAN_CMP[14] = {
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
};

uint8 SEND_RSSI[2] = {0x09, 0x00};

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
/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 simpleBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 simpleBLEScanning = FALSE;

// RSSI polling state
static uint8 simpleBLERssi = FALSE;


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
static uint8 simpleBLECharVal = 0;

// Value read/write toggle
static bool simpleBLEDoWrite = FALSE;

// GATT read/write procedure state
static bool simpleBLEProcedureInProgress = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
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
  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  simpleBLECentralPasscodeCB,
  simpleBLECentralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

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
/*
  unsigned char state[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 
                               0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
  //unsigned char ciphertext[] = {0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
  //                              0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a};
  unsigned char key1[]   = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                           0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

  unsigned char key2[]   = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                           0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
*/  
//  aes_enc_dec(state,key1,0);

//  aes_enc_dec(state,key2,1);

//  ssp_HW_KeyInit( key );

//  sspAesEncryptHW( key1, state );

//  sspAesDecryptHW( key1, state );
 
  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

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
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
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

    return ( events ^ START_DEVICE_EVT );
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


  if ( events & SBP_SEND_RSSI_EVT )
  {
    OneConnetedDevice_WriteCharX(simpleBLEScanIdx, BLE_CHAR4, (uint8*)SEND_RSSI, 2);
    osal_start_timerEx(simpleBLETaskId, SBP_SEND_RSSI_EVT, 500);
    return ( events ^ SBP_SEND_RSSI_EVT );
  }
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
    if ( simpleBLEState == BLE_STATE_CONNECTED &&
    simpleBLECharHdl[charIdx] != 0 &&
    /*p->simpleBLEProcedureInProgress == FALSE &&*/
    simpleBLEConnHandle[charIdx] != GAP_CONNHANDLE_INIT)
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
        simpleBLEProcedureInProgress = TRUE;
        
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
  if ( keys & HAL_KEY_UP )
  {
    // Start or stop discovery

#if defined (MULT_PERIPHERAL_CONN)
//    if (connectedPeripheralNum < MAX_PERIPHERAL_NUM)
//#else
    if ( simpleBLEState != BLE_STATE_CONNECTED )
#endif
    {
      if ( !simpleBLEScanning )
      {
        simpleBLEScanning = TRUE;
        simpleBLEScanRes = 0;       
            
        LCD_WRITE_STRING( "Discovering...", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
    
        GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST );      
      }
      else 
      {       
        GAPCentralRole_CancelDiscovery();
      }
    }
   
  }

  if ( keys & HAL_KEY_LEFT )
  {
    // Display discovery results
    if ( !simpleBLEScanning && simpleBLEScanRes > 0 )
    {
        // Increment index of current result (with wraparound)
        simpleBLEScanIdx++;
        if ( simpleBLEScanIdx >= simpleBLEScanRes )
        {
          simpleBLEScanIdx = 0;
        }
        
        LCD_WRITE_STRING_VALUE( "Device", simpleBLEScanIdx + 1,
                                10, HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( simpleBLEDevList[simpleBLEScanIdx].addr ),
                          HAL_LCD_LINE_2 );
    }
  }

  if ( keys & HAL_KEY_RIGHT )
  {
#if 0  
    // Connection update
    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
#if defined (MULT_PERIPHERAL_CONN)
      GAPCentralRole_UpdateLink( simpleBLEConnHandle[0],
#else
      GAPCentralRole_UpdateLink( simpleBLEConnHandle,
#endif
                                 DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                                 DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                                 DEFAULT_UPDATE_SLAVE_LATENCY,
                                 DEFAULT_UPDATE_CONN_TIMEOUT );
    }
#endif

    char test[2] = {0x0A, 0x31};
#if 0
    attWriteReq_t AttReq;       
 
    
    AttReq.handle = p->simpleBLEConnHandle;//0x002A;        
    AttReq.len = 2;
    AttReq.sig = 0;
    AttReq.cmd = 0;
    
    
    osal_memcpy(AttReq.value, test, 2);
    GATT_WriteCharValue( p->simpleBLEConnHandle, &AttReq, simpleBLETaskId );
#else
    OneConnetedDevice_WriteCharX(simpleBLEScanIdx, BLE_CHAR4,
                                 (uint8*)test, 2);
#endif
  }

        
  if ( keys & HAL_KEY_CENTER )
  {
#if 0  
    uint8 addrType;
    uint8 *peerAddr;
    
    // Connect or disconnect
#if defined (MULT_PERIPHERAL_CONN)
    if ( simpleBLEState == BLE_STATE_IDLE 
        || connectedPeripheralNum < MAX_PERIPHERAL_NUM)
#else
    if ( simpleBLEState == BLE_STATE_IDLE )
#endif
    {
      // if there is a scan result
      if ( simpleBLEScanRes > 0 )
      {
        // connect to current device in scan result
        peerAddr = simpleBLEDevList[simpleBLEScanIdx].addr;
        addrType = simpleBLEDevList[simpleBLEScanIdx].addrType;
      
        simpleBLEState = BLE_STATE_CONNECTING;
        //osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                      DEFAULT_LINK_WHITE_LIST,
                                      addrType, peerAddr );
  
        LCD_WRITE_STRING( "Connecting", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( peerAddr ), HAL_LCD_LINE_2 ); 

      }
    }
    else if ( simpleBLEState == BLE_STATE_CONNECTING ||
              simpleBLEState == BLE_STATE_CONNECTED )
    {
      // disconnect
      simpleBLEState = BLE_STATE_DISCONNECTING;

#if defined (MULT_PERIPHERAL_CONN)
      gStatus = GAPCentralRole_TerminateLink( simpleBLEConnHandle[0] );
      gStatus = GAPCentralRole_TerminateLink( simpleBLEConnHandle[1] );
#else
      gStatus = GAPCentralRole_TerminateLink( simpleBLEConnHandle );
#endif
      
    }
#else
  OneConnetedDevice_WriteCharX(simpleBLEScanIdx, BLE_CHAR1,(uint8*)PHONE_NUM, 8);
#endif
  }
  
  if ( keys & HAL_KEY_DOWN )
  {
    // Start or cancel RSSI polling
    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
      if ( !simpleBLERssi )
      {
        simpleBLERssi = TRUE;
#if defined (MULT_PERIPHERAL_CONN)
        GAPCentralRole_StartRssi( simpleBLEConnHandle[0], DEFAULT_RSSI_PERIOD );
#else
        GAPCentralRole_StartRssi( simpleBLEConnHandle, DEFAULT_RSSI_PERIOD );
#endif
      }
      else
      {
        simpleBLERssi = FALSE;
#if defined (MULT_PERIPHERAL_CONN)
        GAPCentralRole_CancelRssi( simpleBLEConnHandle[0] );
#else
        GAPCentralRole_CancelRssi( simpleBLEConnHandle );
#endif        
        LCD_WRITE_STRING( "RSSI Cancelled", HAL_LCD_LINE_1 );
      }
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
 static uint8 ADDMODE[3] = {0x01, 0x4F, 0x4B};
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
      uint8 status = pMsg->msg.errorRsp.errCode;
      
      LCD_WRITE_STRING_VALUE( "Read Error", status, 10, HAL_LCD_LINE_1 );
#endif
    }
    else
    {
#if !defined (CC2540_MINIDK)
      // After a successful read, display the read value
      uint8 valueRead = pMsg->msg.readRsp.value[0];

      LCD_WRITE_STRING_VALUE( "Read rsp:", valueRead, 10, HAL_LCD_LINE_1 );
#endif    
    }
    simpleBLEProcedureInProgress = FALSE;
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP )
    {
#if !defined (CC2540_MINIDK)
      uint8 status = pMsg->msg.errorRsp.errCode;
      
      LCD_WRITE_STRING_VALUE( "Write Error", status, 10, HAL_LCD_LINE_1 );
#endif
    }
    else
    {
      // After a succesful write, display the value that was written and increment value
      LCD_WRITE_STRING_VALUE( "Write sent:", simpleBLECharVal++, 10, HAL_LCD_LINE_1 );      
    }
    
    simpleBLEProcedureInProgress = FALSE;    

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
//        BLE_DEV *p =  simpleBLECentralGetDev(pMsg->connHandle);
//        osal_memcpy(temp3, pMsg->msg.handleValueNoti.value, 16);
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
          osal_memcpy(&sys_config.MAC_RECODE[sys_config.mac_index][0], &simpleBLEDevList[simpleBLEScanIdx-1].addr, 6);
          sys_config.mac_index ++;
          
          LL_Encrypt(key,PHONE_NUM,temp3);
 //         LL_EXT_Decrypt(key,temp,temp2);
          req.handle = 0x0025;

          req.len = 16;

          osal_memcpy(req.value, temp3, 16);
          req.sig = 0;
          req.cmd = 0;
          GATT_WriteCharValue( simpleBLEConnHandle[0], &req, simpleBLETaskId );
          osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
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
  
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        LCD_WRITE_STRING( "BLE Central", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( pEvent->initDone.devAddr ),  HAL_LCD_LINE_2 );
        osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config);
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
           LL_EXT_Decrypt(key, pEvent->deviceInfo.pEvtData, request);
//           VOID osal_memcpy( request, pEvent->deviceInfo.pEvtData, 14  );
           if(osal_memcmp(request, REQSCAN_CMP, 14) && sys_config.UUID_STATUSE == TRUE)
           {
              sys_config.UUID_STATUSE = FALSE;
//              sys_config.Info_Name_Status = TRUE;
              simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
           }
           else
           {
//              sys_config.Info_Name_Status = FALSE;
           }
        } 
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
       simpleBLEScanning = FALSE;

        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          simpleBLEScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }

        LCD_WRITE_STRING_VALUE( "Devices Found", simpleBLEScanRes,
                                10, HAL_LCD_LINE_1 );
        performPeriodicTask_AutoConnect();
        osal_start_timerEx( simpleBLETaskId, SBP_AUTO_CONNECT_EVT, SBP_PERIODIC_EVT_AUTO_CONNECT );
        if ( simpleBLEScanRes > 0 )
        {
          LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
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
          simpleBLEProcedureInProgress = TRUE;    
          
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
//          GAPCentralRole_StartRssi( simpleBLEConnHandle[0], DEFAULT_RSSI_PERIOD );// 关键
          for(uint8 i=0;i<10;i++)
          {
            if(osal_memcmp(&sys_config.MAC_RECODE[i][0], &simpleBLEDevList[simpleBLEScanIdx-1].addr, 6))
            {
#if 0
              OneConnetedDevice_WriteCharX(simpleBLEScanIdx, BLE_CHAR1, (uint8*)PHONE_NUM, 8);
#else             
              attWriteReq_t req;
              req.handle = 0x0025;

              req.len = 16;
              LL_Encrypt(key,(uint8 *)PHONE_NUM,temp);
              osal_memcpy(req.value, temp, 16);
              req.sig = 0;
              req.cmd = 0;
              GATT_WriteCharValue( simpleBLEConnHandle[0], &req, simpleBLETaskId );
              osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
#endif
//              osal_set_event(simpleBLETaskId, SBP_SEND_RSSI_EVT);
            }
          }
          LCD_WRITE_STRING( "Connected", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING( bdAddr2Str( pEvent->linkCmpl.devAddr ), HAL_LCD_LINE_2 );   

        }
        else
        {
          simpleBLEState = BLE_STATE_IDLE;
#if defined (MULT_PERIPHERAL_CONN)
          simpleBLEConnHandle[connectedPeripheralNum] = GAP_CONNHANDLE_INIT;
#else
          simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
#endif
          simpleBLERssi = FALSE;
          simpleBLEDiscState = BLE_DISC_STATE_IDLE;
          
          LCD_WRITE_STRING( "Connect Failed", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING_VALUE( "Reason:", pEvent->gap.hdr.status, 10, HAL_LCD_LINE_2 );
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        simpleBLEState = BLE_STATE_IDLE;
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
        simpleBLERssi = FALSE;
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
        
        simpleBLEProcedureInProgress = FALSE;
//        GAPCentralRole_CancelRssi( simpleBLEConnHandle[0] );// 关键  
       
        LCD_WRITE_STRING( "Disconnected", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING_VALUE( "Reason:", pEvent->linkTerminate.reason,
                                10, HAL_LCD_LINE_2 );

      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        LCD_WRITE_STRING( "Param Update", HAL_LCD_LINE_1 );
      }
      break;
      
    default:
      break;
  }
}

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
    LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
    }
    else
    {
      LCD_WRITE_STRING_VALUE( "Pairing fail", status, 10, HAL_LCD_LINE_1 );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
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
    
           LCD_WRITE_STRING( "CHAR 1 Found", HAL_LCD_LINE_3 );      
           simpleBLEProcedureInProgress = TRUE;      
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
      LCD_WRITE_STRING( "CHAR 2 Found", HAL_LCD_LINE_4 );      
      simpleBLEProcedureInProgress = FALSE;  
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
      LCD_WRITE_STRING( "CHAR 4 Found", HAL_LCD_LINE_5 );      
      simpleBLEProcedureInProgress = FALSE;  
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
      if ( simpleBLEScanRes > 0 )
//        && simpleBLEScanIdx < simpleBLEScanRes )
      {
        // connect to current device in scan result
        peerAddr = simpleBLEDevList[simpleBLEScanIdx].addr;
        addrType = simpleBLEDevList[simpleBLEScanIdx].addrType;
      
        simpleBLEState = BLE_STATE_CONNECTING;
        //osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                      DEFAULT_LINK_WHITE_LIST,
                                      addrType, peerAddr );
  
        LCD_WRITE_STRING( "Connecting", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( peerAddr ), HAL_LCD_LINE_2 ); 
      }
    }
    else if ( simpleBLEState == BLE_STATE_CONNECTING ||
              simpleBLEState == BLE_STATE_CONNECTED ||
              simpleBLEState == BLE_STATE_DISCONNECTING)
    {
      // disconnect
      simpleBLEState = BLE_STATE_DISCONNECTING;

      gStatus = GAPCentralRole_TerminateLink( simpleBLEConnHandle[simpleBLEScanIdx] );
      
      LCD_WRITE_STRING( "Disconnecting", HAL_LCD_LINE_1 ); 
    }
}

/*********************************************************************
*********************************************************************/
