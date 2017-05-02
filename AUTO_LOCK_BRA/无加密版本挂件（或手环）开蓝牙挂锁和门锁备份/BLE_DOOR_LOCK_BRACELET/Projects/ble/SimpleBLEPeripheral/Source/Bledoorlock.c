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

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"

#include "doorGATTprofile.h"

#if defined( CC2540_MINIDK )
//  #include "simplekeys.h"
#endif

#include "peripheral.h"

#include "gapbondmgr.h"

#include "Bledoorlock.h"

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
//#include "crc.h"
#include "simpleble.h"
#include "tmcar.h"
#include "nv080c.h"
#include "ds1302.h"
    
#include "osal_snv.h"
//static UTCTimeStruct UTCTimeSys; //160421

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define DOOR_PERIODIC_EVT_PERIOD                   600


// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160       // �㲥����� ��ֵԽ�󹦺�Խ�͵��ǹ㲥�İ���ʱ������̫��

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     20//80//  ���Ӽ�������ݷ������йأ� ���Ӽ��Խ�̣� ��λʱ���ھ��ܷ���Խ�������

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     80//800//  ���Ӽ�������ݷ������йأ� ���Ӽ��Խ�̣� ��λʱ���ھ��ܷ���Խ�������

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          100//1000  -����ԭ��Ͽ����Ӻ󣬳�ʱ�����¹㲥��ʱ��:  100 = 1s

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE//FALSE//TRUE

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
static uint8 bleDoorLock_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;



/*********************************************************************
 * globle variable don't need save
 *
 */


enum POWER_ONOFF {  PowerOff =0,  PowerOn};   //���ϵͳ״̬λ
bool Power_OnOff = PowerOn ;

enum GAP_CONNECT {  Gapconnecting =0,  Gapconnected};   //Ȩ��ģʽ�µ�����״̬
bool Gap_Connect = Gapconnecting ;

enum OPTION_DOOR {  Door_Close =0,  Door_Open};   //������״̬λ
bool Option_Door = Door_Close;

//enum KEY_ISR {  Key_Onetouch =0,  Key_Sectouch};   //��������
//bool Key_Isr = Key_Onetouch;





// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x0D,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,

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
//  0x51,   // 'Q'

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( DOORPROFILE_SERV_UUID ),
  HI_UINT16( DOORPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "BA100";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void bleDoorLock_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void doorStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void doorProfileChangeCB( uint8 paramID );
//static void doorLockRssiCB( int8 newRSSI )  ;
static uint8 appData_Crc(uint8  *src,uint8 crc, uint8 len);   

static void bleDoorLock_HandleKeys( uint8 shift, uint8 keys );

//static void Battery_Auto(void);
static uint8 SendOrGetBatvalue(uint8 value);    

static void Bat_AdcAuto(void);
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
//static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)



/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t bleDoorLock_PeripheralCBs =
{
  doorStateNotificationCB,  // Profile State Change Callbacks
  NULL//doorLockRssiCB//NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t bleDoorLock_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static doorProfileCBs_t bleDoorLock_DoorProfileCBs =
{
  doorProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */



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

void BleDoorLock_Init( uint8 task_id )
{
//  uint8 test[7] = {0x01,0x34,0x35,0x36,0x37,0x38,0x39};
//  uint8 crc = 0;
//  crc = appData_Crc(test, crc, 7);
  
  bleDoorLock_TaskID = task_id;

  Init_NV080C();
  
//  Init_GlobleVar();
  
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
      uint8 initial_advertising_enable = FALSE;//TRUEFALSE
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


	//����RSSI�Ķ�ȡ���ʣ�Ĭ����0  
//	uint16 desired_rssi_rate = 100;  
//	GAPRole_SetParameter(GAPROLE_RSSI_READ_RATE,sizeof(uint16),&desired_rssi_rate); 
//    uint8 advType = GAP_ADTYPE_ADV_IND;
//    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
	
    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
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
  RegisterForKeys( bleDoorLock_TaskID );   //  һ����Ҫ�������� ���򰴼���������


//#ifdef wdog  
  //���Ź���ʼ��
  watchdog_init();
//#endif

  ADCInit();
  
  Adc_Stop();
  
  TmLed_Init();
  
//  TmLed_True();
  
  TmLed_Close();

  Motor_Init();
  
  Ds1302_Init();
  DS1302_InitConf();
//  DelayMS(500);
  
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

  MBR3_RESET();
  
  DelayMS(500);

  // Register callback with SimpleGATTprofile
  VOID DoorProfile_RegisterAppCBs( &bleDoorLock_DoorProfileCBs );
  
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
  osal_set_event( bleDoorLock_TaskID, DOOR_START_DEVICE_EVT );

  #if defined ( POWER_SAVING )
//     osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // Revert to battery mode after LED off    PWRMGR_ALWAYS_ON  PWRMGR_BATTERY
  #endif

}




/********************************************************************
*   ��ؼ��
*   ʱ��: 2015-07-28
*   ���ߣ�������
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
  
//    static uint16 BatteryTempValue = 1638 ;
//    float BatValue0,BatValue1,BatValue2 ;
//    float temp;
    uint16 BatValue0,BatValue1,BatValue2 ;
    uint16 temp;
    uint8 res[3] = {0};
//    float temp1 = 0;
    uint8 * responseData = osal_mem_alloc(8);
//    if( value < 3  )
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

    global.batteryADC = ( BatValue0 < BatValue1 ?
   ((BatValue1 < BatValue2) ? BatValue1 : (BatValue0 < BatValue2 ? BatValue2 : BatValue0)) :
   ((BatValue1 > BatValue2) ? BatValue1 : (BatValue0 > BatValue2 ? BatValue2 : BatValue0)));
   

    
#if 0      

//#else
           
      responseData[3] = ((uint16)global.batteryADC)/10000;
      responseData[4] = ((uint16)global.batteryADC)%10000/1000;
      responseData[5] = ((uint16)global.batteryADC)%10000%1000/100;
      responseData[6] = ((uint16)global.batteryADC)%10000%1000%100/10;
      responseData[7] = ((uint16)global.batteryADC)%10000%1000%100%10;
//#else
      responseData[3] = res[0];
      responseData[4] = res[1]; 
      responseData[5] = res[2];
#endif
#if 0 
  if(res[2] != 0xAA)  
  {
    if(global.batteryADC > 1943)  // ��С
    {
      res[0] = (uint8)(global.batteryADC - 1943);
      AT24C64_I2C_Write(0x1FFE, 1,(uint8 *) &res[0]);
      AT24C64_I2C_Write(0x1FF0, 1,test);
    }
    else if(global.batteryADC < 1943 && global.batteryADC > 1923) // ����
    {
      res[1] = (uint8)(1943 - global.batteryADC);
      AT24C64_I2C_Write(0x1FFF, 1,(uint8 *) &res[1]);
      AT24C64_I2C_Write(0x1FF0, 1,test);
    }
    else if( global.batteryADC == 1943)
    {
      AT24C64_I2C_Write(0x1FF0, 1,test);
    }
    else
    {
      global.batteryADC = 0;
      res[2] = 0xBB;
      AT24C64_I2C_Write(0x1FF0, 1,(uint8 *) &res[2]);
    }
  }
#endif  
#if 1
  if(res[0])
  {
    global.batteryADC = global.batteryADC - res[0];//1623 - 20
  }
  else if(res[1])
  {
    global.batteryADC = global.batteryADC + res[1];//1623 - 20
  }
  if( global.batteryADC < 1600 )
  {
    global.batteryPer = 100;
  }
  else
  {
    temp =(uint16) global.batteryADC - 1603; 
    global.batteryPer = (uint8)( temp*100/340 ) ;
  }
  
#else  
  if( global.batteryADC < 1603 )
    {
      global.batteryPer = 0;
    }
    else
    {

      temp =(uint16) global.batteryADC - 1603; 
      if(res[0])
      {
        global.batteryPer = (uint8)( temp*100/340 ) + res[0];//1623 - 20
      }
      else if(res[1])
      {
        global.batteryPer = (uint8)( temp*100/340 ) - res[1];//1623 - 20
      }
      else if(res[0] == 0 && res[1] == 0)
      {
        global.batteryPer = (uint8)( temp*100/340 ) ;//1623 - 20
      }
    }
#endif
    if( (gapProfileState == GAPROLE_CONNECTED) && (value == 1))    
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
//      responseData[3] = global.batteryNumOut;   //(uint8)global.batteryPer;
      DoorProfile_SetParameter( DOORPROFILE_CHAR2, 3, responseData );
    }
    
    osal_mem_free(responseData);
    return global.batteryPer;
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
uint16 BleDoorLock_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( bleDoorLock_TaskID )) != NULL )
    {
      bleDoorLock_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & DOOR_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &bleDoorLock_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &bleDoorLock_BondMgrCBs );

    // Set timer for first periodic event
//    osal_start_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT, DOOR_PERIODIC_EVT_PERIOD );
//    osal_start_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT, 10 ); 
    osal_set_event(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT);
    
    return ( events ^ DOOR_START_DEVICE_EVT );
  }

  
//��Ϊϵͳʵʱ��������
  if ( events & DOOR_PERIODIC_EVT )
  {
    // Perform periodic application task
    performPeriodicTask();
//    if( DOOR_PERIODIC_EVT_PERIOD )
    osal_start_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT, DOOR_PERIODIC_EVT_PERIOD );
    
    return (events ^ DOOR_PERIODIC_EVT);
  }

//  ���� ��������
  if( events & DOOR_POWERONOFF_EVT)
  {
      Power_OnOff = !Power_OnOff; 

//      osal_memcpyz(global.batteryPower, 0, 5);
//      global.batteryNumOut = 0;
      uint8 initial_advertising_enable = TRUE;
     // �Ͽ���������
      GAPRole_TerminateConnection();

       // ���������㲥״̬
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
      
      osal_set_event( bleDoorLock_TaskID, DOOR_PERIODIC_EVT );    // ��ѭ��
      if( Power_OnOff == PowerOn )					// ���ѿ���
      {
        
        
        #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // ����
        #endif
        
        
        HAL_DISABLE_INTERRUPTS();
  //      AT24C64_I2C_Read(0x0000 , 1, res);
        AT24C64_I2C_Read(0x0000, sizeof(SYS_24C64), (uint8*)&sys_24c64);
        
        AT24C64_I2C_Read(0x1000, sizeof(SYS_24C64TIME), &sys_24c64time);
        HAL_ENABLE_INTERRUPTS(); 
        
        
       
        
//        GAP_UpdateAdvertisingData( bleDoorLock_TaskID, FALSE, sizeof(attDeviceName), attDeviceName );

        
        
        //  ����һ��ִ��һ��һ��������ʱ��Ƚ�
//        osal_start_timerEx(bleDoorLock_TaskID, DOOR_ONCEPSWD_EVT,100);				

        DsGet_Time();

        BackLight_ON();
        
        send_line(0x00);
        
//        osal_set_event( bleDoorLock_TaskID, DOOR_BATTERY_EVT );
        
//        osal_start_timerEx(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT, 15000);		//  15���޲�������˯��

        
      }
#if 1
      else											// ʱ�䵽�ػ�
      {
/*        
        uint8 initial_advertising_enable = FALSE;
        
        // �Ͽ���������
        GAPRole_TerminateConnection();

         // ���������㲥״̬
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );  

//        GAP_UpdateAdvertisingData( bleDoorLock_TaskID, FALSE, sizeof(attDeviceName), attDeviceName );
        */
        global.longButtonFlag = FALSE;
        
        global.menu_Num = 0;
        
        global.menu_Index = 0;

        global.openDoorTurn = 0;

        global.AddPhoneFlag = 0;

        Motor_Stop();

        TmLed_Close();
  	BackLight_ON();												//  ��ʱȫ�ֱ�����ʼ��
//        Ds1302_Init();
        
        if(global.sys_UpdataStatus || global.app_DownChange == 0x01)
        {
          global.sys_UpdataStatus &= ~(BV(7));
          HAL_DISABLE_INTERRUPTS();							//  ����˯�ߺ������µ�����
          osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
          HAL_ENABLE_INTERRUPTS();
        }
        
        if(global.sys_UpdataStatus ||global.app_DownChange == 0x02)
        {
          HAL_DISABLE_INTERRUPTS();							//  ����˯�ߺ������µ�����
          AT24C64_I2C_Write(0x0000, sizeof(SYS_24C64), (uint8*)&sys_24c64);  
          AT24C64_I2C_Write(0x1000, sizeof(SYS_24C64TIME), &sys_24c64time);
          HAL_ENABLE_INTERRUPTS();

        }
/*        
        BackLight_OFF();
        osal_stop_timerEx(bleDoorLock_TaskID, DOOR_READCAR_EVT);
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_PHONE_EVT);             //
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT);                 //
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT);                 //
  //      osal_stop_timerEx( bleDoorLock_TaskID, DOOR_PASSWORD_MANAGE_EVT);         //
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT ); 
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_ADMININ_EVT);             //
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);             //
        osal_stop_timerEx(bleDoorLock_TaskID, DOOR_OUTTIME_EVT);
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_PSWD_OPEN_EVT);             //
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_ONCEPSWD_EVT);             //
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);             //
        osal_stop_timerEx(bleDoorLock_TaskID, DOOR_BATTERY_EVT);
        
        #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_BATTERY ); // ����͹���
        #endif
        */
      }
#endif
    return ( events ^ DOOR_POWERONOFF_EVT );
  }
 
  // �����������Ա��������
  if ( events & DOOR_ADMININ_EVT)
  {
    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMININ_EVT);
    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
    
    global.longButtonFlag = !global.longButtonFlag;
    
    global.menu_Num = 0;
    
    TmLed_Close();
    
    global.safety_Flag = 0;
     
    global.menu_Index = 0;
    
    osal_memcpyz(global.MvInPswd, 0xFF, 10);
    
    if(global.longButtonFlag)
    {
      uint8 initial_advertising_enable = FALSE;
      
      // �Ͽ���������
      GAPRole_TerminateConnection();

       // ���������㲥״̬
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );  

//      GAP_UpdateAdvertisingData( bleDoorLock_TaskID, FALSE, sizeof(attDeviceName), attDeviceName );
      
      osal_stop_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT);                // ��������Աģʽʱ��ص���ѭ��
      
      

//      osal_stop_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT); 
      
      send_line(0x03);
    }
    else
    {
      uint8 initial_advertising_enable = TRUE;
      
      // �Ͽ���������
      GAPRole_TerminateConnection();

       // ���������㲥״̬
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );  

//      GAP_UpdateAdvertisingData( bleDoorLock_TaskID, FALSE, sizeof(attDeviceName), attDeviceName );
      
//      osal_start_timerEx( bleDoorLock_TaskID, DOOR_PERIODIC_EVT, DOOR_PERIODIC_EVT_PERIOD );    // �˳���������Աģʽʱ�� ������ѭ��
      
//      osal_start_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT, 100 ); 
      
      osal_stop_timerEx(bleDoorLock_TaskID, DOOR_OUTTIME_EVT);

      osal_stop_timerEx(bleDoorLock_TaskID, DOOR_READCAR_EVT);
      
      osal_set_event( bleDoorLock_TaskID, DOOR_PERIODIC_EVT);                // ��������Աģʽʱ��ص���ѭ��
      
      osal_set_event( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT);
      
      send_line(0x27);
    }

    return (events ^ DOOR_ADMININ_EVT);
  }


  
// �������Աģʽ��㼶����
  if ( events & DOOR_ADMIN_PRO_EVT)
  {
    static uint8 choseMode[7][2] = { 
                                      {0x0A,0x0B},              // "*" "#"
                                      {0x31,0x0B},              // "1" "#"
                                      {0x32,0x0B},              // "2" "#"
                                      {0x33,0x0B},              // "3" "#"
                                      {0x34,0x0B},              // "4" "#"
                                      {0x35,0x0B},              // "5" "#"
                                      {0x36,0x0B}               // "6" "#"
                                    };
    global.menu_Index = 0;
    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_OUTTIME_EVT);
//    osal_start_timerEx(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT, 10000);	
    switch( global.menu_Num )
    {
      
      case 0:			//�������Ա����
        if(osal_memcmpn(sys_config.Admin_Pswd, global.MvInPswd, 6))
        {
//          osal_start_timerEx(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT, 25000);		//  10���޲�������˯��
          global.menu_Num = 1;
          send_line(0x22);
          osal_memcpyz(global.MvInPswd, 0xFF, 10);
        }
        else
        {
               //���������������
          global.menu_Num = 0;     
          send_line(0x04);
          global.safety_Flag ++;
          osal_memcpyz(global.MvInPswd, 0xFF, 10);
          if(global.safety_Flag >= 6)
          {
            DsGet_Time();
            global.time_Recode = Ds1302_ConverUTCSecs(&systime);
            osal_set_event(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT);
          }
        }

        break;
      case 1:			//ѡ��
      {
        if(osal_memcmpn(&choseMode[0][0], global.MvInPswd, 2))				// "*" "#"
        {
          
          global.menu_Num = 2;
          send_line(0x05);
        }
        else if(osal_memcmpn(&choseMode[1][0], global.MvInPswd, 2))		// "1" "#"	����Ա����
        {
          global.menu_Num = 3;
          send_line(0x09);
        }
        else if(osal_memcmpn(&choseMode[2][0], global.MvInPswd, 2))		// "2" "#" ��ͨ�û�����
        {
          global.menu_Num = 4;
          osal_memcpyz(global.MvInPswd, 0xFF, 10);
          send_line(0x0D);
        }
        else if(osal_memcmpn(&choseMode[3][0], global.MvInPswd, 2))		// "3" "#" TM ��
        {
          send_line(0x12);
          global.menu_Num = 5;
          osal_set_event(bleDoorLock_TaskID, DOOR_READCAR_EVT);
        }
        else if(osal_memcmpn(&choseMode[4][0], global.MvInPswd, 2))		// "4" "#" ɾ��TM  ���û�
        {
          send_line(0x12);
          global.menu_Num = 6;
          osal_set_event(bleDoorLock_TaskID, DOOR_READCAR_EVT);
        }
        else if(osal_memcmpn(&choseMode[5][0], global.MvInPswd, 2))		// "5" "#"  ɾ���ڼ�����ͨ�����û�
        {
          global.menu_Num = 7;
//          osal_memcpyz(global.MvInPswd, 0xFF, 10);
          send_line(0x19);
        }
        else if(osal_memcmpn(&choseMode[6][0], global.MvInPswd, 2))		// "6" "#"   ɾ���ֻ�
        {
          global.menu_Num = 8;
          send_line(0x1d);
        }
        else
        {
          global.menu_Num = 1;
        }
        osal_start_timerEx(bleDoorLock_TaskID, DOOR_OUTTIME_EVT, 7000);
        global.menu_Index = 0;
      }
        break;
      case 2:
        {
//          static uint8 outPhoneMode[] = {0x01, 0x4F, 0x4F};    //֪ͨ�˳����ģʽ
          uint8 *replyUpdata = osal_mem_alloc(4);
          uint8 crc = 0;
//          if((global.AddPhoneFlag == 1) && (global.MvInPswd[1] == 0x0B || global.MvInPswd[2] == 0x0B))    //��ӵ��ֻ��б��ȥ 
          {
            if((sys_config.Phone_User[global.addPhoneNum][0] == 0xFF) && (global.AddPhoneFlag == 1))
            {
              if( global.addPhoneNum == 0 )
              {
                global.appRecode_Temp[6] |= 0x10;                                                //����ԱȨ��
                global.sys_UpdataStatus |= 0x80;
              }
              else
              {
                global.appRecode_Temp[6]  |= 0x20;                                                //��ͨȨ��
                global.sys_UpdataStatus |= 0x41;
              }
              sys_config.Phone_User[global.addPhoneNum][0] = global.addPhoneNum;			//����Ÿ�ֵ���б���Ϣ
              osal_memcpy(&sys_config.Phone_User[global.addPhoneNum][1], global.appRecode_Temp, 7);
              crc = appData_Crc(&global.appRecode_Temp[0], crc, 6);
              send_line(0x07);
     
              
              replyUpdata[0] = 0x2F;
              replyUpdata[1] = global.addPhoneNum;
              replyUpdata[2] = global.appRecode_Temp[6];
              replyUpdata[3] = crc;
//              DoorProfile_SetParameter(DOORPROFILE_CHAR2, 4, replyUpdata);
              osal_memcpyz(global.appRecode_Temp, 0, 7);
//              osal_set_event(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT);
            }
            else
            {

              send_line(0x24);
//              DoorProfile_SetParameter(DOORPROFILE_CHAR2, 3, outPhoneMode);      //֪ͨ�˳����ģʽ
              
            }
          }

          osal_mem_free(replyUpdata);
      	  }
        global.addPhoneNum = 0xFF;
        global.AddPhoneFlag = 0;
        global.menu_Num = 1;
        break;
      case 3:
        if(osal_memcmp(global.keyRecode_Temp, global.MvInPswd, 6))
        {
          osal_memcpy(sys_config.Admin_Pswd, global.keyRecode_Temp, 6);
          
  //        osal_memcpy(global.MvInPswd, 0, 10);
          global.sys_UpdataStatus |= 0x42;
          send_line(0x0B);
        }
        else
        {
          //����ʧ��
          send_line(0x0C);
        }
        osal_memcpyz(global.keyRecode_Temp, 0xBB, 6);
        global.menu_Num = 1;
        break;
       case 4:
        if(osal_memcmp(global.keyRecode_Temp, global.MvInPswd, 6))
        {
          sys_24c64.General_Pswd[global.addPswdNum][0] = global.addPswdNum;
          osal_memcpy(&sys_24c64.General_Pswd[global.addPswdNum][1], global.keyRecode_Temp, 6);
          
  //        osal_memcpy(global.MvInPswd, 0, 10);
          global.sys_UpdataStatus |= 0x44;
          send_line(0x11);
        }
        else
        {
          send_line(0x0C);//����ʧ��
        }
        global.addPswdNum = 0xFF;
        osal_memcpyz(global.keyRecode_Temp, 0xBB, 6);
        global.menu_Num = 1;
        break;
      case 5:
        {
//          uint8 j=0;
          uint8 * pt = osal_mem_alloc(8);
          uint8 zero[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

          if(global.MvInPswd[0] == 0x0B)
          {
//            if(Read1990a(pt))
            {
//              if(osal_memcmpn(&pt[1], global.tmRecode_Temp, 6))
              {                
                for(uint8 i=0;i<10;i++)
                {
                 if(osal_memcmpn(&sys_24c64.Tm_User[i][0],zero,6) )    //  ��Ϊ 0xFF ˵��Ϊ�գ��������
                 {
                     osal_memcpy(&sys_24c64.Tm_User[i][0], global.tmRecode_Temp, 6);             //TM �����������
                     global.menu_Num = 1;
                     osal_memcpyz(global.tmRecode_Temp , 0xBB, 6);
                     global.sys_UpdataStatus |= 0x48;
                     send_line(0x14);
                     break;

                  }
                  else
                  {
                   if(i == 9)
                   {
  //                      send_line(0x15);           // TM ���Ѿ�����
    //                 global.menu_Num = 1;
                     break;
                   }
                  }

                }
              }
//              else  // ���ζ�ȡ��TM����һ��
              {
                //���Ŷ�ȡʧ��
//                send_line(0x15);
//                break;
              }
            }
          }
          osal_mem_free(pt);
          global.menu_Num = 1;
        }
        
        break;
      case 6:
       {         
          static uint8 xing[]={0x0A,0x0A};
          uint8 * pt = osal_mem_alloc(8);
          if(osal_memcmpn(xing, global.MvInPswd, 2))
//          if(global.MvInPswd[0] == 0x0B)
          {
//            Read1990a(pt);
            for(uint8 i=0;i<10;i++)
            {
//              if(osal_memcmpn(&sys_24c64.Tm_User[i][0], &pt[1], 6))
              {
                if(osal_memcmpn(&sys_24c64.Tm_User[i][0], global.tmRecode_Temp, 6))
//                if(osal_memcmpn(global.tmRecode_Temp,&pt[1],6))
                {
                  osal_memcpyz(&sys_24c64.Tm_User[i][0], 0xFF, 6);             //TM ��ɾ���ɹ�
                  osal_memcpyz(global.tmRecode_Temp , 0xBB, 6);
                  global.menu_Num = 1;
                  global.sys_UpdataStatus |= 0x48;
                  send_line(0x18);
                  break;
                }
                else
                {
                    //���Ŷ�ȡʧ��
                  if(i == 9)
                  {
                      send_line(0x17);//�˿�������
                  }
                }
              }
//              else
//              {
//                  if(i == 9)
//                  {
//                      send_line(0x17);//�˿�������
//                  }
//              }
            }
          }
          osal_mem_free(pt);
          global.menu_Num = 1;
        }
        
        break;
      case 7:
      {
//        static uint8 xing[]={0x0A,0x0A};
//        if(osal_memcmpn(xing, global.MvInPswd, 2))
        if(global.MvInPswd[0] == 0x0B)
        {
 //           sys_24c64.General_Pswd[global.addPswdNum][0] = 0;
            osal_memcpyz(&sys_24c64.General_Pswd[global.addPswdNum][0], 0xBB, 7);		// ɾ������ɹ�
            global.sys_UpdataStatus |= 0x44;
            
            send_line(0x1B);
        } 
        global.menu_Num = 1;
      }
        break;
      case 8:
      {
//        static uint8 xing[]={0x0A,0x0A};
//        if(osal_memcmpn(xing, global.MvInPswd, 2))
        if(global.MvInPswd[0] == 0x0B)
        {
          sys_config.Phone_User[global.addPswdNum][0] = 0xFF;
          osal_memcpyz(&sys_config.Phone_User[global.addPswdNum][1], 0, 7);		// ɾ���ֻ��û��ɹ�
          
          global.sys_UpdataStatus |= 0x41;
          send_line(0x1B);
        }
        global.menu_Num = 1;
      }
        break;
      default:
        
        break;
    }
    global.addPswdNum = 0xFF;
    global.menu_Index = 0;
    osal_memcpyz(global.MvInPswd, 0xFF, 10);
    
    return (events ^ DOOR_ADMIN_PRO_EVT);
  }


 // �����ֻ�ģʽ����
  if ( events & DOOR_PHONE_EVT)
  {
    static uint8 replyUpdata[] = {0x02, 0x53, 0x4A,0x00};
    uint8 test[3] = {0x00};
    switch(global.authoryMode)
    {
      case 1:
        if( gapProfileState == GAPROLE_CONNECTED )
        { 
          if(global.sys_UpdataStatus && !global.longButtonFlag && ((global.sys_UpdataStatus&0x80)!= 0x80 ))		// ���ǵ�һ�����ù���Ա�ֻ������±�־λΪ0xFF ��Ҫ�ϴ���Ϣ��������λ������Աģʽ�����
          {
            global.appUpdata_CRC = 0;
 	         global.sys_UpdataStatus |= 0x40;
            global.appUpdata_CRC = appData_Crc(replyUpdata, global.appUpdata_CRC, 3);
            DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,replyUpdata);		//     ֪ͨҪ�ϴ��û���Ϣ
          }
          else
          {
            test[0] = global.sys_UpdataStatus;
            test[1] = global.longButtonFlag;
            test[2] = gapProfileState;
            DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,test);		//     ֪ͨҪ�ϴ��û���Ϣ
          }
//          osal_start_timerEx(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT, 10000);		//  10���޲�������˯��
        }
      break;
      case 2:
//        global.AppOpenStatus = 0;
//        osal_set_event(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);
      break;
      default:
      break;

    }
    
    
    return (events ^ DOOR_PHONE_EVT);
  }


// �������뿪��
  if ( events & DOOR_PSWD_OPEN_EVT)
  {                            
//    static uint8 openDoor[] = {0x1F, 0x4B, 0x4F};
    
    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_PSWD_OPEN_EVT);
    #if 0
    for(uint8 i=0;i<100;i++)                                                         
    {
      if(osal_memcmpn(&sys_24c64.General_Pswd[i][1], global.MvInPswd, 6))      //  ��ͨ����
      {
//        global.genPswdFlag = 0;							 // ״̬λ����
        sys_24c64.open_DoorRec[1] = 1;
        sys_24c64.open_DoorRec[2] = i;
        osal_memcpy(global.MvInPswd, 0, 10);                                     // �����ʱ 10λ��λ����     
        osal_set_event(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);
//        DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,openDoor);		         // ֪ͨAPP  ���ѿ�
        break;
      }
      else
      {
        if(i == 99)                                                              //���Ұ�
        {
          global.safety_Flag ++;
          if(global.safety_Flag>=6)
          {
            DsGet_Time();
            global.time_Recode = Ds1302_ConverUTCSecs(&systime);
            osal_set_event(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT);
          }
        }
      }
    }
    #endif
    for(uint8 i=0;i<10;i++)                                                         
    {
      if(osal_memcmpn(&sys_24c64.Once_Single[i][1], global.MvInPswd, 6))   // һ�������� ����
      {
//        global.genPswdFlag = 0;							 // ״̬λ����
        sys_24c64.open_DoorRec[1] = 4;
        sys_24c64.open_DoorRec[2] = i;
        global.sys_UpdataStatus |= 0x60;
        osal_memcpyz(global.MvInPswd, 0xFF, 10);                                     // �����ʱ 10λ��λ����
//        sys_24c64.Once_Single[i][0] = 0x00;
        osal_memcpyz(&sys_24c64.Once_Single[i][0], 0xBB, 7);                             // ͬʱɾ��һ��������
        osal_set_event(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);
        
        break;
      }
      else if(osal_memcmpn(&sys_24c64.General_Pswd[i][1], global.MvInPswd, 6))      //  ��ͨ����
      {
//        global.genPswdFlag = 0;							 // ״̬λ����
        sys_24c64.open_DoorRec[1] = 1;
        sys_24c64.open_DoorRec[2] = i;
        osal_memcpyz(global.MvInPswd, 0xFF, 10);                                     // �����ʱ 10λ��λ����     
        osal_set_event(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);

        break;
      }
      
      else if(osal_memcmpn(&sys_24c64.Once_OneDay[i][1], global.MvInPswd, 6))   // һ�������� һ��
      {
//        global.genPswdFlag = 0;							 // ״̬λ����
        sys_24c64.open_DoorRec[1] = 5;
        sys_24c64.open_DoorRec[2] = i;
        osal_memcpyz(global.MvInPswd, 0xff, 10);                                     // �����ʱ 10λ��λ����
        osal_set_event(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);

        break;
      }
      else if(osal_memcmpn(&sys_24c64.Once_OneWeek[i][1], global.MvInPswd, 6))   // һ�������� һ����
      {
//        global.genPswdFlag = 0;							 // ״̬λ����
        sys_24c64.open_DoorRec[1] = 6;
        sys_24c64.open_DoorRec[2] = i;
        osal_memcpyz(global.MvInPswd, 0xFF, 10);                                     // �����ʱ 10λ��λ����
        osal_set_event(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);

        break;
      }
      else if(osal_memcmpn(&sys_24c64.Once_AMonth[i][1], global.MvInPswd, 6))   // һ�������� һ����
      {
//        global.genPswdFlag = 0;							 // ״̬λ����
        sys_24c64.open_DoorRec[1] = 7;
        sys_24c64.open_DoorRec[2] = i;
        osal_memcpyz(global.MvInPswd, 0xFF, 10);                                     // �����ʱ 10λ��λ����
        osal_set_event(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);

        break;
      }
      else
      {
        if(i == 9)                                                              //���Ұ�
        {
          global.safety_Flag ++;
          if(global.safety_Flag>=6)
          {
            DsGet_Time();
            global.time_Recode = Ds1302_ConverUTCSecs(&systime);
            osal_set_event(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT);
          }
          send_line(04);
        }
      }
    }
    
    return (events ^ DOOR_PSWD_OPEN_EVT);
  }
  

  // һ���������������
  if ( events & DOOR_ONCEPSWD_EVT)                                              // ����һ��ִ��һ��
  {
//    osal_ConvertUTCTime(&UTCTimeSys, osal_getClock());
//    uint32 currenTime = osal_ConvertUTCSecs(&UTCTimeSys);
    DsGet_Time();
    uint32 currenTime = Ds1302_ConverUTCSecs(&systime);
    
    for(uint8 i=0;i<10;i++)
    {
   
      if(currenTime >=( sys_24c64time.Once_DayLimit[i][0]+86400))
      {
        sys_24c64time.Once_DayLimit[i][0] = 0x00;
        sys_24c64.Once_OneDay[i][0] = 0x00;
        osal_memcpyz(&sys_24c64.Once_OneDay[i][1], 0xBB, 6);  			//������һ�����Զ����һ��Ȩ�޵�һ��������
      }
      if(currenTime >=( sys_24c64time.Once_WeekLimit[i][0]+604800))
      {
        sys_24c64time.Once_WeekLimit[i][0] = 0x00;
        sys_24c64.Once_OneWeek[i][0] = 0x00;
        osal_memcpyz(&sys_24c64.Once_OneWeek[i][1], 0xBB, 6);  		        //������һ�����Զ����һ����Ȩ�޵�һ��������
      }
      if(currenTime >=( sys_24c64time.Once_MonthLimit[i][0]+2592000))
      {
        sys_24c64time.Once_MonthLimit[i][0]= 0x00;
        sys_24c64.Once_AMonth[i][0] = 0x00;
        osal_memcpyz(&sys_24c64.Once_AMonth[i][1], 0xBB, 6);  			//������һ�����Զ����һ����Ȩ�޵�һ��������
      }
    }
    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ONCEPSWD_EVT);
    return (events ^ DOOR_ONCEPSWD_EVT);
  }

// ���������� 
  if ( events & DOOR_OPENDOOR_EVT)
  {

    if( global.batteryNum > 0)
    {
      osal_stop_timerEx(bleDoorLock_TaskID, DOOR_BATTERY_EVT);
    }
    if((global.openDoorTurn == 0)||(global.openDoorTurn == 3))
    {
      global.openDoorTurn ++;
      Option_Door = !Option_Door;
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT, 10);
    }
    
    else if(global.openDoorTurn == 1)
    {

      global.openDoorTurn ++;
      Motor_Foreward();

      osal_start_timerEx(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT, 500);
      osal_stop_timerEx(bleDoorLock_TaskID, DOOR_PERIODIC_EVT);
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT, 10000);
    }
    else if(global.openDoorTurn ==4)
    {

      global.openDoorTurn ++;
      Motor_Backward();
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT, 500);

    }
    else if(global.openDoorTurn == 6)
    {

      global.openDoorTurn = 0;
      DsGet_Time();

      global.sys_UpdataStatus |= 0x50;
      
      if(sys_24c64.Open_Count >=16)
      {
        sys_24c64.Open_Count = 15;
        sys_24c64.open_DoorRec[0] = sys_24c64.Open_Count;
        Array_Stack();
      }
      sys_24c64.Open_Recode[sys_24c64.open_DoorRec[0]-1][0] = sys_24c64.open_DoorRec[0];
      sys_24c64.Open_Recode[sys_24c64.open_DoorRec[0]-1][1] = sys_24c64.open_DoorRec[1];
      sys_24c64.Open_Recode[sys_24c64.open_DoorRec[0]-1][2] = sys_24c64.open_DoorRec[2];
      
      sys_24c64.Open_Recode[sys_24c64.open_DoorRec[0]-1][3] = systime.cYear;
      sys_24c64.Open_Recode[sys_24c64.open_DoorRec[0]-1][4] = systime.cMon;
      sys_24c64.Open_Recode[sys_24c64.open_DoorRec[0]-1][5] = systime.cDay;
      sys_24c64.Open_Recode[sys_24c64.open_DoorRec[0]-1][6] = systime.cHour;
      sys_24c64.Open_Recode[sys_24c64.open_DoorRec[0]-1][7] = systime.cMin;
      sys_24c64.Open_Recode[sys_24c64.open_DoorRec[0]-1][8] = systime.cSec;
      
      sys_24c64.Open_Count ++;
      sys_24c64.open_DoorRec[0] = sys_24c64.Open_Count;                                                         //�����ż�¼��־
      
      global.safety_Flag = 0;
      osal_stop_timerEx(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_PERIODIC_EVT, 100);
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT, 5000);       //���ź�7�����޲�������˯��״̬
      if( global.batteryNum > 0)
      {
        osal_set_event(bleDoorLock_TaskID, DOOR_BATTERY_EVT);
      }
    }
    else if( global.openDoorTurn ==2 )
    {
      global.openDoorTurn ++;
      Motor_Stop();
      send_line(0x1F);
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT, 5000);
    }
    else if(global.openDoorTurn == 5)
    {
      global.openDoorTurn ++;
      Motor_Stop();
//      osal_start_timerEx(bleDoorLock_TaskID, DOOR_PERIODIC_EVT, 100);
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT, 500);
      
    }

    return (events ^ DOOR_OPENDOOR_EVT);
  }
  
// ��Կ��֤����
  if ( events & DOOR_ENCRYPT_EVT )
  {
//    if(gapProfileState == GAPROLE_CONNECTED)
    {
      //advertising enable 
      uint8 initial_advertising_enable = TRUE;

      // Terminate all connection
      VOID GAPRole_TerminateConnection();

       // adverty state
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );  

      GAP_UpdateAdvertisingData( bleDoorLock_TaskID, FALSE, sizeof(attDeviceName), attDeviceName );
      
//      osal_start_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT, 10 ); 
    } 

    return ( events ^ DOOR_ENCRYPT_EVT );
  }
  
  //���𱨾�
  if( events & DOOR_SAFTYALARM_EVT )
  {
    
    
    if(!ALARM)
    {
      send_line(00);
      osal_stop_timerEx(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT);
      osal_start_timerEx( bleDoorLock_TaskID, DOOR_SAFTYALARM_EVT,200 );
    }
    else
    {
      osal_stop_timerEx( bleDoorLock_TaskID, DOOR_SAFTYALARM_EVT);
      osal_start_timerEx( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT , 10000);
    }
    return ( events ^ DOOR_SAFTYALARM_EVT );
  }
  
  //�ָ���������
  if( events & DOOR_INITCONF_EVT )
  {
    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_INITCONF_EVT);
    if(!SYS_INIT)
    {
      uint8 test[1] = {0x00};
      send_line(0x01);
//      for(uint16 i=0;i<2000;i++)
//        for(uint8 j=0;j<100;j++);
      Init_GlobleVar();
      Init_Sys24c64();
      Init_Sys24c64Time();
      Init_SysConf();
//      sys_24c64.At24c64_WR = 0;
      HAL_DISABLE_INTERRUPTS(); 
      osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
      
      AT24C64_I2C_Write(0x0000, sizeof(SYS_24C64), &sys_24c64);
     
      AT24C64_I2C_Write(0x1000, sizeof(SYS_24C64TIME), &sys_24c64time);
      
      AT24C64_I2C_Write(0x1FF0, 1, test);
      AT24C64_I2C_Write(0x1FFE, 1, test);
      AT24C64_I2C_Write(0x1FFF, 1, test);
      
      HAL_ENABLE_INTERRUPTS();
      
      HAL_SYSTEM_RESET();
    }
    
    
//    return ( events ^ DOOR_INITCONF_EVT );
  }

  
  //��ȡTM ����Ϣ
  if( events & DOOR_READCAR_EVT )
  {
    uint8 * pt = osal_mem_alloc(8);

    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_OUTTIME_EVT);

    if(Read1990a(pt)) 
    {
      for(uint8 j=0;j<6;j++)
      {
        if(pt[j+1] == 0)
        {
            pt[j+1] = 0x30;
        }
      }
      if(global.menu_Num == 5)            //  ��ȡTM ���ɹ���ȷ������밴 "#" �ż�
      {
        for(uint8 i=0;i<10;i++)
        {
         if(osal_memcmpn(&sys_24c64.Tm_User[i][0],&pt[1],6))
         {
            send_line(0x10);  // �����û�����
            global.menu_Index = 0;
            osal_memcpyz(global.MvInPswd, 0xFF, 10);
            global.menu_Num = 1;
            osal_stop_timerEx(bleDoorLock_TaskID, DOOR_READCAR_EVT);
            return ( events ^ DOOR_READCAR_EVT );
         }
        }
        send_line(0x13);// ���TM ��ģʽ����֪ͨ�Ƿ�ȷ�����         
      }
      else if(global.menu_Num == 6)
      {
        for(uint8 i=0;i<10;i++)
        {
         if(osal_memcmpn(&sys_24c64.Tm_User[i][0],&pt[1],6))  //�����ڿ�ɾ��
         {
            
            global.menu_Index = 0;
            send_line(0x16);// ɾ��TM ��ģʽ����֪ͨ�Ƿ�ȷ��ɾ��
//            osal_stop_timerEx(bleDoorLock_TaskID, DOOR_READCAR_EVT);
//            return ( events ^ DOOR_READCAR_EVT );
            break;
         }
         else
          {
            if(i ==9)
            {
              send_line(0x17);  // �����û�������
              global.menu_Index = 0;
              osal_stop_timerEx(bleDoorLock_TaskID, DOOR_READCAR_EVT);
              return ( events ^ DOOR_READCAR_EVT );
            }
         }
        }
        
      }
      
      osal_memcpy(global.tmRecode_Temp , &pt[1], 6); 
      osal_stop_timerEx(bleDoorLock_TaskID, DOOR_READCAR_EVT);
      return ( events ^ DOOR_READCAR_EVT );
    }
    else
    {
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_READCAR_EVT,500);
    }
    osal_mem_free(pt);
    return ( events ^ DOOR_READCAR_EVT );
  }

  
  //keys option timeout
  if( events & DOOR_OUTTIME_EVT )
  {
    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_OUTTIME_EVT);

    global.menu_Num = 1;
    send_line(0x21);
    //������ʱ�������˵�
    osal_memcpyz(global.MvInPswd, 0xFF, 10);

    global.menu_Index = 0;
    return ( events ^ DOOR_OUTTIME_EVT );
  }
  
  
  if( events & DOOR_BATTERY_EVT )
  {
    uint8 temp = 0;
    
    global.batteryNum ++ ;
    if( ( global.batteryNum > 6 ) && ( global.batteryNum <= 11 ) )
    {
      temp = osal_array(global.batteryPower);
      Adc_Stop();
      if(temp <= 10)
      {
//        osal_stop_timerEx(bleDoorLock_TaskID, DOOR_PERIODIC_EVT);
        send_line(0x25);
        osal_start_timerEx(bleDoorLock_TaskID, DOOR_BATTERY_EVT,800);
      }
      else
      {
        global.batteryNum = 0;
        osal_memcpyz(global.batteryPower , 0, 5);
        osal_stop_timerEx(bleDoorLock_TaskID, DOOR_BATTERY_EVT);
        return ( events ^ DOOR_BATTERY_EVT );
      }
      
    }
    else if( global.batteryNum <= 6 && global.batteryNum > 0)
    {
      Adc_Start();
      SendOrGetBatvalue(0);

//      if( global.batteryPer > 0)   //��һ�ζ���Ҫ
      { 
        if(global.batteryNum > 1)
        {
          global.batteryPower[global.batteryNum-2] = global.batteryPer;
        }
      }
#if 0      
      else
      {
        if(global.batteryNum > 0)
        {
          global.batteryNum -- ;
        }
        
        global.batteryNumOut ++ ;
        if(global.batteryNumOut >= 10)
        {
          global.batteryNumOut = 0;
          global.batteryNum = 0;
          Adc_Stop();
          osal_memcpyz(global.batteryPower , 0, 5);
          osal_stop_timerEx(bleDoorLock_TaskID, DOOR_BATTERY_EVT);
          return ( events ^ DOOR_BATTERY_EVT );
        }
        
      }
#endif
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_BATTERY_EVT,100);
      
    }
    else if(global.batteryNum > 11)
    {
      global.batteryNum = 0;
//      Adc_Stop();
      osal_memcpyz(global.batteryPower , 0, 5);
      osal_stop_timerEx(bleDoorLock_TaskID, DOOR_BATTERY_EVT);
//      osal_set_event(bleDoorLock_TaskID, DOOR_PERIODIC_EVT);
      return ( events ^ DOOR_BATTERY_EVT );
    }
    
    return ( events ^ DOOR_BATTERY_EVT );
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

static void bleDoorLock_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
 // #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      bleDoorLock_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
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
static void bleDoorLock_HandleKeys( uint8 shift, uint8 keys )
{
  VOID shift;
  
//  static uint8 addPhoneMode[] = {0x01, 0x4F, 0x4B};    //֪ͨ�������ģʽ
//  static uint8 outPhoneMode[] = {0x01, 0x4F, 0x4F};    //֪ͨ�˳����ģʽ
  
  static uint8 Button_Num = 0xFF;
//  global.addPswdNum = 0;
  
  
  if( keys == HAL_KEY_SW_1 )
  {
    if(Power_OnOff == PowerOn && global.openDoorTurn == 0)
    {
      
      Button_Num =  ReadButtonStatus(); 
      
      if(Button_Num != 0xFF)
      {
        send_line(0x00);
        global.MvInPswd[global.menu_Index] = Button_Num;
        global.menu_Index++;
        if(!global.longButtonFlag)
        {
          osal_stop_timerEx(bleDoorLock_TaskID, DOOR_PERIODIC_EVT);
        }
        if( global.batteryNum > 0 )
        {
          osal_stop_timerEx(bleDoorLock_TaskID, DOOR_BATTERY_EVT);
        }
//        osal_start_timerEx(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT, 15000);     //  10���޲�������˯��
      }
      else if(Button_Num == 0xFF && !global.longButtonFlag)
      {
        osal_start_timerEx(bleDoorLock_TaskID, DOOR_PERIODIC_EVT, 3000);
      }
      else if( Button_Num == 0xFF )
      {
        if( global.batteryNum > 0 )
        {
          osal_set_event(bleDoorLock_TaskID, DOOR_BATTERY_EVT);
        }
      }
      
      if(global.menu_Num >1)
      {
        osal_stop_timerEx(bleDoorLock_TaskID, DOOR_OUTTIME_EVT);
      }  
      
      if(Button_Num == 0x0A )            // �˴��ж��Ƿ��¡�#���ż�
      {
        osal_start_timerEx(bleDoorLock_TaskID, DOOR_ADMININ_EVT, 2000);     // �Ƿ񳤰���#�����ǣ��������Աģʽ   ����
      }
      else 
      {
        osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMININ_EVT);                //  �̰�
      }
    }
   
    
    if(Power_OnOff == PowerOn) 			//���ڻ���״̬�²�������ʵ������
    {                                      
      if(Button_Num != 0xFF && global.openDoorTurn == 0)
      {  
          if(global.longButtonFlag)  //  �ڹ���Աģʽ�¶�ȡ����������ֵ
          {
            osal_start_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT, 5000);		//  3���ڰ�����"#" �ż�ȷ�ϣ��Զ�����Ƚ�
            
            switch(global.menu_Num)
            {
            case 0:                                                                     
              if(global.MvInPswd[global.menu_Index-1] == 0x0B || global.menu_Index >=10)               //����յ�"#" ����Ƚ�
              {
//                global.menu_Index = 0;
                osal_set_event(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
              }
              break;
              
            case 1:// ����һ������   
              if(global.menu_Index == 2 &&  Button_Num == 0x0B) //����յ�"#" �����Ѿ��������ΰ����Զ�����Ƚ�  global.MvInPswd[global.menu_Index-1] == 0x0B || 
              {
//                global.menu_Index = 0;
                osal_set_event(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
              }
              else if( global.menu_Index >= 3 )
              {
                send_line(0x30);
//                global.menu_Index = 0;
                global.menu_Num = 1;
                osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
              }
              break;
              
            case 2:
             {
              uint8 * addPhoneMode = osal_mem_alloc(3);
              uint8 * outPhoneMode = osal_mem_alloc(3);
              addPhoneMode[0] = 0x01;
              addPhoneMode[1] = 0x4F;
              addPhoneMode[2] = 0x4B;
              outPhoneMode[0] = 0x01;
              outPhoneMode[1] = 0x4F;
              outPhoneMode[2] = 0x4F;
              if( global.menu_Index >= 2 && Button_Num == 0x0B)                 // >=3 Ϊ���������û�Ԥ��
              {
                if(global.menu_Index == 3)
                {
                  global.addPhoneNum = BUTTON_UINT8(global.MvInPswd[0],global.MvInPswd[1]);
                }
                else if(global.menu_Index == 2)
                {
                  global.addPhoneNum = global.MvInPswd[0] - 0x30;
                }
                if(sys_config.Phone_User[global.addPhoneNum][0] == 0xFF)
                { 
                  global.AddPhoneFlag = 1;
                  uint8 initial_advertising_enable;

          
                  initial_advertising_enable = TRUE;                
                  // �Ͽ���������
                  GAPRole_TerminateConnection();
                   // ���������㲥״̬
                  GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
//                  sys_config.Phone_User[global.addPhoneNum][0] = global.addPhoneNum;                                                            
//                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 3, addPhoneMode);	    // ֪ͨAPP �������ģʽ
                               

//                  osal_set_event(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
//                  osal_start_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT,500);
//                  osal_start_timerEx(bleDoorLock_TaskID, DOOR_OUTTIME_EVT, 4000);
                  osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
                  
                }
                else                                                            // ������û��Ѵ��� ����������
                {
                  send_line(0x06);
                  global.addPhoneNum = 0xFF;
                  global.AddPhoneFlag = 0;
                  osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
//                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 3, outPhoneMode);	    // ֪ͨAPP �˳����ģʽ
                  
                }
                global.menu_Index = 0;
                
              }
              else if(global.menu_Index >= 4 || (global.menu_Index == 3 && Button_Num != 0x0B))
              {
                send_line(0x30);
                global.menu_Index = 0;
                global.addPhoneNum = 0xFF;
                global.menu_Num = 1;
                global.AddPhoneFlag = 0;
//                osal_memcpyz(global.MvInPswd, 0xFF, 10);
                osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 3, outPhoneMode);	    // ֪ͨAPP �˳����ģʽ
              }
              osal_mem_free( addPhoneMode );
              osal_mem_free( outPhoneMode );
             }
             break;
              
            case 3:                                                             // �޸Ĺ���Ա����
              if(global.menu_Index == 6 && global.keyRecode_Temp[5] == 0xBB)   // �޶�������������붼Ϊ6λ����������λ
              {
                osal_memcpy(global.keyRecode_Temp, global.MvInPswd, 6);
                osal_memcpyz(global.MvInPswd, 0xFF, 10);
                global.menu_Index = 0;
                send_line(0x0A);
              }
              else if( global.menu_Index == 6 && global.keyRecode_Temp[5] != 0xBB)
              {
                if(osal_memcmpn(global.keyRecode_Temp, global.MvInPswd, 6))
                {
                  
                  osal_set_event(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
                }
                else
                {
                  send_line(0x30);
                  osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
                }
                global.menu_Index = 0;
              }
              break;
              
              case 4:                                                          // �����ͨ�����û�����
                if( global.menu_Index >= 2 && Button_Num == 0x0B)
                {
                  
                  if(global.menu_Index == 3)
                  {
                    global.addPswdNum = BUTTON_UINT8(global.MvInPswd[0],global.MvInPswd[1]);
                  }
                  else if(global.menu_Index == 2)
                  {
                    global.addPswdNum = global.MvInPswd[0] - 0x30;
                  }
//                  global.addPswdNum = global.MvInPswd[2];   //��ȡ���
//                  osal_memcpyz(global.MvInPswd, 0xFF, 10);
//                  global.menu_Index = 0;
                  if(sys_24c64.General_Pswd[global.addPswdNum][0] !=0xBB )
                  {
                    send_line(0x10);    // ������û��Ѵ��� ����������
                    global.addPswdNum = 0xFF;
                    global.menu_Num = 1;
                    
                  }
                  else
                  {
                    send_line(0x0E);
                    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
                  }
                  global.menu_Index = 0;
//                  osal_memcpyz(global.MvInPswd, 0xFF, 10);
                }
                else if(global.menu_Index > 3 && global.addPswdNum == 0xFF)
                {
                  send_line(0x30);
//                  global.addPswdNum = 0xFF;
                  global.menu_Num = 1;
                  global.menu_Index = 0;
//                  osal_memcpyz(global.MvInPswd, 0xFF, 10);
                  osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
                }
                // �޶�������������붼Ϊ6λ����������λ
                else if( global.menu_Index == 6 && global.keyRecode_Temp[5] == 0xBB && global.addPswdNum != 0xFF )   
                {
                  osal_memcpy(global.keyRecode_Temp, global.MvInPswd, 6);
                  osal_memcpyz(global.MvInPswd, 0xFF, 10);
                  global.menu_Index = 0;
                  send_line(0x0F);
                  
                }
                else if(global.menu_Index == 6 && global.keyRecode_Temp[5] != 0xBB && global.addPswdNum != 0xFF)
                {
                  if(osal_memcmp(global.keyRecode_Temp, global.MvInPswd, 6))
                  {
                    
                    osal_set_event(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
                  }
                  else
                  {
                    send_line(0x30);

                    global.addPswdNum = 0xFF;

                    osal_memcpyz(global.keyRecode_Temp, 0xBB, 6);

                    global.menu_Num = 1;

                    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
                  }
                  global.menu_Index = 0;
                }
                
                
              break;
              
            case 5:                                                             // ���TM ��
            {
              uint8 test[6] = {0xBB,0xBB,0xBB,0xBB,0xBB,0xBB};
              if(Button_Num == 0x0B && global.menu_Index == 1 )                                            // "#"  ȷ�����
              {
                if(osal_memcmpn(global.tmRecode_Temp, test, 6))
                {
                  send_line(0x12);
                  global.menu_Index = 0;
                }
                else
                {
                  osal_set_event(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
                }

              }
              else if(global.menu_Index > 1 || (global.menu_Index == 1 && Button_Num != 0x0B))
              {
                send_line(0x30);
                global.menu_Num = 1;
                global.menu_Index = 0;
//                osal_memcpyz(global.MvInPswd, 0xFF, 10);
                osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
              }
             }
              break;
              
            case 6:                                                             // ɾ�� TM ��
//              if( Button_Num == 0x0A && global.menu_Index == 2)                                       
//              if(global.MvInPswd[0] == 0x0B)
            {
              uint8 test[6] = {0xBB,0xBB,0xBB,0xBB,0xBB,0xBB};
              if(global.menu_Index == 2 )//global.MvInPswd[0] == 0x0A && global.MvInPswd[1] == 0x0A && 
              {
                if(osal_memcmpn(global.tmRecode_Temp, test, 6))
                {
                  send_line(0x12);
                  global.menu_Index = 0;
                }
                else
                {
                  osal_set_event(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
                }

              }
              else if(global.menu_Index > 2 )
              {
                send_line(0x30);
                global.menu_Num = 1;
                global.menu_Index = 0;
                osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
              }
            }
              break;
              
            case 7:                                                             // ɾ����ͨ�û�
              if( Button_Num == 0x0B && global.menu_Index >= 2)
//              if(global.MvInPswd[0] == 0x0B)
              {
                if(global.menu_Index == 3)
                {
                  global.addPswdNum = BUTTON_UINT8(global.MvInPswd[0],global.MvInPswd[1]);
                }
                if(global.menu_Index == 2)
                {
                  global.addPswdNum = global.MvInPswd[0] - 0x30;
                }
//                  global.addPswdNum = global.MvInPswd[2];   //��ȡ���
                if(sys_24c64.General_Pswd[global.addPswdNum][0] == 0xBB)
                {
                  send_line(0x1C);  // ������û��Ѳ����� ����������
                  global.addPswdNum = 0xFF;
                  global.menu_Num = 1;

                }
                else
                {
                  send_line(0x1A);
                }
                global.menu_Index = 0;
                osal_memcpyz(global.MvInPswd, 0xFF, 10);
              }
              else if( global.menu_Index > 3 )
              {
                send_line(0x30);
                global.menu_Num = 1;
                global.menu_Index = 0;
                osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
              }
              else if( Button_Num == 0x0B && global.menu_Index == 1)
              {                
                global.menu_Index = 0;
                osal_set_event(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
              }
              
              break;
              
            case 8:                                                             // ɾ���ֻ��û�
              if( global.menu_Index >= 2 && Button_Num == 0x0B)
//              if(global.MvInPswd[0] == 0x0B)
              {
                if(global.menu_Index == 3)
                {
                  global.addPhoneNum = BUTTON_UINT8(global.MvInPswd[0],global.MvInPswd[1]);
                }
                else if(global.menu_Index == 2)
                {
                  global.addPhoneNum = global.MvInPswd[0] - 0x30;
                }
//                  global.addPswdNum = global.MvInPswd[2];   //��ȡ���
                if((0xFF == sys_config.Phone_User[global.addPhoneNum][0])||(global.addPhoneNum == 0)) //  ������Ų���Ϊ0
                {
                  send_line(0x1C);  // ������ֻ��û������� ����������
                  global.addPhoneNum= 0xFF;
                  
                  global.menu_Num = 1;
                  
                }
                else
                {
//                  send_line(0x1E);
//                  global.menu_Index = 0;
//                  osal_memcpyz(global.MvInPswd, 0xFF, 10);
                }
//                osal_memcpyz(global.MvInPswd, 0xFF, 10);
                global.menu_Index = 0;
              }
              else if(global.menu_Index > 3)
              {
                send_line(0x30);
                global.menu_Num = 1;
                global.menu_Index = 0;
                osal_stop_timerEx(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
              }
              else if( global.menu_Index == 1 && Button_Num == 0x0B)
              {
                osal_set_event(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
              }
              break;
              
            default:
              break;
            }
            
          }
          else
          {
            if((global.MvInPswd[global.menu_Index-1] == 0x0B) || (global.menu_Index >=10))                   //����յ�"#" ����Ƚ�
            {
              global.menu_Index = 0;
              global.genPswdFlag = 1;   
              osal_set_event(bleDoorLock_TaskID, DOOR_PSWD_OPEN_EVT);
            }
            else if(global.menu_Index >=6)
            {
              osal_start_timerEx(bleDoorLock_TaskID, DOOR_PSWD_OPEN_EVT,3000);             //  ������ѭ��
            }
          }
      }
    }
    else									//�ǻ���״̬����뻽������״̬ת��
    {
      uint32 res = 0;
//      osal_memcpyz(global.MvInPswd, 0xFF, 10);
      global.menu_Index = 0;

      if(global.safety_Flag >= 6)                                           // �Ƿ��Ұ�
      {
        DsGet_Time();
        res = Ds1302_ConverUTCSecs(&systime);
        if(res >= global.time_Recode + 600)                                 // 10���ӱ�־ ʱ�䵽�Ϳ��Ի��ѷ���������
        {
          global.safety_Flag = 0;
          global.time_Recode = 0;
          osal_start_timerEx(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT,50);
        }
        else
        {
          return;
        }
      }
      else
      {
//        global.safety_Flag = 0;
        osal_set_event(bleDoorLock_TaskID, DOOR_POWERONOFF_EVT);
      }
    }
  }
  if( keys == HAL_KEY_SW_2)             // ���𱨾�
  {
    if(Power_OnOff == PowerOff)         // �Ȼ���
    {
      osal_set_event( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT );
    }
    osal_set_event( bleDoorLock_TaskID, DOOR_SAFTYALARM_EVT );
  }
  else
  {
    osal_stop_timerEx( bleDoorLock_TaskID, DOOR_SAFTYALARM_EVT );
  }
  if( keys == HAL_KEY_SW_3)             // �����ָ���������
  {
    if(Power_OnOff == PowerOff)         // �Ȼ���
    {
      osal_set_event( bleDoorLock_TaskID, DOOR_POWERONOFF_EVT );
//      osal_start_timerEx(bleDoorLock_TaskID, DOOR_INITCONF_EVT, 3000);
    }
//    else
    {
      osal_start_timerEx(bleDoorLock_TaskID, DOOR_INITCONF_EVT, 3000);
    }
  }
  else
  {
    osal_stop_timerEx(bleDoorLock_TaskID, DOOR_INITCONF_EVT);
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
static void doorStateNotificationCB( gaprole_States_t newState )
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

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        global.authoryMode = 0;
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        osal_stop_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT ); 
      }
      break;

    case GAPROLE_CONNECTED:
      {        
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
//        if ( first_conn_flag == 0 && global.authoryMode==1)   //����ʱ���ϴ�һ�� ����Ա�ֻ�  
//        if ( first_conn_flag == 0)   //����ʱ���ϴ�һ�� �ֻ�
//        {
//          first_conn_flag = 1;
//          Adc_Start();
//          SendOrGetBatvalue(1);  // 1: send  0:get
//          Adc_Stop();
//        }
#if 0        
        if(global.AddPhoneFlag == 1)        //���ģʽ�¹ر���֤
        {
          uint8 * addPhoneMode = osal_mem_alloc(3);
          addPhoneMode[0] = 0x01;
          addPhoneMode[1] = 0x4F;
          addPhoneMode[2] = 0x4B;
          DoorProfile_SetParameter(DOORPROFILE_CHAR2, 3, addPhoneMode);	    // ֪ͨAPP �������ģʽ
          osal_set_event(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
          osal_stop_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT);                 //
          osal_mem_free(addPhoneMode);
        }
        if(!global.authoryMode && global.AddPhoneFlag == 0)       // ��Ȩ�޶Ͽ�����
        {
          osal_start_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT, 100 ); 
        }
        else      // ��Ȩ�޺�ر���֤
        {
          osal_stop_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT);                 //
        }
          
      }
#else
        if(global.AddPhoneFlag == 1)        //���ģʽ�¹ر���֤
        {
          uint8 * addPhoneMode = osal_mem_alloc(3);
          addPhoneMode[0] = 0x01;
          addPhoneMode[1] = 0x4F;
          addPhoneMode[2] = 0x4B;
          DoorProfile_SetParameter(DOORPROFILE_CHAR2, 3, addPhoneMode);	    // ֪ͨAPP �������ģʽ
//          osal_set_event(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
          osal_stop_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT);                 //
        }
      }
#endif
      break;
/*
    case GAPROLE_CONNECTED_ADV:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break; 
 */     
    case GAPROLE_WAITING:
      {
        global.authoryMode = 0;
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          
        // Reset flag for next connection.
//        first_conn_flag = 0;

      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
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

//  static uint8 openDoor[] = {0x1F, 0x4B, 0x4F};
#if 1  

  uint8 * pt= osal_mem_alloc(8);

//  HAL_DISABLE_INTERRUPTS();
  if(Read1990a(pt))                                                             //��Ibutton  TM��ģʽ����                                   
  {
   
    for(uint8 j=0;j<6;j++)
    {
      if(pt[j+1] == 0)
      {
          pt[j+1] = 0x30;
      }
    }
//    if(!global.longButtonFlag)
    {
      for(uint8 i=0;i<10;i++)
      {
        if(osal_memcmpn(&sys_24c64.Tm_User[i][0],&pt[1],6))    //������TM�����Ƚ��Ƿ���Ȩ��
        {
          TmLed_True();
          sys_24c64.open_DoorRec[1] = 3;
          sys_24c64.open_DoorRec[2] = i;
          osal_set_event(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);
//          DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,openDoor);		         //     ֪ͨAPP  ���ѿ�
          break;
        }
        else
        {
          if(i == 9)
          {
            TmLed_False();
            send_line(0x01);
          }
        }
      }
    } 
//    osal_mem_free( openDoor );
  }
  osal_mem_free(pt);
//  HAL_ENABLE_INTERRUPTS();
#endif  
  
  if(global.authoryMode != 0)                                                   //     ��Ȩ�޿���
  {
    uint8 * openDoor = osal_mem_alloc(3);
    openDoor[0] = 0x1F;
    openDoor[1] = 0x4B;
    openDoor[2] = 0x4F;
    if(global.safety_Rssi[0] <= 65 )// global.safety_Rssi[1])				//     ֻ�а�ȫ�����²ſ��Կ���
    {
      global.authoryMode = 0;
      osal_set_event(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);
      DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,openDoor);		         //     ֪ͨAPP  ���ѿ�
    }
    osal_mem_free(openDoor);
  }
/*  
  else if(global.authoryMode == 2)                                                   //     
  {
    uint8  * openDoor = osal_mem_alloc(3);
    openDoor[0] = 0x1F;
    openDoor[1] = 0x4B;
    openDoor[2] = 0x4F;
    
    if(global.safety_Rssi[0] <= 65 )//global.safety_Rssi[1])				//     ֻ�а�ȫ�����²ſ��Կ���
    {
      global.authoryMode = 0;
      osal_set_event(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);
      DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,openDoor);		         //     ֪ͨAPP  ���ѿ�
    }
    osal_mem_free(openDoor);
  }
*/ 

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
static void doorProfileChangeCB( uint8 paramID )
{
  static uint8 responseData[DOORPROFILE_CHAR1_LEN]={0};
//  uint8 * responseData = osal_mem_alloc(DOORPROFILE_CHAR1_LEN);
//  static uint8 replyUpdata[4] = {0};
  uint8 * replyUpdata = osal_mem_alloc(4);
//  uint8 timeReq[6];
//  uint8 res[20] = {0x00};
  static uint8 reqStatus[] = {0x4F, 0x4B, 0};

  switch( paramID )
  {
    case DOORPROFILE_CHAR1:
      DoorProfile_GetParameter( DOORPROFILE_CHAR1, responseData );
      osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config);
      switch(responseData[0])
      {
        case 0x01 :
        {
          for(uint8 i=0; i<11; i++)
          {
            if(osal_memcmpn(&sys_config.Phone_User[i][1],&responseData[1],6))           // admin
            {
              Gap_Connect = Gapconnected;
              sys_24c64.open_DoorRec[1] = 2;
              sys_24c64.open_DoorRec[2] = i;
//              osal_stop_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT );   
              if( i == 0 )
              {
                global.authoryMode = 1;
//                replyUpdata[0] = 0x77;
//                replyUpdata[1] = 0x01;
//                replyUpdata[2] = 0x00;
//                DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,replyUpdata);		//     ֪ͨҪ�ϴ��û���Ϣ
                osal_set_event( bleDoorLock_TaskID, DOOR_PHONE_EVT);		//�������Ա
              }
              else
              {
                global.authoryMode = 2;
                osal_set_event( bleDoorLock_TaskID, DOOR_PHONE_EVT);		//������ͨ�ֻ��û�
              }
              Adc_Start();
              SendOrGetBatvalue(1);  // 1: send  0:get
              Adc_Stop();
              if((responseData[7] & 0x0F) != (sys_config.Phone_User[i][7] & 0x0F))       //�ֻ�APP�����˿���ģʽ  ����ģʽ���ı������±���
              {
                sys_config.Phone_User[i][7] = responseData[7];

              }
              if((responseData[7] & 0x0F) == 0x01)                              // open mode  �Զ�
              {
		         global.AppOpenStatus = 1;
              }
              else if((responseData[7] & 0x0F) == 0x02)			// ������Ļ����
              {
                global.AppOpenStatus = 2;
              }
              if((responseData[7] & 0xF0) == 0x00 )                                //APP ��װ
              {
                replyUpdata[0] = 0x21;
                if( i == 0 )
                {
                  replyUpdata[1] = 0x10;                                                 // ����ԱȨ��
                }
                else
                {
                  replyUpdata[1] = 0x20;                                                 // ��ͨȨ��
                } 
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
              }
              
              break;
            }
            
            else
            {
//              Gap_Connect = Gapconnecting;
              if(i == 10)
              {
                if( global.AddPhoneFlag == 1)
                {
                   osal_memcpy(global.appRecode_Temp, &responseData[1], 7);				// ���ûȨ�޵��ֻ�����ʱ�洢8�ֽ�Ӧ������
                   osal_set_event(bleDoorLock_TaskID, DOOR_ADMIN_PRO_EVT);
                }
//                osal_start_timerEx( bleDoorLock_TaskID, DOOR_ENCRYPT_EVT ,100);
              }
            }

          }

        }

        break;
        
        case 0x02:
          {
            static uint8 varUpdata[] = {0, 0x53, 0x4A};
            static uint8 falseReq[] = {0, 0x43, 0x57};
            static uint8 sysUpdata_temp = 0;
            sysUpdata_temp = global.sys_UpdataStatus;
            uint8 test[3] = {0x00};
            reqStatus[2] = global.appUpdata_CRC;
//            reqStatus[2] = 0x55;
            test[0] = global.authoryMode;
            test[1] = global.appUpdata_CRC;
            test[2] = global.sys_UpdataStatus;
            DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,test);		
            if(osal_memcmpn(reqStatus, &responseData[1], 3) && global.authoryMode == 1)
            {
              global.appUpdata_CRC = 0;
              for(uint8 i=0;i<7;i++)
              {
                if(sysUpdata_temp&0x01)
                {
                  switch(i)
                  {
                  case 0x00:                                                      //  �ϴ��ֻ�       
                    varUpdata[0] = 0x41;
                    global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);
                    DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,varUpdata);		
                    return;
                  case 0x01:                                                      //  ����Ա����
                    varUpdata[0] = 0x42;
                    global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);
                    DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,varUpdata);
                    return;
                  case 0x02:                                                      //  ��ͨ��������
                    varUpdata[0] = 0x43;
                    global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);
                    DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,varUpdata);
                    return;
                  case 0x03:                                                      //  TM ����Ϣ        
                    varUpdata[0] = 0x44;
                    global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);
                    DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,varUpdata);
                    return;
                  case 0x04:                                                      // ���ż�¼
                    varUpdata[0] = 0x45;
                    global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);
                    DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,varUpdata);
                    return;
                  case 0x05:                                                      // ����
                    varUpdata[0] = 0x46;
//                    global.sys_OptionFlag = 0;
                    global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);
                    DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,varUpdata);
                    return;
                  case 0x06:                                                      // ����
                    varUpdata[0] = 0x47;
//                    global.sys_OptionFlag = 0;
                    global.appUpdata_CRC = appData_Crc(varUpdata, global.appUpdata_CRC, 3);
                    DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,varUpdata);
                    return;
                  default:
                    break;                  
                  }
                }
                sysUpdata_temp >>= 1;
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
                      case 0x08:
                        falseReq[0] = 0x44;
                      break;
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
                  global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
                }
            }
          }
        
        break;
        
        case 0x03:              //�ϴ��ֻ��û���Ϣ
        {
//          static uint8 replyUpdata[] = {0x02, 0x53, 0x4A,0x00};
          static uint8 falseReq[] = {0, 0x43, 0x57};
//          uint8 test[1] = {0x00};
          if( osal_memcmpn(reqStatus, &responseData[1], 2) && global.authoryMode == 1 )
          {     
            switch(responseData[4])
            {
            case 0x41:                                                          //  �ֻ��û�
            {
              if(global.appUpdata_CRC == responseData[3])
              {
                global.appUpdata_CRC = 0;
                for(uint8 i=1; i<10; i++)
                {
                  global.appUpdata_CRC = appData_Crc(&sys_config.Phone_User[i][0], global.appUpdata_CRC, 8);        //��ȡУ���� CRC
                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 8, &sys_config.Phone_User[i][0]);
                }
//                test[0] = global.appUpdata_CRC;
//                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 1, test);
                global.sys_UpdataStatus &= ~(BV(0)) ;
                global.sys_CurrentUp = 0x01;
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x41;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,falseReq);		//     �ϴ�������Ϣ
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
            }
              break;
              
            case 0x42:                                                          //  ����Ա����
            {
              if(global.appUpdata_CRC == responseData[3])
              {
                global.appUpdata_CRC = 0;
                global.appUpdata_CRC = appData_Crc(sys_config.Admin_Pswd, global.appUpdata_CRC, 6);               //��ȡУ���� CRC
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 6, sys_config.Admin_Pswd);
                global.sys_CurrentUp = 0x02;
//                test[0] = global.appUpdata_CRC;
//                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 1, test);
                global.sys_UpdataStatus &= ~(BV(1)) ;
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x42;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,falseReq);		//     �ϴ�������Ϣ
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
            }
              break;
              
            case 0x43:                                                          //  ��ͨ��������
            {
              if(global.appUpdata_CRC == responseData[3])
              {
                global.appUpdata_CRC = 0;
                for(uint8 i=0; i<10; i++)
                {
                  global.appUpdata_CRC = appData_Crc(&sys_24c64.General_Pswd[i][0], global.appUpdata_CRC, 7);               //��ȡУ���� CRC
                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 7, &sys_24c64.General_Pswd[i][0]);
                }
//                test[0] = global.appUpdata_CRC;
//                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 1, test);
                global.sys_UpdataStatus &= ~(BV(2)) ;
                global.sys_CurrentUp = 0x04;
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x43;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,falseReq);		//     �ϴ�������Ϣ
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
            }
              break;
               
            case 0x44:                                                          //  TM ����Ϣ
            {
              if(global.appUpdata_CRC == responseData[3])
              {
                global.appUpdata_CRC = 0;
                for(uint8 i=0; i<10; i++)
                {
                  global.appUpdata_CRC = appData_Crc(&sys_24c64.Tm_User[i][0], global.appUpdata_CRC, 6);               //��ȡУ���� CRC
                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 6, &sys_24c64.Tm_User[i][0]);

                }
//                test[0] = global.appUpdata_CRC;
//                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 1, test);
                global.sys_UpdataStatus &= ~(BV(3)) ;
                global.sys_CurrentUp = 0x08;
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x44;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,falseReq);		//     �ϴ�������Ϣ
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
            }
              break;
              
            case 0x45:                                                          // ���ż�¼
            {
              if(global.appUpdata_CRC == responseData[3])
              {
                global.appUpdata_CRC = 0;
                for(uint8 i=0; i<15; i++)
                {
                  global.appUpdata_CRC = appData_Crc(&sys_24c64.Open_Recode[i][0], global.appUpdata_CRC, 9);               //��ȡУ���� CRC
                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 9, &sys_24c64.Open_Recode[i][0]);
                }
                global.sys_UpdataStatus &= ~(BV(4)) ;
                global.sys_CurrentUp = 0x10;
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x45;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,falseReq);		//     ֪ͨҪ�ϴ��û���Ϣ
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
            }
              break;
            case 0x46:                                                          //  ��ͨ��������
            {
              
              if(global.appUpdata_CRC == responseData[3])
              {
                global.appUpdata_CRC = 0;
                for(uint8 i=0; i<10; i++)
                {
                  global.appUpdata_CRC = appData_Crc(&sys_24c64.Once_Single[i][0], global.appUpdata_CRC, 7);               //��ȡУ���� CRC
                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 7, &sys_24c64.Once_Single[i][0]);
                }
//                test[0] = global.appUpdata_CRC;
//                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 1, test);
                global.sys_UpdataStatus &= ~(BV(5)) ;
                global.sys_CurrentUp = 0x20;
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x46;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,falseReq);		//     �ϴ�������Ϣ
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
            }
              break;  
            case 0x47:   
            {  
              if(global.appUpdata_CRC == responseData[3])
              {
                global.appUpdata_CRC = 0;
                global.sys_UpdataStatus = 0;                                //  �ϴ�����  ���±�־λ�� 0
                global.sys_CurrentUp = 0x00;
                Init_Recode();//�ϴ���������տ��ż�¼�����¼��㡣
              }
              else
              {
                global.appUpdata_CRC = 0;
                falseReq[0] = 0x47;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,falseReq);		//     �ϴ�������Ϣ
                global.appUpdata_CRC = appData_Crc(falseReq, global.appUpdata_CRC, 3);
              }
            }
              break;
              
            default:
              break;
            }
            
          }

        }
        break;
		
        case 0x08:              //�·�ʱ��У׼
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
          DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,replyUpdata); // У׼�ɹ�
//           osal_setClock(timeReq);
        }
        break;
        case 0x10: // �ֻ�ģʽ�ǵ�����Ļ����ʱ
        {
          if((responseData[1]==0x4F)&&(responseData[2] == 0x50))
          {
            if(global.safety_Rssi[0] <= global.safety_Rssi[1])				//     ֻ�а�ȫ�����²ſ��Կ���
            {
              osal_set_event(bleDoorLock_TaskID, DOOR_OPENDOOR_EVT);   // ����
              replyUpdata[0] = 0x1F;
              replyUpdata[1] = 0x4B;
              replyUpdata[2] = 0x4F;
              DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,replyUpdata); // ֪ͨ���Уп�����
            }
          }
          
        }
        break;
/*        
        case 0x11: // ���ֻ��в���ʱ�򲻽���͹��ģ��ȴ��Ͽ�����
        {
          
        }
        break;
*/
        default:
        // should not reach here!
        break;
	  
      }

      break;

    case DOORPROFILE_CHAR4:
//      osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config);
      DsGet_Time();
      global.appDedata_CRC = 0;
      DoorProfile_GetParameter( DOORPROFILE_CHAR4, responseData );
      switch(responseData[0])
      {
        case 0x01 :     // ɾ���ֻ��û�
        {
          global.appDedata_CRC = appData_Crc(&sys_config.Phone_User[responseData[1]][0], global.appDedata_CRC, 8);
          if(global.authoryMode == 1 && global.appDedata_CRC == responseData[2])       //ֻ���ڹ���ԱȨ��ģʽ�²ſ��Բ�������
          {
            global.app_DownChange = 0x01;
            osal_memcpyz(&sys_config.Phone_User[responseData[1]][0], 0, 8);
            sys_config.Phone_User[responseData[1]][0] = 0xFF;
            replyUpdata[0] = 0x03;
            replyUpdata[1] = responseData[1];
            replyUpdata[2] = global.appDedata_CRC;
            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 3, replyUpdata);
            osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
          }
          else
          {
            replyUpdata[0] = 0x50;
            replyUpdata[1] = 0xFF;	           
            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
          }
        }

        break;
        
        case 0x02:      // ɾ����������
        {
          static uint8 Init_Pswd[6] = "123456";
          global.appDedata_CRC = appData_Crc(sys_config.Admin_Pswd, global.appDedata_CRC, 6);
          if(global.authoryMode == 1 && global.appDedata_CRC == responseData[1])       //ֻ���ڹ���ԱȨ��ģʽ�²ſ��Բ�������
          {
            global.app_DownChange = 0x01;
            osal_memcpyz(sys_config.Admin_Pswd, 0, 6);
            osal_memcpy(sys_config.Admin_Pswd, Init_Pswd, 6);						//  ��ɾ����ָ���������
            replyUpdata[0] = 0x04;
            replyUpdata[1] = global.appDedata_CRC;
            DoorProfile_SetParameter(DOORPROFILE_CHAR2,2,replyUpdata);
          }
          else
          {
            replyUpdata[0] = 0x51;
            replyUpdata[1] = 0xFF;	           
            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
          }
        }
        break;

        case 0x03:      // ɾ���û�����
        {
          global.appDedata_CRC = appData_Crc(&sys_24c64.General_Pswd[responseData[1]][0], global.appDedata_CRC, 7);
          if(global.authoryMode == 1 && global.appDedata_CRC == responseData[2])       //ֻ���ڹ���ԱȨ��ģʽ�²ſ��Բ�������
          {
            global.app_DownChange = 0x01;
//            sys_24c64.General_Pswd[responseData[1]][0] = 0x00;
            osal_memcpyz(&sys_24c64.General_Pswd[responseData[1]][0], 0xBB, 7);
            replyUpdata[0] = 0x05;
            replyUpdata[1] = responseData[1];
            replyUpdata[2] = global.appDedata_CRC;
            DoorProfile_SetParameter(DOORPROFILE_CHAR2,3,replyUpdata);
          }
          else
          {
            replyUpdata[0] = 0x52;
            replyUpdata[1] = 0xFF;	           
            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
          }
        }
        break;

        case 0x04:      // ɾ��TM��
        {
          if(global.authoryMode == 1  )       //ֻ���ڹ���ԱȨ��ģʽ�²ſ��Բ�������
          {
            for(uint8 i=0;i<10;i++)
            {
              if(osal_memcmpn(&sys_24c64.Tm_User[i][0], &responseData[1], 6))
              {
                osal_memcpyz(&sys_24c64.Tm_User[i][0], 0xFF, 6);
                replyUpdata[0] = 0x06;
                replyUpdata[1] = 0x4B;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2,2,replyUpdata);
                global.app_DownChange = 0x02;
                return;
              }
              else                                                              // ɾ��ʧ��
              {
                if(i==9)
                {
                  replyUpdata[0] = 0x53;
                  replyUpdata[1] = 0xFF;	           
                  DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
                }
              }
            }
            
          }
        }
        break;
        
        case 0x05:      // �޸Ĺ�������
        {
          global.appDedata_CRC = appData_Crc(&responseData[1], global.appDedata_CRC, 6);
          if(global.authoryMode == 1 && global.appDedata_CRC == responseData[7])       //ֻ���ڹ���ԱȨ��ģʽ�²ſ��Բ�������
//          if(global.appDedata_CRC == responseData[7])
          {
            global.app_DownChange = 0x01;
            osal_memcpy(sys_config.Admin_Pswd, &responseData[1], 6);
            replyUpdata[0] = 0x07;
            replyUpdata[1] = 0x4B;          
            DoorProfile_SetParameter(DOORPROFILE_CHAR2,2,replyUpdata);
          }
          else
          {
            replyUpdata[0] = 0x54;
            replyUpdata[1] = 0xFF;	           
            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
          }
        }
        break;
        
        case 0x06:      // �޸���ͨ����
        {
          global.appDedata_CRC = appData_Crc(&responseData[1], global.appDedata_CRC, 7);
//          if(global.authoryMode == 1 && global.appDedata_CRC == responseData[8])       //ֻ���ڹ���ԱȨ��ģʽ�²ſ��Բ�������
          if(global.appDedata_CRC == responseData[8])
          {
            global.app_DownChange = 0x01;
            osal_memcpy(&sys_24c64.General_Pswd[responseData[1]][0], &responseData[1], 7);
            replyUpdata[0] = 0x08;
            replyUpdata[1] = 0x4B;	   
//            replyUpdata[2] = global.appDedata_CRC;
            DoorProfile_SetParameter(DOORPROFILE_CHAR2,2,replyUpdata);
            AT24C64_I2C_Write(0x0000, sizeof(SYS_24C64), (uint8*)&sys_24c64);  

          }
          else
          {
            replyUpdata[0] = 0x55;
            replyUpdata[1] = 0xFF;	           
            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
          }
        }
        break;
        
        case 0x07:      // �·�һ��������
        {
          
          global.appDedata_CRC = appData_Crc(&responseData[1], global.appDedata_CRC, 8);
          if(global.authoryMode == 1 && global.appDedata_CRC == responseData[9])       //ֻ���ڹ���ԱȨ��ģʽ�²ſ��Բ�������
//          if(global.appDedata_CRC == responseData[9])   //������
          {

            switch(responseData[1])
            {
              case 0x31:		//�������������¼ʱ�䣬�����������
              {
                global.app_DownChange = 0x02;
                sys_24c64.Once_Single[responseData[2]][0] = responseData[2];
                osal_memcpy(&sys_24c64.Once_Single[responseData[2]][1], &responseData[3], 6);
                replyUpdata[0] = 0x09;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
              }
              break;
              
              case 0x32:
              {
                global.app_DownChange = 0x02;
                sys_24c64.Once_OneDay[responseData[2]][0] = responseData[2];
                osal_memcpy(&sys_24c64.Once_OneDay[responseData[2]][1], &responseData[3], 6);
                sys_24c64time.Once_DayLimit[responseData[2]][0] = Ds1302_ConverUTCSecs(&systime);
//                  osal_memcpy(&sys_24c64.Once_DayLimit[responseData[2]][0],&UTCTimeSys,6);	//��¼��ǰʱ��
                replyUpdata[0] = 0x09;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	          
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);                    
              }
              break;
              
              case 0x33:
              {
                global.app_DownChange = 0x02;
                sys_24c64.Once_OneWeek[responseData[2]][0] = responseData[2];
                osal_memcpy(&sys_24c64.Once_OneWeek[responseData[2]][1], &responseData[3], 6);
//                  osal_memcpy(&sys_24c64.Once_WeekLimit[responseData[2]][0],&UTCTimeSys,6);	//��¼��ǰʱ��
                sys_24c64time.Once_WeekLimit[responseData[2]][0] = Ds1302_ConverUTCSecs(&systime);
                replyUpdata[0] = 0x09;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	          
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
              }
              break;
              
              case 0x34:
              {
                global.app_DownChange = 0x02;
                sys_24c64.Once_AMonth[responseData[2]][0] = responseData[2];
                osal_memcpy(&sys_24c64.Once_AMonth[responseData[2]][1], &responseData[3], 6);
//                  osal_memcpy(&sys_24c64.Once_MonthLimit[responseData[2]][0],&UTCTimeSys,6);	//��¼��ǰʱ��
                sys_24c64time.Once_MonthLimit[responseData[2]][0] = Ds1302_ConverUTCSecs(&systime);
                replyUpdata[0] = 0x09;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	          
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
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
            DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
          }
        }
        break;
       case 0x08:																												//  ɾ��һ��������
        {
			
          if(global.authoryMode == 1 )       //ֻ���ڹ���ԱȨ��ģʽ�²ſ��Բ�������
          {

            switch(responseData[1])
            {
            case 0x31:		//�������������¼ʱ�䣬�����������
            {
              global.appDedata_CRC = appData_Crc(&sys_24c64.Once_Single[responseData[2]][0], global.appDedata_CRC, 7);
              if( global.appDedata_CRC == responseData[3] )
              {
                global.app_DownChange = 0x02;
//                sys_24c64.Once_Single[responseData[2]][0] = 0;
                osal_memcpyz(&sys_24c64.Once_Single[responseData[2]][0], 0xBB, 7);		//  ɾ��
                
                replyUpdata[0] = 0x0A;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
              }
              else
              {
                replyUpdata[0] = 0x57;
                replyUpdata[1] = 0xFF;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	          
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
              }
            }
            break;

            case 0x32:
            {
              global.appDedata_CRC = appData_Crc(&sys_24c64.Once_OneDay[responseData[2]][0], global.appDedata_CRC, 7);
              if( global.appDedata_CRC == responseData[3] )
              {
                global.app_DownChange = 0x02;
//                sys_24c64.Once_Single[responseData[2]][0] = 0;
                osal_memcpyz(&sys_24c64.Once_OneDay[responseData[2]][0], 0xBB, 7);			// ɾ��
                sys_24c64time.Once_DayLimit[responseData[2]][0] = 0;
                replyUpdata[0] = 0x0A;
                replyUpdata[1] = 0x4B;		   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	          
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);    
              }
              else
              {
                replyUpdata[0] = 0x57;
                replyUpdata[1] = 0xFF;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	          
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
              }
            }
            break;

            case 0x33:
            {
              global.appDedata_CRC = appData_Crc(&sys_24c64.Once_OneWeek[responseData[2]][0], global.appDedata_CRC, 7);
              if( global.appDedata_CRC == responseData[3] )
              {
                global.app_DownChange = 0x02;
//                sys_24c64.Once_Single[responseData[2]][0] = 0;
                osal_memcpyz(&sys_24c64.Once_OneWeek[responseData[2]][0], 0xBB, 7);
                sys_24c64time.Once_WeekLimit[responseData[2]][0] = 0;
                replyUpdata[0] = 0x0A;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	          
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
              }
              else
              {
                replyUpdata[0] = 0x57;
                replyUpdata[1] = 0xFF;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	          
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
              }
            }
            break;

            case 0x34:
            {
              global.appDedata_CRC = appData_Crc(&sys_24c64.Once_AMonth[responseData[2]][0], global.appDedata_CRC, 7);
              if( global.appDedata_CRC == responseData[3] )
              {
                global.app_DownChange = 0x02;
//                sys_24c64.Once_Single[responseData[2]][0] = 0;
                osal_memcpyz(&sys_24c64.Once_AMonth[responseData[2]][0], 0xBB, 7);
                sys_24c64time.Once_MonthLimit[responseData[2]][0] = 0;
                replyUpdata[0] = 0x0A;
                replyUpdata[1] = 0x4B;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	          
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
              }
              else
              {
                replyUpdata[0] = 0x57;
                replyUpdata[1] = 0xFF;	   
//                replyUpdata[2] = responseData[2];
//                replyUpdata[3] = global.appDedata_CRC;	          
                DoorProfile_SetParameter(DOORPROFILE_CHAR2, 2, replyUpdata);
              }
            }
            break;

            default:
            break;

            }
          }
        }
        break;
        
       case 0x09:               // ʵʱ
         global.safety_Rssi[0] = responseData[1];
                
        break;
       case 0x0A:               // ��ȫ����
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
//  osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);		//�޸��걣��
//  osal_mem_free(responseData);
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
#if 0
static void Battery_Auto(void)
{
  static uint8 temp = 0;
  static uint8 res[1] = {0},res1[1] = {0xAA};
//  uint8 res = 0;
  
  Adc_Start();

  SendOrGetBatvalue(0);
  SendOrGetBatvalue(0);
  temp = SendOrGetBatvalue(0);
  
  if( (temp >= 84) && (temp <= 96))
  {
    if( temp < 90 )       // ���� + 90 - temp ��ֵ
    {
      res[0] = 90 - temp;
      if(res[0] <= 6)
      {
        AT24C64_I2C_Write(0x1FFE, 1, res);
      }
    }
    else if( temp > 90 ) // ���� - temp - 90 ��ֵ
    {
      res[0] = temp - 90;
      if(res[0] <= 6)
      {
        AT24C64_I2C_Write(0x1FFF, 1, res);
      }
    }
    AT24C64_I2C_Write(0x1FF0, 1, res1);
  }
  
  Adc_Stop();
}
#endif
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
   
  
    if(global.batteryADC > 1943)  // ��С
    {
      res[0] = (uint8)(global.batteryADC - 1943);
      AT24C64_I2C_Write(0x1FFE, 1,(uint8 *) &res[0]);
      AT24C64_I2C_Write(0x1FF0, 1,test);
    }
    else if(global.batteryADC < 1943 && global.batteryADC > 1923) // ����
    {
      res[1] = (uint8)(1943 - global.batteryADC);
      AT24C64_I2C_Write(0x1FFF, 1,(uint8 *) &res[1]);
      AT24C64_I2C_Write(0x1FF0, 1,test);
    }
    else if( global.batteryADC == 1943)
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
