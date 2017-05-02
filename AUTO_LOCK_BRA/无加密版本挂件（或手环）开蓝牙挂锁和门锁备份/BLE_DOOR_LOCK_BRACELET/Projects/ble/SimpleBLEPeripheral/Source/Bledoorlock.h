/**************************************************************************************************
  Filename:       simpleBLEperipheral.h
  Revised:        $Date: 2010-08-01 14:03:16 -0700 (Sun, 01 Aug 2010) $
  Revision:       $Revision: 23256 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  definitions and prototypes.

  Copyright 2010 - 2011 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED ìAS ISî WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef SIMPLEBLEPERIPHERAL_H
#define SIMPLEBLEPERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#if 0
#define ADMIN_PSWD_LEN  10
#define GENERAL_PSWD_LEN  10

#define ADMIN_PHONE_LEN   6
#define GENERAL_PHONE_LEN	6

#endif

  
#define BUTTON_UINT8(hiByte, loByte) \
          ((uint8)(((loByte-0x30) & 0x0F) + (((hiByte-0x30) & 0x0F))*10))

#define SYS_INIT    P1_7
#define ALARM    P1_6
//extern  uint8 bleDoorLock_TaskID;

/*********************************************************************
 * CONSTANTS
 */


// Simple BLE Peripheral Task Events
#define DOOR_START_DEVICE_EVT                   0x0001
#define DOOR_PERIODIC_EVT                       0x0002	 //	œµÕ≥ µ ±–‘»ŒŒÒ
#define DOOR_ENCRYPT_EVT			0x0004	 //	√ÿ‘øºÏ≤‚»ŒŒÒ
#define DOOR_PHONE_EVT		                0x0008	 //	 ÷ª˙”√ªß»ŒŒÒ
#define DOOR_OPENDOOR_EVT			0x0010	 //     ø™πÿ√≈»ŒŒÒ
#define DOOR_INITCONF_EVT	                0x0020   //     ª÷∏¥≥ˆ≥ß…Ë÷√
#define DOOR_POWERONOFF_EVT			0x0040	 //	¥˝ª˙ ∫ÕªΩ–—»ŒŒÒ
#define DOOR_SAFTYALARM_EVT                     0x0080   //      ∑¿≤±®æØ
#define DOOR_ONCEPSWD_EVT			0x0100	 //	“ª¥Œ–‘√‹¬Îπ‹¿Ì»ŒŒÒ
#define DOOR_ADMININ_EVT		        0x0200	 //	∞¥º¸π‹¿Ì‘±≤Ÿ◊˜»ŒŒÒ
#define DOOR_ADMIN_PRO_EVT			0x0400   //	Ω¯»Îπ‹¿Ì‘±ƒ£ Ω∫Û≤„º∂¥¶¿Ì
#define DOOR_PSWD_OPEN_EVT                      0x0800   //     ∞¥º¸√‹¬Îø™√≈
#define DOOR_OUTTIME_EVT                        0x1000   //     ∞¥º¸≤Ÿ◊˜≥¨ ±¥¶¿Ì
#define DOOR_READCAR_EVT                        0x2000   //     ∂¡»°TM ø®∫≈»ŒŒÒ
#define DOOR_BATTERY_EVT                        0x4000   //     µÁ≥ÿºÏ≤‚»ŒŒÒ
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void BleDoorLock_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 BleDoorLock_ProcessEvent( uint8 task_id, uint16 events );
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
extern char *bdAddr2Str( uint8 *pAddr );
#endif
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLEPERIPHERAL_H */
