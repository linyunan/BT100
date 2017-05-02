/**************************************************************************************************
  Filename:       SimpleBLEPeripheral_Main.c
  Revised:        $Date: 2010-07-06 15:39:18 -0700 (Tue, 06 Jul 2010) $
  Revision:       $Revision: 22902 $

  Description:    This file contains the main and callback functions for
                  the Simple BLE Peripheral sample application.

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
  PROVIDED “AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
/* Hal Drivers */
#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"
#include "simpleble.h"
#include <math.h>
#include <stdlib.h>    
#include "hal_sensor.h"
#include "hal_i2c.h"
/**************************************************************************************************
 * FUNCTIONS
 **************************************************************************************************/

/* This callback is triggered when a key is pressed */
void MSA_Main_KeyCallback(uint8 keys, uint8 state);

/**************************************************************************************************
 * @fn          main
 *
 * @brief       Start of application.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
int main(void)
{
  uint8 test[1] = {0x00};
  /* Initialize hardware */
  HAL_BOARD_INIT();

  // Initialize board I/O
  InitBoard( OB_COLD );
 
  /* Initialze the HAL driver */
  HalDriverInit();

  /* Initialize NV system */
  osal_snv_init();

  Init_GlobleVar();

  uint8 res = osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config );
  if( res == NV_OPER_FAILED )
  {
    Init_SysConf();
    osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);
//    global.app_DownChange = 0x01;
  }
  AT24C64_I2C_Read(0x0000, sizeof(SYS_24C64), &sys_24c64);
  AT24C64_I2C_Read(0x1000, sizeof(SYS_24C64TIME), &sys_24c64time);
  if( sys_24c64.At24c64_WR != 0xBC )
  {
    Init_Sys24c64();
    Init_Sys24c64Time();
    
    AT24C64_I2C_Write(0x0000, sizeof(SYS_24C64), &sys_24c64);
   
    AT24C64_I2C_Write(0x1000, sizeof(SYS_24C64TIME), &sys_24c64time);
    
//    AT24C64_I2C_Write(0x1FFE, 1, test);
    AT24C64_I2C_Write(0x1FF0, 1, test);
    AT24C64_I2C_Write(0x1FFE, 1, test);
    AT24C64_I2C_Write(0x1FFF, 1, test);
    
//    global.app_DownChange = 0x02;
  }
//  AT24C64_I2C_Write(0x1FFE, 1, (uint8 *)&test[0]);

  /* Initialize LL */

  /* Initialize the operating system */
  osal_init_system();

  /* Enable interrupts */
  HAL_ENABLE_INTERRUPTS();

  // Final board initialization
  InitBoard( OB_READY );

  #if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_BATTERY );
  #endif

  /* Start OSAL */
  osal_start_system(); // No Return from here

  return 0;
}

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
