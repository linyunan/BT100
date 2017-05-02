/**************************************************************************************************
  Filename:       hal_sensor.h
  Revised:        $Date: 2013-03-26 07:47:25 -0700 (Tue, 26 Mar 2013) $
  Revision:       $Revision: 33597 $

  Description:    Interface to sensor driver shared code.


  Copyright 2012-2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef HAL_SENSOR_H
#define HAL_SENSOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
   
#define BTN1_INDEX	0x01
#define BTN2_INDEX	0x02
#define BTN3_INDEX	0x04
#define BTN4_INDEX	0x08

#define BTN5_INDEX	0x10
#define BTN6_INDEX	0x20
#define BTN7_INDEX	0x40
#define BTN8_INDEX	0x80
   
   
#define BTN9_INDEX	0x0100
#define BTN10_INDEX	0x0200
#define BTN11_INDEX	0x0400
#define BTN12_INDEX	0x0800


   
/*****************************************************************************
* MACRO Definition
*****************************************************************************/

/* CY8CMBR3116 Register Map Offset Address */
#define REGMAP_ORIGIN			0x00
#define SENSOR_PATTERN			0x00
#define FSS_EN					0x02
#define TOGGLE_EN				0x04
#define LED_ON_EN				0x06
#define SENSITIVITY0			0x08
#define SENSITIVITY1			0x09
#define SENSITIVITY2			0x0A
#define SENSITIVITY3			0x0B
#define BASE_THRESHOLD0			0x0C
#define BASE_THRESHOLD1			0x0D
#define FINGER_THRESHOLD2		0x0E
#define FINGER_THRESHOLD3		0x0F
#define FINGER_THRESHOLD4		0x10
#define FINGER_THRESHOLD5		0x11
#define FINGER_THRESHOLD6		0x12
#define FINGER_THRESHOLD7		0x13
#define FINGER_THRESHOLD8		0x14
#define FINGER_THRESHOLD9		0x15
#define FINGER_THRESHOLD10		0x16
#define FINGER_THRESHOLD11		0x17
#define FINGER_THRESHOLD12		0x18
#define FINGER_THRESHOLD13		0x19
#define FINGER_THRESHOLD14		0x1A
#define FINGER_THRESHOLD15		0x1B
#define SENSOR_DEBOUNCE			0x1C
#define BUTTON_HYS				0x1D
#define BUTTON_BUT				0x1E
#define BUTTON_LBR				0x1F
#define BUTTON_NNT				0x20
#define BUTTON_NT				0x21
#define PROX_EN					0x26
#define PROX_CFG				0x27
#define PROX_CFG2				0x28
#define PROX_TOUCH_TH0			0x2A
#define PROX_TOUCH_TH1			0x2C
#define PROX_HYS				0x30
#define PROX_BUT				0x31
#define PROX_LBR				0x32
#define PROX_NNT				0x33
#define PROX_NT					0x34
#define PROX_POSITIVE_TH0		0x35
#define PROX_POSITIVE_TH1		0x36
#define PROX_NEGATIVE_TH0		0x39
#define PROX_NEGATIVE_TH1		0x3A
#define LED_ON_TIME				0x3D
#define BUZZER_CFG				0x3E
#define BUZZER_ON_TIME			0x3F
#define GPO_CFG					0x40
#define PWM_DUTYCYCLE_CFG0		0x41
#define PWM_DUTYCYCLE_CFG1		0x42
#define PWM_DUTYCYCLE_CFG2		0x43
#define PWM_DUTYCYCLE_CFG3		0x44
#define PWM_DUTYCYCLE_CFG4		0x45
#define PWM_DUTYCYCLE_CFG5		0x46
#define PWM_DUTYCYCLE_CFG6		0x47
#define PWM_DUTYCYCLE_CFG7		0x48
#define SPO_CFG					0x4C
#define DEVICE_CFG0				0x4D
#define DEVICE_CFG1				0x4E
#define DEVICE_CFG2				0x4F
#define I2C_ADDR				0x51
#define REFRESH_CTRL			0x52
#define STATE_TIMEOUT			0x55
#define SLIDER_CFG				0x5D
#define SLIDER1_CFG				0x61
#define SLIDER1_RESOLUTION		0x62
#define SLIDER1_THRESHOLD		0x63
#define SLIDER2_CFG				0x67
#define SLIDER2_RESOLUTION		0x68
#define SLIDER2_THRESHOLD		0x69
#define SLIDER_DEBOUNCE			0x6F
#define SLIDER_BUT				0x70
#define SLIDER_LBR				0x71
#define SLIDER_NNT				0x72
#define SLIDER_NT				0x73
#define CONFIG_CRC				0x7E
#define GPO_OUTPUT_STATE		0x80
#define SENSOR_ID				0x82
#define CTRL_CMD				0x86
#define BUTTON_STATUS			0xAA

/* Command Codes */
#define CMD_NULL				0x00
#define SAVE_CALC_CRC           0x01
#define SAVE_CHECK_CRC          0x02
#define CALC_CRC                0x03
#define LOAD_FACTORY            0x04
#define LOAD_PRIMARY            0x05
#define LOAD_SECONDARY          0x06
#define SLEEP                   0x07
#define CLEAR_LATCHED_STATUS    0x08
#define CMD_RESET_PROX0_FILTER	0x09
#define CMD_RESET_PROX1_FILTER	0x0A
#define ENTER_CONFIG_MODE       0x0B
#define EXIT_CONTROL_RUN        0xFE
#define SW_RESET                0xFF

/* Total number of configuration registers */
#define TOTAL_CONFIG_REG_COUNT	0x80

/* Length of Register Map */
#define REG_MAP_LEN	256

/* Slave Address (Default) */
#define SLAVE_ADDR				0x37
	
#define NO_OF_KIT_BUTTONS	4



/*****************************************************************************
* Function Prototypes
*****************************************************************************/


/*****************************************************************************
* External Function Prototypes
*****************************************************************************/
   
#define MBR3_DEV_ADDR				0x37  //MBR3´¥ÃþÐ¾Æ¬Æ÷¼þµØÖ· 
#define MBR3_RA_WHO_AM_I			0x51 //MBR3´¥ÃþÐ¾Æ¬Æ÷¼þµØÖ· ËùÔÚ¼Ä´æÆ÷µØÖ·


#define E2P_24C64_ADDR                      0x50 //24C64´æ´¢Ð¾Æ¬Æ÷¼þµØÖ·
   
   

/*********************************************************************
 * CONSTANTS and MACROS
 */

/* Self test assertion; return FALSE (failed) if condition is not met */
#define ST_ASSERT(cond) st( if (!(cond)) return FALSE; )

/* Ative delay: 125 cycles ~1 msec */
#define ST_HAL_DELAY(n) st( { volatile uint32 i; for (i=0; i<(n); i++) { }; } )

/*********************************************************************
 * Globalvariables
 */
extern uint8 readBuffer[2];
extern uint8 writeBuffer[2];
extern uint16 buttonStatus;
extern uint8 bbb[7];

/*********************************************************************
 * FUNCTIONS
 */
void   HalSensorInit( uint8 );
bool   HalSensorReadReg( uint8 , uint8 *, uint8  );
bool   HalSensorWriteReg( uint8 , uint8 *, uint8  );



uint8 MBR3_I2C_Read( uint8 addr, uint8 len, uint8 *buf);
uint8 MBR3_I2C_Write( uint8 addr, uint8 len, uint8 *buf);
extern uint8 MBR3_RESET( void );
void DelayMS(uint16);  //ÑÓÊ± microSecs * 1ms

extern int8 ReadButtonStatus(void);
void BackLight_ON( void );
void BackLight_OFF( void );
void I2C_24C64(void);


/*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HAL_SENSOR_H */
