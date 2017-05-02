/**************************************************************************************************
  Filename:       simpleGATTprofile.c
  Revised:        $Date: 2013-05-06 13:33:47 -0700 (Mon, 06 May 2013) $
  Revision:       $Revision: 34153 $

  Description:    This file contains the Simple GATT profile sample GATT service 
                  profile for use with the BLE sample application.

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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "hal_lcd.h"

#include "doorGATTprofile.h"

#include "bledoorlock.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define DOORSERVAPP_NUM_ATTR_SUPPORTED        14

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Door GATT Profile Service UUID: 0xFFE0
CONST uint8 doorProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(DOORPROFILE_SERV_UUID), HI_UINT16(DOORPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFE1
CONST uint8 doorProfilechar1UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(DOORPROFILE_CHAR1_UUID), HI_UINT16(DOORPROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFE2
CONST uint8 doorProfilechar2UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(DOORPROFILE_CHAR2_UUID), HI_UINT16(DOORPROFILE_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFE3
CONST uint8 doorProfilechar3UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(DOORPROFILE_CHAR3_UUID), HI_UINT16(DOORPROFILE_CHAR3_UUID)
};

// Characteristic 4 UUID: 0xFFE4
CONST uint8 doorProfilechar4UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(DOORPROFILE_CHAR4_UUID), HI_UINT16(DOORPROFILE_CHAR4_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static doorProfileCBs_t *doorProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// door Profile Service attribute
static CONST gattAttrType_t doorProfileService = { ATT_BT_UUID_SIZE, doorProfileServUUID };


// door Profile Characteristic 1 Properties
static uint8 doorProfileChar1Props = GATT_PROP_READ | GATT_PROP_WRITE ;

// Characteristic 1 Value
static uint8 doorProfileChar1[DOORPROFILE_CHAR1_LEN] ;

static uint8 doorProfileChar1UserDesp[17] = "Characteristic 1\0";
// door Profile Characteristic 2 Properties
static uint8 doorProfileChar2Props = GATT_PROP_NOTIFY | GATT_PROP_READ | GATT_PROP_WRITE_NO_RSP;

// Characteristic 2 Value
static uint8 doorProfileChar2[DOORPROFILE_CHAR2_LEN];
static uint8 doorProfileChar2len = 0;

// Door Profile Characteristic 2 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t doorProfileChar2Config[GATT_MAX_NUM_CONN];

static uint8 doorProfileChar2UserDesp[17] = "Characteristic 2\0";

// door Profile Characteristic 3 Properties
static uint8 doorProfileChar3Props = GATT_PROP_READ | GATT_PROP_WRITE  ;

// Characteristic 3 Value
static uint8 doorProfileChar3[DOORPROFILE_CHAR3_LEN] ;

static uint8 doorProfileChar3UserDesp[17] = "Characteristic 3\0";

// door Profile Characteristic 4 Properties
static uint8 doorProfileChar4Props = GATT_PROP_READ | GATT_PROP_WRITE ;

// Characteristic 4 Value
static uint8 doorProfileChar4[DOORPROFILE_CHAR4_LEN] ;

static uint8 doorProfileChar4UserDesp[17] = "Characteristic 4\0";
/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t doorProfileAttrTbl[DOORSERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Simple Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&doorProfileService            /* pValue */
  },

    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &doorProfileChar1Props 
    },

      // Characteristic Value 1
      { 
        { ATT_BT_UUID_SIZE, doorProfilechar1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        doorProfileChar1 
      },
    
      // Characteristic 1 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        doorProfileChar1UserDesp 
      },      

    // Characteristic 2 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &doorProfileChar2Props 
    },

      // Characteristic Value 2
      { 
        { ATT_BT_UUID_SIZE, doorProfilechar2UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,  
        0, 
        doorProfileChar2 
      },          

      // Notification value Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)doorProfileChar2Config 
      },
      // Characteristic 2 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        doorProfileChar2UserDesp 
      },         
      
    // Characteristic 3 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &doorProfileChar3Props 
    },

      // Characteristic Value 3
      { 
        { ATT_BT_UUID_SIZE, doorProfilechar3UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        doorProfileChar3 
      },

      // Characteristic 3 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        doorProfileChar3UserDesp 
      },
      // Characteristic 4 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &doorProfileChar4Props 
    },

      // Characteristic Value 4
      { 
        { ATT_BT_UUID_SIZE, doorProfilechar4UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        doorProfileChar4 
      },
    
      // Characteristic 4 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        doorProfileChar4UserDesp 
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 doorProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t doorProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void doorProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// Door Profile Service Callbacks
CONST gattServiceCBs_t doorProfileCBs =
{
  doorProfile_ReadAttrCB,  // Read callback function pointer
  doorProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      DoorProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t DoorProfile_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, doorProfileChar2Config );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( doorProfile_HandleConnStatusCB );  
  
  if ( services & DOORPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( doorProfileAttrTbl, 
                                          GATT_NUM_ATTRS( doorProfileAttrTbl ),
                                          &doorProfileCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      DoorProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t DoorProfile_RegisterAppCBs( doorProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    doorProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}
  

/*********************************************************************
 * @fn      DoorProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t DoorProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case DOORPROFILE_CHAR1:
      if ( len <= DOORPROFILE_CHAR1_LEN ) 
      {
        VOID osal_memcpy( doorProfileChar1, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DOORPROFILE_CHAR2:
      if ( len <= DOORPROFILE_CHAR2_LEN ) 
      {
        VOID osal_memcpy( doorProfileChar2, value, len );//DOORPROFILE_CHAR2_LEN
	     static attHandleValueNoti_t noti;
        noti.handle = doorProfileAttrTbl[5].handle ;
        noti.len = len;
        doorProfileChar2len = len;
        osal_memcpy((uint8*)noti.value, (uint8*)doorProfileChar2, len);
        VOID GATT_Notification( 0, &noti, FALSE);   
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DOORPROFILE_CHAR3:
      if ( len <= DOORPROFILE_CHAR3_LEN ) 
      {
        VOID osal_memcpy( doorProfileChar3, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    case DOORPROFILE_CHAR4:
      if ( len <= DOORPROFILE_CHAR4_LEN ) 
      {
        VOID osal_memcpy( doorProfileChar4, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;  
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      DoorProfile_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t DoorProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case DOORPROFILE_CHAR1:
      VOID osal_memcpy( value, doorProfileChar1, DOORPROFILE_CHAR1_LEN );
      break;

    case DOORPROFILE_CHAR2:
      VOID osal_memcpy( value, doorProfileChar2, DOORPROFILE_CHAR2_LEN );
      break;      

    case DOORPROFILE_CHAR3:
      VOID osal_memcpy( value, doorProfileChar3, DOORPROFILE_CHAR3_LEN );
      break;  
    
    case DOORPROFILE_CHAR4:
      VOID osal_memcpy( value, doorProfileChar4, DOORPROFILE_CHAR4_LEN );
      break;  
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          doorProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 doorProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 1 and 2 have read permissions
      // characteritisc 3 does not have read permissions; therefore it is not
      //   included here
      // characteristic 4 does not have read permissions, but because it
      //   can be sent as a notification, it is included here
      case DOORPROFILE_CHAR1_UUID:
	*pLen = DOORPROFILE_CHAR1_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, DOORPROFILE_CHAR1_LEN );
	break;
	
      case DOORPROFILE_CHAR2_UUID:
	*pLen = doorProfileChar2len;
        VOID osal_memcpy( pValue, pAttr->pValue, DOORPROFILE_CHAR2_LEN );
	break;
	
      case DOORPROFILE_CHAR3_UUID:
	*pLen = DOORPROFILE_CHAR3_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, DOORPROFILE_CHAR3_LEN );
	break;
	
      case DOORPROFILE_CHAR4_UUID:
	*pLen = DOORPROFILE_CHAR4_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, DOORPROFILE_CHAR4_LEN );
	break;
        
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      doorProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t doorProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    
    switch ( uuid )
    {
      case DOORPROFILE_CHAR1_UUID:
	 //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > DOORPROFILE_CHAR1_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
//          uint8 *pCurValue = (uint8 *)pAttr->pValue;        
//          *pCurValue = pValue[0];

	  VOID osal_memcpy( pAttr->pValue, pValue, DOORPROFILE_CHAR1_LEN);

//          if( osal_memcmp(pAttr->pValue , doorProfileChar1, len) )
          {
            notifyApp = DOORPROFILE_CHAR1;        
          }
//          else
//          {
//            notifyApp = 0xFF;
//          }
        }

        break;
		
      case DOORPROFILE_CHAR2_UUID:
        if ( offset == 0 )
        {
          if ( len > DOORPROFILE_CHAR1_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
//          uint8 *pCurValue = (uint8 *)pAttr->pValue;        
//          *pCurValue = pValue[0];

	  VOID osal_memcpy( pAttr->pValue, pValue, DOORPROFILE_CHAR2_LEN);

//          if( osal_memcmp(pAttr->pValue , doorProfileChar2, len) )
          {
            notifyApp = DOORPROFILE_CHAR2;        
          }
//          else
//          {
//            notifyApp = 0xFF;
//          }
        }

        break;
		
      case DOORPROFILE_CHAR3_UUID:

        if ( offset == 0 )
        {
          if ( len > DOORPROFILE_CHAR3_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
	  VOID osal_memcpy( pAttr->pValue, pValue, DOORPROFILE_CHAR3_LEN);

//          if( osal_memcmp(pAttr->pValue , doorProfileChar3, len) )
          {
            notifyApp = DOORPROFILE_CHAR3;        
          }
//          else
//          {
//            notifyApp = 0xFF;
//          }
        }

        break;
      case DOORPROFILE_CHAR4_UUID:
	 //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > DOORPROFILE_CHAR4_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }        
        //Write the value
        if ( status == SUCCESS )
        {
	  VOID osal_memcpy( pAttr->pValue, pValue, DOORPROFILE_CHAR4_LEN);

//          if( osal_memcmp(pAttr->pValue , doorProfileChar1, len) )
          {
            notifyApp = DOORPROFILE_CHAR4;        
          }
        }

        break;
        
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && doorProfile_AppCBs && doorProfile_AppCBs->pfnDoorProfileChange )
  {
    doorProfile_AppCBs->pfnDoorProfileChange( notifyApp );  
  }
  
  return ( status );
}

/*********************************************************************
 * @fn          doorProfile_HandleConnStatusCB
 *
 * @brief       Door Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void doorProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, doorProfileChar2Config );
    }
  }
}


/*********************************************************************
*********************************************************************/
