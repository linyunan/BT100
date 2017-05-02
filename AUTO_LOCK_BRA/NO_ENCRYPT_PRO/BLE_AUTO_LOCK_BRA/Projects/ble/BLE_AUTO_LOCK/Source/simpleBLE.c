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
#include "peripheral.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "npi.h"
#include "osal_snv.h"
#include "simpleBLE.h"

#include "bleautolock.h"

#include "osal_clock.h"

#include <string.h>
SYS_CONFIG sys_config;

GLOBAL_VARIABLE global;

SYS_24C64 sys_24c64;

SYS_RECODE sys_recode;
static uint8 Init_Pswd[8] = "00000000";

void Array_Stack(void)  // ��������и����滻
{
  for(uint8 i=0;i<19;i++)
  {
    osal_memcpy(&sys_recode.Open_Recode[i][0], &sys_recode.Open_Recode[i+1][0], 9);    
  }
}

/*********************************************************************
 * @fn      osal_memcmpn
 *
 * @brief
 *
 *   Generic memory compare . ��src2 ��src1  ������len �����ֽ�
 *
 * @param   src1 - source 1 addrexx
 * @param   src2 - source 2 address
 * @param   len - number of bytes to compare
 *
 * @return  TRUE - same, FALSE - different
 */
uint8 osal_memcmpn( const void GENERIC *src1, const void GENERIC *src2, unsigned int len )
{
  const uint8 GENERIC *pSrc1;
  const uint8 GENERIC *pSrc2;
  
  
  
  uint8 temp1=0;

  pSrc1 = src1;
  pSrc2 = src2;
  
//  scanf("%s", pSrc2);
  uint8 lenght = strlen(src2)+2;
//  uint8 lenght = sizeof(pSrc2)/sizeof(pSrc2[0]);
  
  while( lenght -- )
  {
    if( *pSrc1++ == *pSrc2++ )
    {
      temp1 ++;
      if(temp1 == len)
        return TRUE;
      
    }
    else
    {

      while(temp1--)
      {
        *pSrc2 --;
        lenght ++;
      }

      temp1 = 0;
      pSrc1 = src1;  // ���¸�ֵ�Ƚ�
    
    }
  } 
  return FALSE;
}

/*********************************************************************
 * @fn      osal_memcpyz
 *
 * @brief
 *
 *   Generic memory copy.
 *
 *   Note: This function differs from the standard memcpy(), since
 *         it returns the pointer to the next destination uint8. The
 *         standard memcpy() returns the original destination address.
 *
 * @param   dst - destination address
 * @param   src - source address
 * @param   len - number of bytes to copy
 *
 * @return  pointer to end of destination buffer
 */
void *osal_memcpyz( void *dst, uint8 data, unsigned int len )
{
  uint8 *pDst;
//  const uint8 GENERIC *pSrc;

//  pSrc = src;
  pDst = dst;

  while ( len-- )
    *pDst++ = data;

  return ( pDst );
}
/********************************************************************
 * ��ȡ�����м�ֵ
 *
 *
********************************************************************/
uint8 osal_array(uint8 * src)
{
  uint8 lenght = strlen((char *)src);
  uint8 temp = 0;
  uint8 *psrc ;
  psrc = src;
  for(uint8 i=0;i<lenght-1;i++)
  {
    for(uint8 j=i+1;j<lenght;j++)  
    {
      if(psrc[i]>psrc[j])
      {     
        temp = psrc[i];     
        psrc[i] = psrc[j];         
        psrc[j] = temp;
      }
    } 
  }
  return psrc[lenght/2 + 1];
}
uint8 Crc_Check(uint8 * src, uint16 len)
{
  uint8 i;
  uint8 temp = 0;
  const uint8 * pSrc;
  pSrc = src;
#if 1
  for( i=0; i<len; i++)
  {
    temp = (pSrc[i] + temp)&0x0F;
  }

  return temp;
#else
  for( i=0; i<len; i++)
  {
    if(!(pSrc[i]%2))
      temp ++;

  }
  if((temp)&&(!(temp%2)))
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
#endif  
  
}

//static uint8 InitSys_ZG[] = {0};
//  ������SNV �µı�����ʼ��
void Init_SysConf(void)
{
  osal_memcpy(sys_config.Admin_Pswd, Init_Pswd, 8);
  
  for(uint8 i=0;i<11; i++)
  {
    sys_config.Phone_User[i][0] = 0xFF;
    osal_memcpyz(&sys_config.Phone_User[i][1], 0, 7);
  }
  sys_config.ChiOrEng = 0xFF;
  sys_config.Language_Choice = TRUE;
}


//  ������E2PROM �µı�����ʼ��
void Init_Sys24c64(void)
{

  for(uint8 i=0;i<10;i++)
  {
    osal_memcpyz(&sys_24c64.General_Pswd[i][0], 0xBB, 9);
  }

  for(uint8 i=0;i<10;i++)
  {
    sys_24c64.Once_DayLimit[i][0] = 0x00;
    sys_24c64.Once_WeekLimit[i][0] = 0x00;
    sys_24c64.Once_MonthLimit[i][0] = 0x00;
  }
  
  for(uint8 i=0;i<10;i++)
  {
    osal_memcpyz(&sys_24c64.Once_Single[i][0], 0xBB,9);		// һ��������ֻʹ��һ��

    osal_memcpyz(&sys_24c64.Once_OneDay[i][0], 0xBB,9);	        // һ��������ʹ��һ��

    osal_memcpyz(&sys_24c64.Once_OneWeek[i][0], 0xBB,9);	// һ��������ʹ��һ������

    osal_memcpyz(&sys_24c64.Once_AMonth[i][0], 0xBB,9);		// һ��������ʹ��һ����
  }

  sys_24c64.Admin_Phone_Flag = 0;               // ����Ա���ñ�־λ�� 0 Ϊδ�����ֻ�����Ա �� 1Ϊ�Ѿ������ֻ�����Ա
  
  sys_24c64.At24c64_WR = 0xBC;
  sys_24c64.Keys_num = 0xBB;
  sys_24c64.Phone_Num = 0xBB;

}

void Init_Recode(void)
{
  sys_recode.open_DoorRec[0] = 1;
  sys_recode.open_DoorRec[1] = 0xFF;
  sys_recode.open_DoorRec[2] = 0xFF;
  sys_recode.Open_Count = 1;
  for(uint8 i=0;i<20;i++)
  {
    osal_memcpyz(&sys_recode.Open_Recode[i][0], 0, 9);		// ���ż�¼��Ϣ
  }
}

//  ϵͳȫ�ֱ�����ʼ��
void Init_GlobleVar(void)
{
  global.AddPhoneFlag = 0;
  global.authoryMode = 0;
  global.openDoorMode = 0;
//  global.addPhoneNum = 0;
  global.openDoorTurn = 0;
  global.AppOpenStatus = 0;
  global.longButtonFlag = FALSE;
  
//  global.addPswdNum = 0xFF;		// ���ӵڼ�����������

  global.genPswdFlag = 0;		//  �����������뿪�ű�־   0 : Ϊ�޲�����������������
  global.admPswdFlag = 0;		//  �������Աģʽ��ѡ���־λ 1 : ����ֻ���2 : �޸Ĺ���Ա���� ������
  
  global.menu_Num= 0 ;
  global.menu_Index = 0;
  osal_memcpyz(global.keyRecode_Temp , 0xBB, 8);

  
  global.sys_UpdataStatus = 0;
  global.sys_CurrentUp = 0;
  global.appUpdata_CRC = 0;
  global.appDedata_CRC = 0;
  global.sys_OptionFlag = 0x00;
  global.safety_Flag = 0;
  global.app_DownChange = 0;

  global.batteryNum = 0;             // ������ȡ����
//  global.batteryNumOut = 0;      // ��ȡ��Ч����
  global.batteryADC = 0;             // ADCֵ
  global.batteryPer = 0;             // �ٷֱ�
  
  global.delPhoneNum = 1;
  global.delKeysNum = 0;
  global.MenuDelay = 0;
  
  global.mainMenuUpDown = 2;  //������Ч��Χ  2 - 6
  
  global.MY_KEY_CODE = 0;
  global.Choice_Flag = 0;
  
  osal_memcpyz(global.batteryPower , 0, 3);      // ������ȡ
  osal_memcpyz(global.appRecode_Temp , 0, 8);
  osal_memcpyz(global.safety_Rssi , 0, 2);
  osal_memcpyz(global.MvInPswd, 0xFF, sizeof(global.MvInPswd));		//  �10λ��λ��������
}


