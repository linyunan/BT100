#ifndef SIMPLEBLE_H
#define SIMPLEBLE_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "comdef.h"
/*********************************************************************
 * global variable to snv
 */
typedef struct 
{
  uint8 Admin_Pswd[8];		        // ����Ա����	
//  uint8 General_Pswd[10][7];		// �û�����
 
  uint8 Phone_User[10][8];
//  uint8 MY_KEY_CODE;
  uint8 MY_KEYCODE[16];
//  uint8 sys_InitStatus;				// ��������ԱAPP �ϴ������λ�����Ϊ�������
  union
  {
    uint16 Recode_Addr_Fir;                  // ����������洢��ַ
    uint8 Recode_Addr_F2[2];
  };
  union
  {
    uint16 Recode_Addr_Sec;                  // ����������洢��ַ 
    uint8 Recode_Addr_S2[2];
  };
  
  uint8 ChiOrEng;
  bool Language_Choice;
  
}SYS_CONFIG;


/*********************************************************************
 * global variable to 24C64
 */
typedef struct
{
  uint8 At24c64_WR;               //  ��ַ0x1FFF �Ƿ�д��
  uint8 General_Pswd[10][9];		// �û�����

  uint8 Admin_Phone_Flag;               // ����Ա���ñ�־λ�� 0 Ϊδ�����ֻ�����Ա �� 1Ϊ�Ѿ������ֻ�����Ա
  
  uint8 Once_Single[10][9];			// һ��������ֻʹ��һ��
  uint8 Once_OneDay[10][9];		// һ��������ʹ��һ��

  uint8 Once_OneWeek[10][9];		// һ��������ʹ��һ������

  uint8 Once_AMonth[10][9];		// һ��������ʹ��һ����

  uint8 Phone_Num;                      // �Ѿ����ڵ��ֻ��û�����
  uint8 Keys_num;                       // �Ѿ����ڵİ����û�����
  
  union
  {
    uint8 DayLimit[10][4];
    uint32 Once_DayLimit[10][1];   		// һ��������һ������ʱ�����ƴ洢�ռ�
  };
  union
  {
    uint8 WeekLimit[10][4];
    uint32 Once_WeekLimit[10][1];		// һ��������һ��������ʱ�����ƴ洢�ռ�
  };
  union
  {
    uint8 MonthLimit[10][4];		// һ��������ʹ��һ����
    uint32 Once_MonthLimit[10][1];	// һ��������һ��������ʱ�����ƴ洢�ռ�
  };
}SYS_24C64;

typedef struct
{
  uint8 open_DoorRec[3];           //   ���ż�¼ 0-19  ����19����
  uint8 Open_Recode[15][9];		// ���ż�¼��Ϣ
  uint8 Open_Count;
}SYS_RECODE;

/*********************************************************************
 * global variable don't need save
 */
typedef struct 
{
  uint8 authoryMode;            // Ȩ��ģʽ��0: ��Ȩ��  1���ֻ�����Ա   2���ֻ���ͨ�û���
  uint8 openDoorMode;           //  �ֻ�����ģʽ( 0�� δѡ��ģʽ��Ĭ���Զ���   1:  �Զ�    2:  �����ֻ���Ļ)
  uint8 AddPhoneFlag;		// ��λ�������ģʽ(0��������ģʽ    1. �������ģʽ   ����������? )

//  uint8 addPhoneNum;		// ���ӵڼ����ֻ��û�
//  uint8 addPswdNum;		// ���ӵڼ�����������

  uint8 batteryPower[3];		// ������ȡ
  uint8 openDoorTurn;           //  ��������ת��
  uint8 AppOpenStatus;	        // APP ����״̬λ    0 :  ������  1 : �Զ�ģʽ  2 : ������Ļ����ģʽ
  
  bool longButtonFlag;          //  ������־
  uint8 genPswdFlag;		//  �����������뿪�ű�־   0 : Ϊ�޲�����������������
  uint8 admPswdFlag;		//  �������Աģʽ��ѡ���־λ 1 : ����ֻ���2 : �޸Ĺ���Ա���� ������
  uint8 MvInPswd[15];		//  �10λ��λ��������

  uint8 optionOutTime;	        //	������ʱ
  uint8 menu_Num;		//	�������Աģʽ������־������һ��������1
  uint8 menu_Index;		//	�������Աģʽ����������
  uint8 keyRecode_Temp[8];	        //	��ʱ����Ƚ�  ������������ıȽ�
//  uint8 tmRecode_Temp[6];         //  TM ����ʱ�洢�Ƚ�
  
  uint8 sys_OptionFlag;		//    ��Ϊ����λ���� ���λ��־
  uint8 appRecode_Temp[7];	//	�洢 APP �·����ֻ�����Ϣ	

  uint8 appUpdata_CRC;		//  �ϴ�����CRC У����
  uint8 appDedata_CRC;		//  app ɾ���û���Ϣʹ�õ�CRC У����
  
  uint8 safety_Flag;            //   ��ֹ��������������룬����6�ν������10���Ӳſɱ�����
  uint32 time_Recode;            //   ���Ұ�����ʱ���¼ 10����
  uint8 safety_Rssi[2];     //   [0]  ʵʱ��RSSI   [1] ��ȫRSSI ֵ			
  
  uint8 sys_UpdataStatus;		//	ϵͳ�û���Ϣ�Ƿ��и���          0x00 : �����£�0x01 : ����Ա�������
  							//   0x02 : �ֻ��û� 0x04 : ��ͨ�û�����    0x08 :  TM ������   0x10: ���ż�¼����	
  uint8 sys_CurrentUp;      // ��ǰ��һ���ϴ�����ʲô�û���Ϣ����¼ ���Ա�����ش�
  uint8 app_DownChange;	//  APP �·�������λ��������־λ  0x01  (����Ա���ֻ��û�)  0x02 (TM ������ͨ�����û���һ����)
  
  uint8 batteryNum;             // ������ȡ����
//  uint8 batteryNumOut;      // ��ȡ��Ч����
  uint16 batteryADC;             // ADCֵ
  uint8 batteryPer;             // �ٷֱ�
  uint8 mainMenuUpDown;        // ����ѡ��
  uint8 delPhoneNum;           // ɾ���ֻ��û�
  uint8 delKeysNum;            // ɾ�������û�
  uint8 MenuDelay;              // ��ʱ��ʾ�޷�����
  uint8 MY_KEY_CODE;
  uint8 Battery_Empty;         
  uint8 Choice_Flag;
  
  uint8 Del_Phone_User[10][6];  // ����ɾ�����ֻ��û�
  uint8 Del_Phone_Num;          // ����ɾ�����ֻ��û�����
}GLOBAL_VARIABLE; 
  

extern SYS_CONFIG sys_config;

extern GLOBAL_VARIABLE global;

extern SYS_24C64 sys_24c64;

extern SYS_RECODE sys_recode;


void Init_GlobleVar(void);
void Init_Sys24c64(void);
void Init_SysConf(void);
void Init_Recode(void);

void Array_Stack(void);
void *osal_memcpyz( void *dst, uint8 data, unsigned int len );
uint8 osal_memcmpn( const void GENERIC *src1, const void GENERIC *src2, unsigned int len );
uint8 osal_array(uint8 * src);

#endif /* SIMPLEBLE_H */
