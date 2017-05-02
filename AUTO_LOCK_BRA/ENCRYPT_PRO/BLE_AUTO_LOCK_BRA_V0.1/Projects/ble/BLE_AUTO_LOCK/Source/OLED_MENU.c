#include "OLED_MENU.h"
#include "hal_sensor.h"

void OLED_DISPLAY_ONOFF(bool OnOff)
{
  if(OnOff)
  {
    Write_Command(0xAF); //Set Display On 
    OLED_Welcom();
//    OLED_INPUT_UNLOCK();
  }
  else
  {
    Clear_Screen(0);
    Write_Command(0xAE); //Set Display Off 
  }
}

void OLED_Welcom(void)
{
  Clear_Screen(0);
  if( sys_config.ChiOrEng )
  {
    //   ������
    LCD_P16x16Ch(0,0,0,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,0,Font_Menu_Fir);  
    LCD_P16x16Ch(32,0,1,Font_Menu_Fir); 
    LCD_P16x16Ch(48,0,2,Font_Menu_Fir);  
    LCD_P16x16Ch(64,0,3,Font_Menu_Fir);  
            
    //  �����ٵ��ӿƼ�
    LCD_P16x16Ch(0,2,4,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,5,Font_Menu_Fir);  
    LCD_P16x16Ch(32,2,6,Font_Menu_Fir);  
    LCD_P16x16Ch(48,2,7,Font_Menu_Fir);  
    LCD_P16x16Ch(64,2,8,Font_Menu_Fir);  
    LCD_P16x16Ch(80,2,9,Font_Menu_Fir);  
    LCD_P16x16Ch(96,2,10,Font_Menu_Fir);  
    LCD_P16x16Ch(112,2,0,Font_Menu_Fir);  
  }
  else
  {
    LCD_P8x16Str(24, 0, "SmartLock");
    LCD_P8x16Str( 0, 2, "BoazJachinE Co.");
  }
}

void OLED_MENUMAIN(void)//   ������
{
  Clear_Screen(1);
    
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,0,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,0,Font_Menu_Fir);  
    LCD_P16x16Ch(32,0,1,Font_Menu_Fir); 
    LCD_P16x16Ch(48,0,2,Font_Menu_Fir);  
    LCD_P16x16Ch(64,0,3,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(24, 0, "SmartLock");
  }
}

void OLED_INPUT_UNLOCK(void)  // ��������
{ 
  if( sys_config.ChiOrEng )
  {
    Clear_Screen(0); 
    LCD_P16x16Ch(0,0,12,Font_Menu_Fir); 
    LCD_P16x16Ch(16,0,13,Font_Menu_Fir);  
    LCD_P16x16Ch(32,0,14,Font_Menu_Fir);  
    LCD_P16x16Ch(48,0,15,Font_Menu_Fir); 
  }
  else
  {
    LCD_P8x16Str(0, 0, "Enter Password  ");
  }
}

void OLED_OPENGATE_SHOW(void)  // ��֤ͨ��
{
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,16,Font_Menu_Fir); 
    LCD_P16x16Ch(16,0,17,Font_Menu_Fir);  
    LCD_P16x16Ch(32,0,18,Font_Menu_Fir);  
    LCD_P16x16Ch(48,0,19,Font_Menu_Fir); 
  }
  else
  {
    
  }
}

void OLED_INPUTADMIN_PSWD(void)  // ���������Ա����
{
  Clear_Screen(2);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,11,Font_Menu_Fir); 
    LCD_P16x16Ch(16,0,12,Font_Menu_Fir); 
    LCD_P16x16Ch(32,0,13,Font_Menu_Fir);
    
    LCD_P16x16Ch(48,0,22,Font_Menu_Fir);  
    LCD_P16x16Ch(64,0,23,Font_Menu_Fir); 
    LCD_P16x16Ch(80,0,24,Font_Menu_Fir);
    
    LCD_P16x16Ch(96,0,14,Font_Menu_Fir);  
    LCD_P16x16Ch(112,0,15,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 0, "Enter MGR PW    ");
  }
}

void OLED_INPUT_FALSE(void)  // ���������������
{
  Clear_Screen(2);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,14,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,15,Font_Menu_Fir);
    LCD_P16x16Ch(32,0,29,Font_Menu_Fir);  
    LCD_P16x16Ch(48,0,30,Font_Menu_Fir);
    LCD_P16x16Ch(64,0,31,Font_Menu_Fir);  
    LCD_P16x16Ch(80,0,32,Font_Menu_Fir);
    LCD_P16x16Ch(96,0,12,Font_Menu_Fir);  
    LCD_P16x16Ch(112,0,13,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 0, "Enter PW Again  ");
  }
}

void OLED_INADMIN_MODE(void)    // ����Աģʽ
{
//  LCD_P16x16Ch(0,0,20,Font_Menu_Fir);  
//  LCD_P16x16Ch(16,0,21,Font_Menu_Fir);
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,22,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,23,Font_Menu_Fir);
    LCD_P16x16Ch(32,0,24,Font_Menu_Fir);  
    LCD_P16x16Ch(48,0,25,Font_Menu_Fir);
    LCD_P16x16Ch(64,0,26,Font_Menu_Fir);  
  }
  else
  {
    LCD_P8x16Str(0, 0, "MGR Mode        ");
  }
}

void OLED_OUTADMIN_MODE(void)    // �˳�����Աģʽ
{
//  LCD_P16x16Ch(0,0,20,Font_Menu_Fir);  
//  LCD_P16x16Ch(16,0,21,Font_Menu_Fir);
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,27,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,28,Font_Menu_Fir);
    LCD_P16x16Ch(32,0,22,Font_Menu_Fir);  
    LCD_P16x16Ch(48,0,23,Font_Menu_Fir);
    LCD_P16x16Ch(64,0,24,Font_Menu_Fir);  
    LCD_P16x16Ch(80,0,25,Font_Menu_Fir);
    LCD_P16x16Ch(96,0,26,Font_Menu_Fir);  
  }
  else
  {
    LCD_P8x16Str(0, 0, "Out MGR Mode    ");
  }
}

void OLED_ADDPHONE_MODE(void)   //����ֻ��û���
{
  Clear_Screen(2);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,33,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,34,Font_Menu_Fir);
    LCD_P16x16Ch(32,2,37,Font_Menu_Fir);  
    LCD_P16x16Ch(48,2,38,Font_Menu_Fir);
    LCD_P16x16Ch(64,2,41,Font_Menu_Fir);  
    LCD_P16x16Ch(80,2,42,Font_Menu_Fir);
    LCD_P16x16Ch(96,2,43,Font_Menu_Fir); 
    LCD_P16x16Ch(112,2,0,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 2, "Add Phone User ?");
  }
}


void OLED_INADDPHONE_MODE(void)   //�������ģʽ
{
  Clear_Screen(0);
  
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,20,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,21,Font_Menu_Fir);
    
    LCD_P16x16Ch(32,0,33,Font_Menu_Fir);  
    LCD_P16x16Ch(48,0,34,Font_Menu_Fir);
    
    LCD_P16x16Ch(64,0,46,Font_Menu_Fir);  
    LCD_P16x16Ch(80,0,47,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 0, "To Add A Mode   ");
  }
}

void OLED_ADDPHONE_ENTER(void)  // ȷ��
{
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,48,Font_Menu_Fir);
    LCD_P16x16Ch(16,2,49,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 2, "OK");
  }
}

void OLED_ADDPHONE_CANCEL(void) // ȡ��
{
//  Clear_Screen(2);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(96,2,50,Font_Menu_Fir);
    LCD_P16x16Ch(112,2,51,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(80, 2, "Cancle");
  }
}

void OLED_ADDPHONE_SUCC(uint8 Num)   // ������ֻ��û�xx
{
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,33,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,34,Font_Menu_Fir);
    LCD_P16x16Ch(32,2,59,Font_Menu_Fir); 
    LCD_P16x16Ch(48,2,37,Font_Menu_Fir);  
    LCD_P16x16Ch(64,2,38,Font_Menu_Fir);
    LCD_P16x16Ch(80,2,41,Font_Menu_Fir);  
    LCD_P16x16Ch(96,2,42,Font_Menu_Fir);  
    LCD_P16x16Ch(112,2,Num,Font_Menu_Num);
  }
  else
  {
    LCD_P8x16Str(0, 2, "User   Add OK\'d ");
    LCD_P16x16Ch(36,2,Num,Font_Menu_Num);
  }
}

void OLED_ADDPHONE_FAIL(void)   // ���ʧ��
{
  Clear_Screen(2);
   
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,33,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,34,Font_Menu_Fir);
    
    LCD_P16x16Ch(32,2,57,Font_Menu_Fir); 
    LCD_P16x16Ch(48,2,58,Font_Menu_Fir); 
  }
  else
  {
    LCD_P8x16Str(0, 0, "User Add Failed ");
  }
}

void OLED_CHANGEADMIN_MODE(void)   //�޸Ĺ���Ա���룿
{
  Clear_Screen(2);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,44,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,45,Font_Menu_Fir);
    
    LCD_P16x16Ch(32,2,22,Font_Menu_Fir);  
    LCD_P16x16Ch(48,2,23,Font_Menu_Fir); 
    LCD_P16x16Ch(64,2,24,Font_Menu_Fir);
    
    LCD_P16x16Ch(80,2,14,Font_Menu_Fir);  
    LCD_P16x16Ch(96,2,15,Font_Menu_Fir);
    LCD_P16x16Ch(112,2,43,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 2, "Modify MGR PW ? ");
  }
}

void OLED_ONENEWADMIN_PSWD(void) // �����¹���Ա����
{
  Clear_Screen(2);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,12,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,13,Font_Menu_Fir);
    
    LCD_P16x16Ch(32,0,32,Font_Menu_Fir);  
    LCD_P16x16Ch(48,0,22,Font_Menu_Fir); 
    LCD_P16x16Ch(64,0,23,Font_Menu_Fir);
    
    LCD_P16x16Ch(80,0,24,Font_Menu_Fir);  
    LCD_P16x16Ch(96,0,14,Font_Menu_Fir);
    LCD_P16x16Ch(112,0,15,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 0, "Enter New MGR PW");
  }
}

void OLED_SECNEWADMIN_PSWD(void) // �ٴ���������
{
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,53,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,54,Font_Menu_Fir);
    LCD_P16x16Ch(32,0,12,Font_Menu_Fir); 
    LCD_P16x16Ch(48,0,13,Font_Menu_Fir);  
    LCD_P16x16Ch(64,0,14,Font_Menu_Fir);
    LCD_P16x16Ch(80,0,15,Font_Menu_Fir); 
  }
  else
  {
    LCD_P8x16Str(0, 0, "Enter PW Again  ");
  }
}

void OLED_NEWADMIN_SUCC(void)   // ������ȷ�޸ĳɹ�
{
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,14,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,15,Font_Menu_Fir);  
    LCD_P16x16Ch(32,0,60,Font_Menu_Fir);  // ��
    LCD_P16x16Ch(48,0,48,Font_Menu_Fir);  // ȷ
    LCD_P16x16Ch(64,0,44,Font_Menu_Fir);  
    LCD_P16x16Ch(80,0,45,Font_Menu_Fir);  
    LCD_P16x16Ch(96,0,55,Font_Menu_Fir);
    LCD_P16x16Ch(112,0,56,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 0, "PW Modify Succe ");
  }
}

void OLDE_NEWADMIN_FAIL(void)   // ��������޸�ʧ��
{
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,14,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,15,Font_Menu_Fir);  
    LCD_P16x16Ch(32,0,29,Font_Menu_Fir);  // ��
    LCD_P16x16Ch(48,0,30,Font_Menu_Fir);  // ��
    LCD_P16x16Ch(64,0,44,Font_Menu_Fir);  
    LCD_P16x16Ch(80,0,45,Font_Menu_Fir);  
    LCD_P16x16Ch(96,0,57,Font_Menu_Fir);
    LCD_P16x16Ch(112,0,58,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 0, "PW Modify Failed");
  }
}

void OLED_ADDKEYS_MODE(void)   //��Ӱ����û���
{
  Clear_Screen(2);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,33,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,34,Font_Menu_Fir);
    
    LCD_P16x16Ch(32,2,39,Font_Menu_Fir);  
    LCD_P16x16Ch(48,2,40,Font_Menu_Fir);
    
    LCD_P16x16Ch(64,2,41,Font_Menu_Fir);  
    LCD_P16x16Ch(80,2,42,Font_Menu_Fir);
    LCD_P16x16Ch(96,2,43,Font_Menu_Fir); 
    LCD_P16x16Ch(112,2,0,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 2, "Add PW User ? ");
  }
}

void OLED_ONENEWGENERAL_PSWD(void) // ��������
{
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,12,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,13,Font_Menu_Fir);
    LCD_P16x16Ch(32,0,14,Font_Menu_Fir); 
    LCD_P16x16Ch(48,0,15,Font_Menu_Fir);   
  }
  else
  {
    LCD_P8x16Str(0, 0, "Enter Password  ");
  }
}

void OLED_SECNEWGENERAL_PSWD(void) // �ٴ���������
{
  Clear_Screen(1);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,53,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,54,Font_Menu_Fir);
    LCD_P16x16Ch(32,0,12,Font_Menu_Fir); 
    LCD_P16x16Ch(48,0,13,Font_Menu_Fir);  
    LCD_P16x16Ch(64,0,14,Font_Menu_Fir);
    LCD_P16x16Ch(80,0,15,Font_Menu_Fir); 
  }
  else
  {
    LCD_P8x16Str(0, 0, "Enter PW Again  ");
  }
}

void OLED_ADDGENERAL_SUCC(uint8 Num)   // �û�xx��ӳɹ�
{
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,41,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,42,Font_Menu_Fir);  
    LCD_P16x16Ch(32,0,Num,Font_Menu_Num);  
    LCD_P16x16Ch(48,0,33,Font_Menu_Fir);  
    LCD_P16x16Ch(64,0,34,Font_Menu_Fir);  
    LCD_P16x16Ch(80,0,55,Font_Menu_Fir);  
    LCD_P16x16Ch(96,0,56,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 0, "User   Add OK\'d ");
    LCD_P16x16Ch(36,0,Num,Font_Menu_Num);
  }
}

void OLED_ADDGENERAL_FAIL(void)   //  ����������ʧ��
{
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,14,Font_Menu_Fir);  
    LCD_P16x16Ch(16,0,15,Font_Menu_Fir);  
    LCD_P16x16Ch(32,0,29,Font_Menu_Fir);  // ��
    LCD_P16x16Ch(48,0,30,Font_Menu_Fir);  // ��
    LCD_P16x16Ch(64,0,33,Font_Menu_Fir);  
    LCD_P16x16Ch(80,0,34,Font_Menu_Fir);  
    LCD_P16x16Ch(96,0,57,Font_Menu_Fir);
    LCD_P16x16Ch(112,0,58,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 0, "PW Err Add Fail ");
  }
}

void OLED_DELPHONE_MODE(void)   //ɾ���ֻ��û���
{
  Clear_Screen(2);
   
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,35,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,36,Font_Menu_Fir);
    
    LCD_P16x16Ch(32,2,37,Font_Menu_Fir);  
    LCD_P16x16Ch(48,2,38,Font_Menu_Fir);
    
    LCD_P16x16Ch(64,2,41,Font_Menu_Fir);  
    LCD_P16x16Ch(80,2,42,Font_Menu_Fir);
    LCD_P16x16Ch(96,2,43,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 2, "Erase Phone Usr?");
  }
}

void OLED_DELPHONE_NUM(uint8 Num)   //ɾ���ֻ��û�xx��
{
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,35,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,36,Font_Menu_Fir);
    
    LCD_P16x16Ch(32,2,37,Font_Menu_Fir);  
    LCD_P16x16Ch(48,2,38,Font_Menu_Fir);
    
    LCD_P16x16Ch(64,2,41,Font_Menu_Fir);  
    LCD_P16x16Ch(80,2,42,Font_Menu_Fir);
    LCD_P16x16Ch(96,2,Num,Font_Menu_Num); 
    LCD_P16x16Ch(112,2,43,Font_Menu_Fir); 
  }
  else
  {
    LCD_P8x16Str(0, 2, "Erase PH Usr   ?");
    LCD_P16x16Ch(100,2,Num,Font_Menu_Num);
  }
}

void OLED_DELPHONE_SUCC(void) // ɾ���ɹ�
{
  Clear_Screen(2);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,35,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,36,Font_Menu_Fir);
    LCD_P16x16Ch(32,2,55,Font_Menu_Fir);
    LCD_P16x16Ch(48,2,56,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 2, "Erase Succeed   ");
  }
}

void OLED_DELKEYS_MODE(void)   //ɾ�������û���
{
  Clear_Screen(2);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,35,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,36,Font_Menu_Fir);
    
    LCD_P16x16Ch(32,2,39,Font_Menu_Fir);  
    LCD_P16x16Ch(48,2,40,Font_Menu_Fir);
    
    LCD_P16x16Ch(64,2,41,Font_Menu_Fir);  
    LCD_P16x16Ch(80,2,42,Font_Menu_Fir);
    LCD_P16x16Ch(96,2,43,Font_Menu_Fir); 
  }
  else
  {
    LCD_P8x16Str(0, 2, "Erase PW User ? ");
  }
}

void OLED_DELKEYS_NUM(uint8 Num)   //ɾ�������û�xx��
{
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,35,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,36,Font_Menu_Fir);
    
    LCD_P16x16Ch(32,2,39,Font_Menu_Fir);  
    LCD_P16x16Ch(48,2,40,Font_Menu_Fir);
    
    LCD_P16x16Ch(64,2,41,Font_Menu_Fir);  
    LCD_P16x16Ch(80,2,42,Font_Menu_Fir);
    
    LCD_P16x16Ch(96,2,Num,Font_Menu_Num); 
    LCD_P16x16Ch(112,2,43,Font_Menu_Fir);  
  }
  else
  {
    LCD_P8x16Str(0, 2, "Erase PW Usr   ?");
    LCD_P16x16Ch(100,2,Num,Font_Menu_Num); 
  }
}

void OLED_DELKEYS_SUCC(void) // ɾ���ɹ�
{
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,35,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,36,Font_Menu_Fir);
    LCD_P16x16Ch(32,2,55,Font_Menu_Fir);
    LCD_P16x16Ch(48,2,56,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 2, "Erase Succeed   ");
  }
}

void OLED_LANGUAGE_CHOICE(void)  // ����ѡ��
{
  
  if( sys_config.ChiOrEng )
  {
    Clear_Screen(2);
    LCD_P16x16Ch(0,2,74,Font_Menu_Fir);  
    LCD_P16x16Ch(16,2,75,Font_Menu_Fir);
    
    LCD_P16x16Ch(32,2,76,Font_Menu_Fir);  
    LCD_P16x16Ch(48,2,77,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 2, "Language Choice ");
  }
}

void OLED_CHINA_ENGLISH(void)
{
  LCD_P16x16Ch(16,0,78,Font_Menu_Fir);
  LCD_P16x16Ch(32,0,79,Font_Menu_Fir); 
  LCD_P16x16Ch(48,0,0,Font_Menu_Fir);
  LCD_P16x16Ch(64,0,0,Font_Menu_Fir); 
  LCD_P16x16Ch(80,0,0,Font_Menu_Fir);
  LCD_P16x16Ch(96,0,0,Font_Menu_Fir); 
  LCD_P16x16Ch(112,0,0,Font_Menu_Fir);
  LCD_P8x16Str(16, 2, "English       ");
  if( sys_config.Language_Choice )
  { 
    LCD_P8x16Str(0, 0, "> ");
    LCD_P8x16Str(0, 2, "  ");
  }
  else
  {
    LCD_P8x16Str(0, 0, "  ");
    LCD_P8x16Str(0, 2, "> ");
  }
}

void OLED_INHIDDEN_PSWD( uint8 index ) // �������� * �Ŵ���
{
  if( index <= 7)
    LCD_P16x16Ch(index*16, 2, 10, Font_Menu_Num); 
}

void OLED_INDISPLAY_PSWD(uint8 index, uint8 Num) // ������ʾ 
{
  if( index <= 7)
    LCD_P16x16Ch(index*16, 2, Num, Font_Menu_Num); 
}

void OLED_MAINMENU_NUM(uint8 Num)
{
  switch(Num)
  {
    case 2:     // ����ֻ���
      OLED_ADDPHONE_MODE();
    break;
    case 3:     // �޸Ĺ���Ա���룿
      OLED_CHANGEADMIN_MODE();
    break;      
    case 4:     // ��Ӱ����û���
      OLED_ADDKEYS_MODE();
    break;      
    case 5:     // ɾ���ֻ��û���
      OLED_DELPHONE_MODE();
    break;      
      
    case 6:     // ɾ�������û���
      OLED_DELKEYS_MODE();
    break;
    case 7:     // ����ѡ��
      OLED_LANGUAGE_CHOICE();
    break;
    default:
    
    break;
  }
}

void OLED_USER_OVER(void)        // �û�����
{
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,41,Font_Menu_Fir);
    LCD_P16x16Ch(16,0,42,Font_Menu_Fir); 
    
    LCD_P16x16Ch(32,0,61,Font_Menu_Fir);  
    LCD_P16x16Ch(48,0,62,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 0, "User-Full");
  }
}

void OLED_USER_EMPTY(void)        // δ����û�
{
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,0,63,Font_Menu_Fir);
    LCD_P16x16Ch(16,0,33,Font_Menu_Fir); 
    LCD_P16x16Ch(32,0,34,Font_Menu_Fir);  
    LCD_P16x16Ch(48,0,41,Font_Menu_Fir);
    LCD_P16x16Ch(64,0,42,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 0, "Non-User");
  }
}

void OLED_OPENING_LOCK(void) // ������
{
  Clear_Screen(2);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,64,Font_Menu_Fir);
    LCD_P16x16Ch(16,2,3,Font_Menu_Fir); 
    LCD_P16x16Ch(32,2,65,Font_Menu_Fir);  
    LCD_P16x16Ch(48,2,66,Font_Menu_Fir);
    LCD_P16x16Ch(64,2,66,Font_Menu_Fir);
    LCD_P16x16Ch(80,2,66,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 2, "Unlocking....   ");
  }
}

void OLED_BATTERY_EMPTY(void)//��������
{
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(64,0,67,Font_Menu_Fir); 
    LCD_P16x16Ch(80,0,68,Font_Menu_Fir);  
    LCD_P16x16Ch(96,0,69,Font_Menu_Fir);
    LCD_P16x16Ch(112,0,70,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(56, 0, "Low-Power");
  }
}

void OLED_INIT_SHOW(void) //��ʼ���С�����
{
  Clear_Screen(0);
  
  if( sys_config.ChiOrEng )
  {
    LCD_P16x16Ch(0,2,71,Font_Menu_Fir);
    LCD_P16x16Ch(16,2,72,Font_Menu_Fir); 
    LCD_P16x16Ch(32,2,73,Font_Menu_Fir);  
    LCD_P16x16Ch(48,2,65,Font_Menu_Fir);
    LCD_P16x16Ch(64,2,66,Font_Menu_Fir);
    LCD_P16x16Ch(80,2,66,Font_Menu_Fir);
    LCD_P16x16Ch(96,2,66,Font_Menu_Fir);
  }
  else
  {
    LCD_P8x16Str(0, 2, "Initializing....");
  }

}