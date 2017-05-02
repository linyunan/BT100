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

void OLED_MENUMAIN(void)//   ������
{
  Clear_Screen(1);
  LCD_P16x16Ch(0,0,0,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,0,Font_Menu_Fir);  
  LCD_P16x16Ch(32,0,1,Font_Menu_Fir); 
  LCD_P16x16Ch(48,0,2,Font_Menu_Fir);  
  LCD_P16x16Ch(64,0,3,Font_Menu_Fir);  
}

void OLED_INPUT_UNLOCK(void)  // ��������
{
//  LCD_P16x16Ch(0,0,11,Font_Menu_Fir); 
  Clear_Screen(0); 
  LCD_P16x16Ch(0,0,12,Font_Menu_Fir); 
  LCD_P16x16Ch(16,0,13,Font_Menu_Fir);  
  LCD_P16x16Ch(32,0,14,Font_Menu_Fir);  
  LCD_P16x16Ch(48,0,15,Font_Menu_Fir);  
//  LCD_P16x16Ch(64,0,15,Font_Menu_Fir); 
}

void OLED_OPENGATE_SHOW(void)  // ��֤ͨ��
{
  Clear_Screen(0);
  LCD_P16x16Ch(0,0,16,Font_Menu_Fir); 
  LCD_P16x16Ch(16,0,17,Font_Menu_Fir);  
  LCD_P16x16Ch(32,0,18,Font_Menu_Fir);  
  LCD_P16x16Ch(48,0,19,Font_Menu_Fir); 
}

void OLED_INPUTADMIN_PSWD(void)  // ���������Ա����
{
  Clear_Screen(2);
  LCD_P16x16Ch(0,0,11,Font_Menu_Fir); 
  LCD_P16x16Ch(16,0,12,Font_Menu_Fir); 
  LCD_P16x16Ch(32,0,13,Font_Menu_Fir);
  
  LCD_P16x16Ch(48,0,22,Font_Menu_Fir);  
  LCD_P16x16Ch(64,0,23,Font_Menu_Fir); 
  LCD_P16x16Ch(80,0,24,Font_Menu_Fir);
  
  LCD_P16x16Ch(96,0,14,Font_Menu_Fir);  
  LCD_P16x16Ch(112,0,15,Font_Menu_Fir);
}

void OLED_INPUT_FALSE(void)  // ���������������
{
  Clear_Screen(2);
  LCD_P16x16Ch(0,0,14,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,15,Font_Menu_Fir);
  LCD_P16x16Ch(32,0,29,Font_Menu_Fir);  
  LCD_P16x16Ch(48,0,30,Font_Menu_Fir);
  LCD_P16x16Ch(64,0,31,Font_Menu_Fir);  
  LCD_P16x16Ch(80,0,32,Font_Menu_Fir);
  LCD_P16x16Ch(96,0,12,Font_Menu_Fir);  
  LCD_P16x16Ch(112,0,13,Font_Menu_Fir);
}

void OLED_INADMIN_MODE(void)    // ����Աģʽ
{
//  LCD_P16x16Ch(0,0,20,Font_Menu_Fir);  
//  LCD_P16x16Ch(16,0,21,Font_Menu_Fir);
  Clear_Screen(0);
  LCD_P16x16Ch(0,0,22,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,23,Font_Menu_Fir);
  LCD_P16x16Ch(32,0,24,Font_Menu_Fir);  
  LCD_P16x16Ch(48,0,25,Font_Menu_Fir);
  LCD_P16x16Ch(64,0,26,Font_Menu_Fir);  
 
}

void OLED_OUTADMIN_MODE(void)    // �˳�����Աģʽ
{
//  LCD_P16x16Ch(0,0,20,Font_Menu_Fir);  
//  LCD_P16x16Ch(16,0,21,Font_Menu_Fir);
  Clear_Screen(0);
  LCD_P16x16Ch(0,0,27,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,28,Font_Menu_Fir);
  LCD_P16x16Ch(32,0,22,Font_Menu_Fir);  
  LCD_P16x16Ch(48,0,23,Font_Menu_Fir);
  LCD_P16x16Ch(64,0,24,Font_Menu_Fir);  
  LCD_P16x16Ch(80,0,25,Font_Menu_Fir);
  LCD_P16x16Ch(96,0,26,Font_Menu_Fir);  

}

void OLED_ADDPHONE_MODE(void)   //����ֻ��û���
{
  Clear_Screen(2);
  LCD_P16x16Ch(0,2,33,Font_Menu_Fir);  
  LCD_P16x16Ch(16,2,34,Font_Menu_Fir);
  LCD_P16x16Ch(32,2,37,Font_Menu_Fir);  
  LCD_P16x16Ch(48,2,38,Font_Menu_Fir);
  LCD_P16x16Ch(64,2,41,Font_Menu_Fir);  
  LCD_P16x16Ch(80,2,42,Font_Menu_Fir);
  LCD_P16x16Ch(96,2,43,Font_Menu_Fir); 
  LCD_P16x16Ch(112,2,0,Font_Menu_Fir);
}


void OLED_INADDPHONE_MODE(void)   //�������ģʽ
{
  Clear_Screen(0);
  
  LCD_P16x16Ch(0,0,20,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,21,Font_Menu_Fir);
  
  LCD_P16x16Ch(32,0,33,Font_Menu_Fir);  
  LCD_P16x16Ch(48,0,34,Font_Menu_Fir);
  
  LCD_P16x16Ch(64,0,46,Font_Menu_Fir);  
  LCD_P16x16Ch(80,0,47,Font_Menu_Fir);
  
}

void OLED_ADDPHONE_ENTER(void)  // ȷ��
{
  LCD_P16x16Ch(0,2,48,Font_Menu_Fir);
  LCD_P16x16Ch(16,2,49,Font_Menu_Fir);
}

void OLED_ADDPHONE_CANCEL(void) // ȡ��
{
//  Clear_Screen(2);
  LCD_P16x16Ch(96,2,50,Font_Menu_Fir);
  LCD_P16x16Ch(112,2,51,Font_Menu_Fir);
}

void OLED_ADDPHONE_SUCC(uint8 Num)   // ������ֻ��û�xx
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

void OLED_ADDPHONE_FAIL(void)   // ���ʧ��
{
  Clear_Screen(2);
  LCD_P16x16Ch(0,2,33,Font_Menu_Fir);  
  LCD_P16x16Ch(16,2,34,Font_Menu_Fir);
  
  LCD_P16x16Ch(32,2,57,Font_Menu_Fir); 
  LCD_P16x16Ch(48,2,58,Font_Menu_Fir);  
}

void OLED_CHANGEADMIN_MODE(void)   //�޸Ĺ���Ա���룿
{
  Clear_Screen(2);
  LCD_P16x16Ch(0,2,44,Font_Menu_Fir);  
  LCD_P16x16Ch(16,2,45,Font_Menu_Fir);
  
  LCD_P16x16Ch(32,2,22,Font_Menu_Fir);  
  LCD_P16x16Ch(48,2,23,Font_Menu_Fir); 
  LCD_P16x16Ch(64,2,24,Font_Menu_Fir);
  
  LCD_P16x16Ch(80,2,14,Font_Menu_Fir);  
  LCD_P16x16Ch(96,2,15,Font_Menu_Fir);
  LCD_P16x16Ch(112,2,43,Font_Menu_Fir);
}

void OLED_ONENEWADMIN_PSWD(void) // �����¹���Ա����
{
  Clear_Screen(2);
  LCD_P16x16Ch(0,0,12,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,13,Font_Menu_Fir);
  
  LCD_P16x16Ch(32,0,32,Font_Menu_Fir);  
  LCD_P16x16Ch(48,0,22,Font_Menu_Fir); 
  LCD_P16x16Ch(64,0,23,Font_Menu_Fir);
  
  LCD_P16x16Ch(80,0,24,Font_Menu_Fir);  
  LCD_P16x16Ch(96,0,14,Font_Menu_Fir);
  LCD_P16x16Ch(112,0,15,Font_Menu_Fir);
}

void OLED_SECNEWADMIN_PSWD(void) // �ٴ���������
{
  Clear_Screen(0);
  LCD_P16x16Ch(0,0,53,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,54,Font_Menu_Fir);
  LCD_P16x16Ch(32,0,12,Font_Menu_Fir); 
  LCD_P16x16Ch(48,0,13,Font_Menu_Fir);  
  LCD_P16x16Ch(64,0,14,Font_Menu_Fir);
  LCD_P16x16Ch(80,0,15,Font_Menu_Fir); 
}

void OLED_NEWADMIN_SUCC(void)   // ������ȷ�޸ĳɹ�
{
  Clear_Screen(0);
  LCD_P16x16Ch(0,0,14,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,15,Font_Menu_Fir);  
  LCD_P16x16Ch(32,0,60,Font_Menu_Fir);  // ��
  LCD_P16x16Ch(48,0,48,Font_Menu_Fir);  // ȷ
  LCD_P16x16Ch(64,0,44,Font_Menu_Fir);  
  LCD_P16x16Ch(80,0,45,Font_Menu_Fir);  
  LCD_P16x16Ch(96,0,55,Font_Menu_Fir);
  LCD_P16x16Ch(112,0,56,Font_Menu_Fir);
}

void OLDE_NEWADMIN_FAIL(void)   // ��������޸�ʧ��
{
  Clear_Screen(0);
  LCD_P16x16Ch(0,0,14,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,15,Font_Menu_Fir);  
  LCD_P16x16Ch(32,0,29,Font_Menu_Fir);  // ��
  LCD_P16x16Ch(48,0,30,Font_Menu_Fir);  // ��
  LCD_P16x16Ch(64,0,44,Font_Menu_Fir);  
  LCD_P16x16Ch(80,0,45,Font_Menu_Fir);  
  LCD_P16x16Ch(96,0,57,Font_Menu_Fir);
  LCD_P16x16Ch(112,0,58,Font_Menu_Fir);
}

void OLED_ADDKEYS_MODE(void)   //��Ӱ����û���
{
  Clear_Screen(2);
  LCD_P16x16Ch(0,2,33,Font_Menu_Fir);  
  LCD_P16x16Ch(16,2,34,Font_Menu_Fir);
  
  LCD_P16x16Ch(32,2,39,Font_Menu_Fir);  
  LCD_P16x16Ch(48,2,40,Font_Menu_Fir);
  
  LCD_P16x16Ch(64,2,41,Font_Menu_Fir);  
  LCD_P16x16Ch(80,2,42,Font_Menu_Fir);
  LCD_P16x16Ch(96,2,43,Font_Menu_Fir); 
  LCD_P16x16Ch(112,2,0,Font_Menu_Fir);
}

void OLED_ONENEWGENERAL_PSWD(void) // ��������
{
  Clear_Screen(0);
  LCD_P16x16Ch(0,0,12,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,13,Font_Menu_Fir);
  LCD_P16x16Ch(32,0,14,Font_Menu_Fir); 
  LCD_P16x16Ch(48,0,15,Font_Menu_Fir);   
}

void OLED_SECNEWGENERAL_PSWD(void) // �ٴ���������
{
  Clear_Screen(1);
  LCD_P16x16Ch(0,0,53,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,54,Font_Menu_Fir);
  LCD_P16x16Ch(32,0,12,Font_Menu_Fir); 
  LCD_P16x16Ch(48,0,13,Font_Menu_Fir);  
  LCD_P16x16Ch(64,0,14,Font_Menu_Fir);
  LCD_P16x16Ch(80,0,15,Font_Menu_Fir); 
}

void OLED_ADDGENERAL_SUCC(uint8 Num)   // �û�xx��ӳɹ�
{
  Clear_Screen(0);
  LCD_P16x16Ch(0,0,41,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,42,Font_Menu_Fir);  
  LCD_P16x16Ch(32,0,Num,Font_Menu_Num);  
  LCD_P16x16Ch(48,0,33,Font_Menu_Fir);  
  LCD_P16x16Ch(64,0,34,Font_Menu_Fir);  
  LCD_P16x16Ch(80,0,55,Font_Menu_Fir);  
  LCD_P16x16Ch(96,0,56,Font_Menu_Fir);
}

void OLED_ADDGENERAL_FAIL(void)   //  ����������ʧ��
{
  Clear_Screen(0);
  LCD_P16x16Ch(0,0,14,Font_Menu_Fir);  
  LCD_P16x16Ch(16,0,15,Font_Menu_Fir);  
  LCD_P16x16Ch(32,0,29,Font_Menu_Fir);  // ��
  LCD_P16x16Ch(48,0,30,Font_Menu_Fir);  // ��
  LCD_P16x16Ch(64,0,33,Font_Menu_Fir);  
  LCD_P16x16Ch(80,0,34,Font_Menu_Fir);  
  LCD_P16x16Ch(96,0,57,Font_Menu_Fir);
  LCD_P16x16Ch(112,0,58,Font_Menu_Fir);
}

void OLED_DELPHONE_MODE(void)   //ɾ���ֻ��û���
{
  Clear_Screen(2);
  LCD_P16x16Ch(0,2,35,Font_Menu_Fir);  
  LCD_P16x16Ch(16,2,36,Font_Menu_Fir);
  
  LCD_P16x16Ch(32,2,37,Font_Menu_Fir);  
  LCD_P16x16Ch(48,2,38,Font_Menu_Fir);
  
  LCD_P16x16Ch(64,2,41,Font_Menu_Fir);  
  LCD_P16x16Ch(80,2,42,Font_Menu_Fir);
  LCD_P16x16Ch(96,2,43,Font_Menu_Fir); 
}

void OLED_DELPHONE_NUM(uint8 Num)   //ɾ���ֻ��û�xx��
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

void OLED_DELPHONE_SUCC(void) // ɾ���ɹ�
{
  Clear_Screen(2);
  LCD_P16x16Ch(0,2,35,Font_Menu_Fir);  
  LCD_P16x16Ch(16,2,36,Font_Menu_Fir);
  LCD_P16x16Ch(32,2,55,Font_Menu_Fir);
  LCD_P16x16Ch(48,2,56,Font_Menu_Fir);
}

void OLED_DELKEYS_MODE(void)   //ɾ�������û���
{
  Clear_Screen(2);
  LCD_P16x16Ch(0,2,35,Font_Menu_Fir);  
  LCD_P16x16Ch(16,2,36,Font_Menu_Fir);
  
  LCD_P16x16Ch(32,2,39,Font_Menu_Fir);  
  LCD_P16x16Ch(48,2,40,Font_Menu_Fir);
  
  LCD_P16x16Ch(64,2,41,Font_Menu_Fir);  
  LCD_P16x16Ch(80,2,42,Font_Menu_Fir);
  LCD_P16x16Ch(96,2,43,Font_Menu_Fir); 
}

void OLED_DELKEYS_NUM(uint8 Num)   //ɾ�������û�xx��
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

void OLED_DELKEYS_SUCC(void) // ɾ���ɹ�
{
  Clear_Screen(0);
  LCD_P16x16Ch(0,2,35,Font_Menu_Fir);  
  LCD_P16x16Ch(16,2,36,Font_Menu_Fir);
  LCD_P16x16Ch(32,2,55,Font_Menu_Fir);
  LCD_P16x16Ch(48,2,56,Font_Menu_Fir);
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
    default:
    
    break;
  }
}

void OLED_USER_OVER(void)        // �û�����
{
  Clear_Screen(0);
  LCD_P16x16Ch(0,0,41,Font_Menu_Fir);
  LCD_P16x16Ch(16,0,42,Font_Menu_Fir); 
  
  LCD_P16x16Ch(32,0,61,Font_Menu_Fir);  
  LCD_P16x16Ch(48,0,62,Font_Menu_Fir);

}

void OLED_USER_EMPTY(void)        // δ����û�
{
  Clear_Screen(0);
  LCD_P16x16Ch(0,0,63,Font_Menu_Fir);
  LCD_P16x16Ch(16,0,33,Font_Menu_Fir); 
  LCD_P16x16Ch(32,0,34,Font_Menu_Fir);  
  LCD_P16x16Ch(48,0,41,Font_Menu_Fir);
  LCD_P16x16Ch(64,0,42,Font_Menu_Fir);
}

void OLED_OPENING_LOCK(void) // ������
{
  Clear_Screen(2);
  LCD_P16x16Ch(0,2,64,Font_Menu_Fir);
  LCD_P16x16Ch(16,2,3,Font_Menu_Fir); 
  LCD_P16x16Ch(32,2,65,Font_Menu_Fir);  
  LCD_P16x16Ch(48,2,66,Font_Menu_Fir);
  LCD_P16x16Ch(64,2,66,Font_Menu_Fir);
  LCD_P16x16Ch(80,2,66,Font_Menu_Fir);
}

void OLED_BATTERY_EMPTY(void)//��������
{
  Clear_Screen(0);
  LCD_P16x16Ch(64,0,67,Font_Menu_Fir); 
  LCD_P16x16Ch(80,0,68,Font_Menu_Fir);  
  LCD_P16x16Ch(96,0,69,Font_Menu_Fir);
  LCD_P16x16Ch(112,0,70,Font_Menu_Fir);
}

void OLED_INIT_SHOW(void) //��ʼ���С�����
{
  Clear_Screen(0);
  LCD_P16x16Ch(0,2,71,Font_Menu_Fir);
  LCD_P16x16Ch(16,2,72,Font_Menu_Fir); 
  LCD_P16x16Ch(32,2,73,Font_Menu_Fir);  
  LCD_P16x16Ch(48,2,65,Font_Menu_Fir);
  LCD_P16x16Ch(64,2,66,Font_Menu_Fir);
  LCD_P16x16Ch(80,2,66,Font_Menu_Fir);
  LCD_P16x16Ch(96,2,66,Font_Menu_Fir);

}