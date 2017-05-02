#ifndef _OLED_MENU_H_
#define _OLED_MENU_H_
#include "hal_types.h"
#include "simpleble.h"
void OLED_Welcom(void);

void OLED_INPUT_UNLOCK(void);                   // ��������
void OLED_OPENGATE_SHOW(void);                  // ��֤ͨ��
void OLED_INPUTADMIN_PSWD(void);                    // ���������Ա����
void OLED_INPUT_FALSE(void);                    // ���������������
void OLED_INADMIN_MODE(void);                   // ����Աģʽ
void OLED_OUTADMIN_MODE(void);                  // �˳�����Աģʽ
void OLED_ADDPHONE_MODE(void);                  // ����ֻ��û���
void OLED_INADDPHONE_MODE(void);                // �������ģʽ
void OLED_ADDPHONE_ENTER(void);                 // ȷ��
void OLED_ADDPHONE_CANCEL(void);                // ȡ��
void OLED_ADDPHONE_SUCC(uint8 Num);             // ������ֻ��û�xx
void OLED_ADDPHONE_FAIL(void);                  // ���ʧ��
void OLED_CHANGEADMIN_MODE(void);               // �޸Ĺ���Ա���룿
void OLED_ONENEWADMIN_PSWD(void);               // �����¹���Ա����
void OLED_SECNEWADMIN_PSWD(void);               // �ٴ���������
void OLED_NEWADMIN_SUCC(void);                  // ������ȷ�޸ĳɹ�
void OLDE_NEWADMIN_FAIL(void);                  // ��������޸�ʧ��
void OLED_ADDKEYS_MODE(void);                   // ��Ӱ����û���
void OLED_ONENEWGENERAL_PSWD(void);             // ��������
void OLED_SECNEWGENERAL_PSWD(void);             // �ٴ���������
void OLED_ADDGENERAL_SUCC(uint8 Num);           // �û�xx��ӳɹ�
void OLED_ADDGENERAL_FAIL(void);                // ����������ʧ��
void OLED_DELPHONE_MODE(void);                  // ɾ���ֻ��û���
void OLED_DELPHONE_NUM(uint8 Num);              // ɾ���ֻ��û�xx��
void OLED_DELPHONE_SUCC(void);                  // ɾ���ɹ�
void OLED_DELKEYS_MODE(void);                   // ɾ�������û���
void OLED_DELKEYS_NUM(uint8 Num);               // ɾ�������û�xx��
void OLED_DELKEYS_SUCC(void);                   // ɾ���ɹ�
void OLED_MENUMAIN(void);                       // ������

void OLED_DISPLAY_ONOFF(bool OnOff);            // ����  ����


void OLED_INDISPLAY_PSWD(uint8 index, uint8 Num); // ������ʾ 
void OLED_INHIDDEN_PSWD( uint8 index ); // �������� * �Ŵ���

void OLED_USER_OVER(void);                      // �û�����
void OLED_USER_EMPTY(void);                     // δ����û�
void OLED_MAINMENU_NUM(uint8 Num);              // ���˵�ѡ��
void OLED_BATTERY_EMPTY(void);                  //��������
void OLED_OPENING_LOCK(void);                   // ������
void OLED_INIT_SHOW(void);                      //��ʼ���С�����

void OLED_CHINA_ENGLISH(void);
void OLED_LANGUAGE_CHOICE(void);                // ����ѡ��
#endif