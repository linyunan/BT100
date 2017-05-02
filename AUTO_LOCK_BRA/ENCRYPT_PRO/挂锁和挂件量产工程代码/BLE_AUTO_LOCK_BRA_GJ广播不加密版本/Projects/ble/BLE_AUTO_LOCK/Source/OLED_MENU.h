#ifndef _OLED_MENU_H_
#define _OLED_MENU_H_
#include "hal_types.h"
#include "simpleble.h"
void OLED_Welcom(void);

void OLED_INPUT_UNLOCK(void);                   // 输入密码
void OLED_OPENGATE_SHOW(void);                  // 验证通过
void OLED_INPUTADMIN_PSWD(void);                    // 请输入管理员密码
void OLED_INPUT_FALSE(void);                    // 密码错误重新输入
void OLED_INADMIN_MODE(void);                   // 管理员模式
void OLED_OUTADMIN_MODE(void);                  // 退出管理员模式
void OLED_ADDPHONE_MODE(void);                  // 添加手机用户？
void OLED_INADDPHONE_MODE(void);                // 进入添加模式
void OLED_ADDPHONE_ENTER(void);                 // 确定
void OLED_ADDPHONE_CANCEL(void);                // 取消
void OLED_ADDPHONE_SUCC(uint8 Num);             // 添加了手机用户xx
void OLED_ADDPHONE_FAIL(void);                  // 添加失败
void OLED_CHANGEADMIN_MODE(void);               // 修改管理员密码？
void OLED_ONENEWADMIN_PSWD(void);               // 输入新管理员密码
void OLED_SECNEWADMIN_PSWD(void);               // 再次输入密码
void OLED_NEWADMIN_SUCC(void);                  // 密码正确修改成功
void OLDE_NEWADMIN_FAIL(void);                  // 密码错误修改失败
void OLED_ADDKEYS_MODE(void);                   // 添加按键用户？
void OLED_ONENEWGENERAL_PSWD(void);             // 输入密码
void OLED_SECNEWGENERAL_PSWD(void);             // 再次输入密码
void OLED_ADDGENERAL_SUCC(uint8 Num);           // 用户xx添加成功
void OLED_ADDGENERAL_FAIL(void);                // 密码错误添加失败
void OLED_DELPHONE_MODE(void);                  // 删除手机用户？
void OLED_DELPHONE_NUM(uint8 Num);              // 删除手机用户xx？
void OLED_DELPHONE_SUCC(void);                  // 删除成功
void OLED_DELKEYS_MODE(void);                   // 删除按键用户？
void OLED_DELKEYS_NUM(uint8 Num);               // 删除按键用户xx？
void OLED_DELKEYS_SUCC(void);                   // 删除成功
void OLED_MENUMAIN(void);                       // 智能锁

void OLED_DISPLAY_ONOFF(bool OnOff);            // 关屏  开屏


void OLED_INDISPLAY_PSWD(uint8 index, uint8 Num); // 密码显示 
void OLED_INHIDDEN_PSWD( uint8 index ); // 密码隐藏 * 号代替

void OLED_USER_OVER(void);                      // 用户已满
void OLED_USER_EMPTY(void);                     // 未添加用户
void OLED_MAINMENU_NUM(uint8 Num);              // 主菜单选项
void OLED_BATTERY_EMPTY(void);                  //电量不足
void OLED_OPENING_LOCK(void);                   // 开锁中
void OLED_INIT_SHOW(void);                      //初始化中。。。

void OLED_CHINA_ENGLISH(void);
void OLED_LANGUAGE_CHOICE(void);                // 语言选择
#endif