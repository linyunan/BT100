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
  uint8 Admin_Pswd[8];		        // 管理员密码	
//  uint8 General_Pswd[10][7];		// 用户密码
 
  uint8 Phone_User[10][8];
//  uint8 MY_KEY_CODE;
  uint8 MY_KEYCODE[16];
//  uint8 sys_InitStatus;				// 若给管理员APP 上传过则此位被标记为非零参数
  union
  {
    uint16 Recode_Addr_Fir;                  // 产生的随机存储地址
    uint8 Recode_Addr_F2[2];
  };
  union
  {
    uint16 Recode_Addr_Sec;                  // 产生的随机存储地址 
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
  uint8 At24c64_WR;               //  地址0x1FFF 是否被写入
  uint8 General_Pswd[10][9];		// 用户密码

  uint8 Admin_Phone_Flag;               // 管理员设置标志位， 0 为未设置手机管理员 ， 1为已经设置手机管理员
  
  uint8 Once_Single[10][9];			// 一次性密码只使用一次
  uint8 Once_OneDay[10][9];		// 一次性密码使用一天

  uint8 Once_OneWeek[10][9];		// 一次性密码使用一个星期

  uint8 Once_AMonth[10][9];		// 一次性密码使用一个月

  uint8 Phone_Num;                      // 已经存在的手机用户个数
  uint8 Keys_num;                       // 已经存在的按键用户个数
  
  union
  {
    uint8 DayLimit[10][4];
    uint32 Once_DayLimit[10][1];   		// 一次性密码一天期限时间限制存储空间
  };
  union
  {
    uint8 WeekLimit[10][4];
    uint32 Once_WeekLimit[10][1];		// 一次性密码一星期期限时间限制存储空间
  };
  union
  {
    uint8 MonthLimit[10][4];		// 一次性密码使用一个月
    uint32 Once_MonthLimit[10][1];	// 一次性密码一个月期限时间限制存储空间
  };
}SYS_24C64;

typedef struct
{
  uint8 open_DoorRec[3];           //   开门记录 0-19  超过19覆盖
  uint8 Open_Recode[15][9];		// 开门记录信息
  uint8 Open_Count;
}SYS_RECODE;

/*********************************************************************
 * global variable don't need save
 */
typedef struct 
{
  uint8 authoryMode;            // 权限模式（0: 无权限  1：手机管理员   2：手机普通用户）
  uint8 openDoorMode;           //  手机开门模式( 0； 未选择模式（默认自动）   1:  自动    2:  点亮手机屏幕)
  uint8 AddPhoneFlag;		// 下位机在添加模式(0；非连接模式    1. 进入添加模式   其他参数都? )

//  uint8 addPhoneNum;		// 增加第几个手机用户
//  uint8 addPswdNum;		// 增加第几个按键密码

  uint8 batteryPower[3];		// 电量读取
  uint8 openDoorTurn;           //  门锁开关转换
  uint8 AppOpenStatus;	        // APP 开门状态位    0 :  不开门  1 : 自动模式  2 : 点亮屏幕开门模式
  
  bool longButtonFlag;          //  长按标志
  uint8 genPswdFlag;		//  正常输入密码开门标志   0 : 为无操作，非零正常操作
  uint8 admPswdFlag;		//  进入管理员模式后选项标志位 1 : 添加手机，2 : 修改管理员密码 。。。
  uint8 MvInPswd[15];		//  最长10位虚位开门密码

  uint8 optionOutTime;	        //	操作超时
  uint8 menu_Num;		//	进入管理员模式后级数标志，增加一级自增加1
  uint8 menu_Index;		//	进入管理员模式后索引参数
  uint8 keyRecode_Temp[8];	        //	临时密码比较  输入两次密码的比较
//  uint8 tmRecode_Temp[6];         //  TM 卡临时存储比较
  
  uint8 sys_OptionFlag;		//    作为更新位右移 与的位标志
  uint8 appRecode_Temp[7];	//	存储 APP 下发的手机号信息	

  uint8 appUpdata_CRC;		//  上传数据CRC 校验码
  uint8 appDedata_CRC;		//  app 删除用户信息使用的CRC 校验码
  
  uint8 safety_Flag;            //   防止被随便乱输入密码，超过6次进入待机10分钟才可被唤醒
  uint32 time_Recode;            //   防乱按密码时间记录 10分钟
  uint8 safety_Rssi[2];     //   [0]  实时的RSSI   [1] 安全RSSI 值			
  
  uint8 sys_UpdataStatus;		//	系统用户信息是否有更新          0x00 : 不更新，0x01 : 管理员密码更新
  							//   0x02 : 手机用户 0x04 : 普通用户更新    0x08 :  TM 卡更新   0x10: 开门记录更新	
  uint8 sys_CurrentUp;      // 对前面一个上传的是什么用户信息做记录 ，以便错误重传
  uint8 app_DownChange;	//  APP 下发更改下位机参数标志位  0x01  (管理员、手机用户)  0x02 (TM 卡，普通按键用户和一次性)
  
  uint8 batteryNum;             // 电量读取次数
//  uint8 batteryNumOut;      // 读取无效次数
  uint16 batteryADC;             // ADC值
  uint8 batteryPer;             // 百分比
  uint8 mainMenuUpDown;        // 上下选择
  uint8 delPhoneNum;           // 删除手机用户
  uint8 delKeysNum;            // 删除按键用户
  uint8 MenuDelay;              // 延时显示无法操作
  uint8 MY_KEY_CODE;
  uint8 Battery_Empty;         
  uint8 Choice_Flag;
  
  uint8 Del_Phone_User[10][6];  // 待被删除的手机用户
  uint8 Del_Phone_Num;          // 待被删除的手机用户数量
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
