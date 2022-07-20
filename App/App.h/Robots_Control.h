#ifndef ROBOTS_CONTROL_H
#define ROBOTS_CONTROL_H

#include "main.h"

#define RedSentry 7
#define BlueSentry 107

#define Stop_game 0

#define Robots_Control_HOOK_FUN {\
	Update_DR16_RobotStatus,\
	Update_Vision_RobotStatus,\
	Updata_Control_RobotStatus,\
	Updata_Outpost_RobotStatus,\
	Updata_Dropblood_RobotStatus,\
	Updata_Frenzytimes_RobotStatus,\
	Updata_Heat_RobotStatus,\
	Updata_edBullet_RobotStatus,\
	Updata_Snipetime_RobotStatus,\
	Updata_Direct_RobotStatus,\
	Init_RobotStatus,\
	Updata_RobotStatus\
}

/* ------------- 机器人模式 枚举变量 ---------------- */

extern uint8_t ChassisDisable_Debug;

/* 主条件 */
/* DR16 各个键位的枚举 */
//一级控制
typedef enum
{
	Strike = 0, //罢工
	Remote,		//遥控
	Automation	//自动
} Master_Control_e;
//二级控制
typedef enum
{
	chassis_Enable = 0, //底盘
	c_Cloud_Enable,		//底盘+云台
	c_c_Autoaim_Enable	//底盘+云台+自瞄
} Slave_Control_e;

/* 子条件 */
//DR16 枚举
typedef enum
{
	Truce = 0, //停火
	Onslaught, //猛攻
	Sag		   //萎靡
} Fire_Control_e;
//视觉 枚举
typedef enum
{
	AutoAim_Enable = 0, //自瞄开启
	AutoAim_Disable		//自瞄失能
} Vision_Control_e;
//裁判系统下 枚举
//掉血情况：控制底盘的运动模式
typedef enum
{
	no_Dropblood = 0, //无掉血
	small_Dropblood,  //小掉血
	big_Dropblood	  //大出血
} Harm_Control_e;
//前哨站状态
typedef enum
{
	surviving = 0, //幸存
	Destroyed = 1, //被摧毁
	Nolonger = 2  //不复存在
} Outpost_Status_e;
//热量
typedef enum
{
	Small_Enough = 0,	//小于100，可用大于200
	Greater_than_Danger //大于危险值
} Heat_Status_e;
typedef enum
{
	edBullet_inequacy = 0,	//未拨够
	edBullet_adequate,
}edBullet_e;
/* 直接控制 */
/* 模块化 */
//底盘
typedef enum
{
	cs_Disable = 0, //底盘失能
	R_cs_Common,	//底盘 遥控 普通
	A_cs_Pathway,	//底盘 自动 确定行程
	A_cs_Outpost,	//底盘 自动 前哨战幸存
	A_cs_Cruise,	//底盘 自动 巡航
	A_cs_Random,	//底盘 自动 随机运动
	A_cs_Frenzy,	//底盘 自动 暴走
	A_cs_Pursue		//底盘 自动 追击
} Chassis_Control_e;
//云台
typedef enum
{
	cd_Disable = 0, //云台失能
	cd_AutoAim,		//云台 遥控 自瞄
	R_cd_Common,	//云台 遥控 普通
	A_cd_Scan		//云台 自动 扫描
} Cloud_Control_e;
//攻击
typedef enum
{
	ak_Disable = 0,			   //熄火
	cease_Fire,				   //停火
	R_ak_Sag_Nooverheat,	   //遥控 萎靡+永不超热量(退弹)
	R_ak_Onslaught_Nooverheat, //遥控 猛攻+永不超热量
	A_ak_Nooverheat,		   //自动 永不超热量(自动默认猛攻)
	A_ak_Frequency,			   //自动 变频
	A_ak_Snipe				   //自动 狙击
} Attack_Control_e;

/* ------------------ 机器人间通用 ------------------------ */
//攻击对象
typedef enum
{
	ShootTarget_default = 0,
	ShootTarget_Self_aiming = 1,
	ShootTarget_BIG_WHEEL,
	ShootTarget_Sentry,
	ShootTarget_base
} AttackTarget_e;
//比赛模式
//队伍的颜色
typedef enum
{
	TeamColor_Blue,
	TeamColor_Red
} TeamColor_e;
//机器人类型
typedef enum
{
	Types_Hero,
	Types_Engineer,
	Types_Standard,
	Types_Aerial = 6,
	Types_Sentry = 7
} Types_e;
/* ---------------------------------------------------- */

/* -------------------- 结构体 ------------------------- */
//伤害结构体：
typedef struct
{
	uint16_t sample_time; //采样时间
	uint16_t real_HP;	  //当前血量
	uint16_t last_HP;	  //上一次血量
	struct critical_value //临界值（标准值）
	{
		uint16_t sample_time;	   //采样时间
		uint16_t NoDropBlood_time; //无掉血时间
		uint16_t small_Dropblood;  //小掉血量
		uint16_t big_Dropblood;	   //大出血量
	} critical_value_t;

} Harm_Control_t;
//底盘运动时间结构体
typedef struct
{
	uint16_t nodropblood_Times;	 //无掉血时间
	uint16_t enter_Frenzy_Times; //进入狂暴状态时间
	uint16_t exit_Frenzy_Times;	 //退出狂暴状态时间
	struct critical_time
	{
		uint16_t FrenzyContinue;
		uint16_t FrenzyExit;
	} critical_time_t;
} Move_Time_t;
typedef struct
{
	uint8_t real_Status; //这次状态
	uint8_t last_Status; //上次状态
	enum change_Status
	{
		Disable = 0,
		Enable = 1
	} change_Status;			//变化状态
	uint8_t Frenzy_Status;		//此次处于狂暴状态
	uint8_t last_Frenzy_Status; //上次处于狂暴状态
} Control_Status_t;
//追击模式的运动结构体
typedef struct
{
	uint8_t ing_Flag;
	uint8_t exit_Flag;
	uint32_t exit_times;
	struct exit_crash_time_t
	{
		uint32_t frontpart_time;  //退出的前半段时间
		uint32_t latterpart_time; //退出的后半段时间
		uint32_t entirely_time;	  //完全退出的时间
	} exit_crash_time_t;
	enum exit_Snipe_e
	{
		entirely_Flag = 0, //完全退出
		frontpart_Flag,	   //退出的前半段
		latterpart_Flag,   //退出的后半段
	} exit_status_e;
} Snipe_Status_t;

/* --------------- Robots 控制 ------------------ */
typedef struct
{
	/* ---------------------- 枚举 ------------------------- */
	/* 各级控制 */
	// DR16
	Master_Control_e Master_e; //一级控制
	Slave_Control_e Slave_e;   //二级控制

	Fire_Control_e Fire_e; //子级控制
	// 视觉
	Vision_Control_e Vision_e; //子级控制
	//裁判系统
	//伤害：
	Harm_Control_e Harm_e;
	//前哨站状态
	Outpost_Status_e Outpost_Status;
	//热量状态
	Heat_Status_e Heat_Status;
	//已拨弹数
	edBullet_e edBullet_Status;

	/* 直接控制 */
	Chassis_Control_e Chassis_e; //底盘
	Cloud_Control_e Cloud_e;	 //云台
	Attack_Control_e Attack_e;	 //攻击

	/* -------------------- 结构体 --------------------------- */
	Harm_Control_t Harm_t;
	Move_Time_t Move_Time;
	Control_Status_t Status_t;
	Snipe_Status_t Snipe_Status;

	/* ---------------- 机器人间通用的 --------------------------*/
	//攻击对象
	AttackTarget_e AttackTarget;
	//比赛相关
	TeamColor_e TeamColor; //我方的团队颜色。
	Types_e Types;		   //我方兵种

} Robots_Control_t;
extern Robots_Control_t Robots_Control;

typedef struct
{
	void (*Update_DR16_RobotStatus)(void);
	void (*Update_Vision_RobotStatus)(void);
	void (*Updata_Control_RobotStatus)(void);
	void (*Updata_Outpost_RobotStatus)(void);
	void (*Updata_Dropblood_RobotStatus)(void);
	void (*Updata_Frenzytimes_RobotStatus)(void);
	void (*Updata_Heat_RobotStatus)(void);
	void (*Updata_edBullet_RobotStatus)(void);
	void (*Updata_Snipetime_RobotStatus)(void);
	void (*Updata_Direct_RobotStatus)(void);
	void (*Init_RobotStatus)(void);
	void (*Updata_RobotStatus)(void);
}Robots_Control_FUN_t;
extern Robots_Control_FUN_t Robots_Control_FUN;

void Update_DR16_RobotStatus(void);
void Update_Vision_RobotStatus(void);
void Updata_Control_RobotStatus(void);
void Updata_Outpost_RobotStatus(void);
void Updata_Dropblood_RobotStatus(void);
void Updata_Frenzytimes_RobotStatus(void);
void Updata_Heat_RobotStatus(void);
void Updata_edBullet_RobotStatus(void);
void Updata_Snipetime_RobotStatus(void);
void Updata_Direct_RobotStatus(void);
void Init_RobotStatus(void);
void Updata_RobotStatus(void);

#endif
