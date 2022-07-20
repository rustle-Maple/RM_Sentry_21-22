#ifndef ATTACK_CONTROL_H
#define ATTACK_CONTROL_H

#include "main.h"
#include "User_typedef.h"
#include "Driver_Control.h"
#include "Heat_Control.h"
#include "Shoot_Control.h"

#define Attack_HOOK_FUN {\
	Interval_Attack,\
	FixedPoint_Snipe_Processing,\
	Fluctuant_Strafe_Processing,\
	Forever_NoOverHeat_Processing,\
	Do_not_Attack,\
	Attack_Init,\
	Attack_Processing\
}

//攻击变量
typedef struct
{
	/* 间隔拨弹*/
	uint32_t beginAttack_time;		//开启攻击时的时刻
	uint32_t ingAttack_time;		//打弹进行的时刻
	uint32_t times_between_Bullets; //每颗弹的间隔时间
	enum barrel_Select
	{
		Barrel1 = 0,
		Barrel2
	} barrel_Select; //枪管的选择

} Attack_t;
extern Attack_t Attack;
typedef struct
{
	void (*Interval_Attack)(void);
	void (*FixedPoint_Snipe_Processing)(void);
	void (*Fluctuant_Strafe_Processing)(void);
	void (*Forever_NoOverHeat_Processing)(void);
	void (*Do_not_Attack)(void);
	void (*Attack_Init)(void);
	void (*Attack_Processing)(void);
} Attack_FUN_t;
extern Attack_FUN_t Attack_FUN;

void Interval_Attack(void);
void FixedPoint_Snipe_Processing(void);
void Fluctuant_Strafe_Processing(void);
void Forever_NoOverHeat_Processing(void);
void Do_not_Attack(void);
void Attack_Init(void);
void Attack_Processing(void);

#endif
