#ifndef HEAT_CONTROL_H
#define HEAT_CONTROL_H

#include "main.h"
#include "RM_JudgeSystem.h"
#include "Driver_Control.h"

#define Judge_Cooling_persec 50.0f
#define Heat_Max 320
#define Heat_Min 0
#define Bullet_Heat 10
#define Danger_Heat (Heat_Max - Bullet_Heat * 2)
#define Snipe_critical_Heat 20*10 

#define Barrel_Heat_HOOK_FUN {\
    user_Calculate_Heat\
}

typedef struct 
{
    uint16_t judge_realHeat;                //裁判系统的当前热量
    uint16_t judge_Cooling;                 //裁判系统的热量冷却值
    float    user_realHeat;                 //用户的当前热量值

}Barrel_Heat_t;
extern Barrel_Heat_t Barrel;
typedef struct 
{
    void (*user_Calculate_Heat)(void);
}Barrel_Heat_FUN_t;
extern Barrel_Heat_FUN_t Barrel_Heat_FUN;

void user_Calculate_Heat(void);

#endif

