#ifndef DRIVER_CONTROL_H
#define DRIVER_CONTROL_H

#include "main.h"
#include "stdbool.h"
#include "User_typedef.h"
#include "Robots_Control.h"
#include "PID_Position.h"
#include "M2006.h"

#define Driver_HOOK_FUN {\
    Driver_Reset,\
    DriverTorque_Detection,\
    Driver_Init,\
    Driver_Control,\
}

//由于哨兵的拨盘有8个弹位，且2006的转动比为1：36
#define Angle_Bulletper (8191 * 36 / 8)
#define Driver_number_max 250

//卡弹结构体
typedef struct
{
    uint8_t flag;            //开启卡弹标志位
    uint32_t ing_times;      //卡弹持续时间（用于若长时间处于卡弹状态那么就摇摇头（切换为扫描状态），将弹摇散）
    int16_t critical_Speed;  //卡弹的临界速度
    uint16_t times;          //卡弹的时间
    uint16_t critical_Times; //卡弹的临界时间
    int32_t Angle;           //卡弹时的角度
    int32_t delta_Angle;     //反卡弹的角度增量
    int32_t target_Angle;    //反卡弹的目标角度
    uint8_t dir_Flag;        //反卡弹的方向

} StuckBullet_t;

typedef struct
{
    M2006_t *driver_EM; //拨盘电机

    uint16_t will_Bullet_numbers; //要发射子弹数

    uint8_t begin_Flag; //开始拨弹的标志位

    int8_t turn_Dir; //旋转方向

    uint16_t ed_Bullet_numbers; //已拨出的弹数

		uint16_t last_ed_Bullet_numbers; //上一刻已经拨的弹数
	
    int16_t residue_Bullet_numbers; //剩余的弹丸数
	
    uint8_t finish_Flag; //拨弹完成标志位

    StuckBullet_t StuckBullet; //卡弹的结构体

} Driver_t;
extern Driver_t Driver;

typedef struct
{
    void (*Driver_Reset)(M2006_t *Driver_EM);
    bool (*DriverTorque_Detection)(Driver_t *driver);
    void (*Driver_Init)(void);
    void (*Driver_Control)(Driver_t *driver);
} Driver_FUN_t;
extern Driver_FUN_t Driver_FUN;

void Driver_Reset(M2006_t *Driver_EM);
bool DriverTorque_Detection(Driver_t *driver);
void Driver_Init(void);
void Driver_Control(Driver_t *driver);


#endif
