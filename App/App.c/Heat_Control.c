#include "Heat_Control.h"

Barrel_Heat_t Barrel;
Barrel_Heat_FUN_t Barrel_Heat_FUN = Barrel_Heat_HOOK_FUN;

//用户计算枪管热量值，摆脱裁判系统的
void user_Calculate_Heat(void)
{
    //获取枪管的冷却值
    Barrel.judge_Cooling = ext_game_robot_state.data.shooter_id1_17mm_cooling_rate;
    if (Judge_Monitor.FPS == 0)
    {
        Barrel.judge_Cooling = Judge_Cooling_persec;
    }
    //判断是否已经拨出一颗弹
    if (Driver.finish_Flag == 1)
    {
        //拨出一颗弹即是热量+10
        Barrel.user_realHeat += 10;
				//清除拨单完成标志位
				Driver.finish_Flag = 0;
    }
    //热量冷却每1ms得缩减热量值
    Barrel.user_realHeat -= (float)Barrel.judge_Cooling / 1000.0f;      //热量的更新应该放在控制任务中，它优先级最高，时序才是正确的
    if (Judge_Monitor.FPS != 0)
    {
        //获取裁判系统热量的当前值
        Barrel.judge_realHeat = ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;
        //若裁判系统热量的当前值与用户热量的当前值差值大于20则做纠正
        if (abs(Barrel.user_realHeat - Barrel.judge_realHeat) > 20)
        {
            //则让裁判系统的值作为用户值
            Barrel.user_realHeat = (float)Barrel.judge_realHeat;
        }
    }
    //极值限制
    if (Barrel.user_realHeat > Heat_Max)
    {
        Barrel.user_realHeat = Heat_Max;
    }
    if (Barrel.user_realHeat < Heat_Min)
    {
        Barrel.user_realHeat = Heat_Min;
    }
}
