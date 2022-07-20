#include "Attack_Control.h"

//由于装甲板的检测频率最高是：20Hz,即50ms打1颗弹，即1s最多只能打20颗弹
//而哨兵的热量上限是320,打一颗弹所花的热量是10,即满热量最多是可以打32颗弹的，但32颗弹不能再1s内打完
//热量的冷却是10Hz的频率(即100ms)进行结算的,每个检测周期热量冷却 = 每秒冷却值(100)/10=10，则重新恢复满热量需要32周期，即3.2s(此值是理论上的,是枪口检测无延时的情况)

//枪口热量结算存在一个延迟，机器人主控读取到裁判系统的射速数据更新到热量数据更新这段时间，相差了 100ms 左右
//再打一发子弹就应该停止否则就会超热量了，然而这发子弹打出去后，热量还没更新，程序上就会认为还能多打一发，结果又多打了一发出去导致超热量
//对于这个延迟问题，如果相邻两发子弹打出去的间隔大于 100ms，即射频在 10 发以下，可以不用处理，只需要一个简单的策略即可把热量限制地很好，比如设定一个热量阈值，当热量接近阈值时就让拨盘停转。
//如果射频在每秒 10 发以上，如果不处理延迟问题，除非把热量的阈值设得很低（这样的话射频就很低），否则怎么限都一定会超热量
//热量限制策略：
//既然要对热量有所限制，那从一开始我们就应该对发射子弹做一个计划调控。
//例如，18年的热量限制：热量上限为 90，设定射速阈值为 20m/s，最多可以发射的子弹数等于热量上限减去当前热量再除以 20，即（90-0）/20，取整为 4。
//知道了当前允许发射的子弹数，还需要知道已经打出了几发子弹。
//这里可以通过裁判系统读取实时射速，若数据有更新就说明打出了一发子弹，以此累加
//当已打出的子弹等于允许打出的子弹时，就让拨弹电机停转。这样做就避免了由于热量结算延迟而多打一发子弹的问题。
//(实际上就是通过一个闭环嘛，即可以得到需要发射的弹丸数，又有反馈你已经发射的弹丸数)

//而后续什么时候把已发射子弹的计数清零，为下一次发射做准备呢？
//我们本可以认为当拨弹电机停转时，本次射击结束，已发射子弹数清零，
//但是如果这样做，由于热量延迟，当已发射子弹数达到了允许发射的子弹数时，拨弹电机停转，已发射子弹数清 0，但是此时热量还没有更新，允许发射的子弹数还是一两发，这样一来，又会有多打出一两发子弹的问题
//所以已发射子弹数清零的条件，除了拨弹电机停转，还应该在热量数据更新后，这里的做法可以是从打出一发子弹后开始计时，同时记下当时的热量，等待 100ms 后，将读取到的热量与记录下来的热量作差，若热量更新，此时的差值肯定约等于最后一发子弹的射速（别忘了 100ms 内已冷却的热量值）

//对于射击模式，目前比较普遍的发射模式有三种，单发（按一下打一发），连发（按住连发，松开停止），一键 n 连发（按一下连发 n 发）。
//一键 n 连发两种模式，看起来类似，其实还是有差别的，
//考虑到热量限制，连发更适用于命中把握不太高的场合，比如和对方步兵混战之类的，一排扫射过去命中几发也总比一串 n 连发全部打空，突然出现了机会却要等热量冷却要好；
//而一键 n 连发适用于命中把握比较高的场合，比如打基地，打英雄和哨兵的大装甲板。

//低射速会导致弹道很差，几乎只能贴脸肉搏，中远距离和高处的目标根本没法打。
//因此，如何在不降低太多射频的情况下，尽量提高射速以提升弹道也是一个值得思考的问题。

//攻击变量
Attack_t Attack;

//攻击函数指针挂钩
Attack_FUN_t Attack_FUN = Attack_HOOK_FUN;

//而对于上下云台不同的板子控制时，
//就必须以下云台为基准，然后通过板间通讯去让上云台will_Bullet_numbers = 1
//而且你需要去判断上下云台是否识别到同一块装甲板
//如何判断是识别到同一块装甲板？通过上下云台的识别到同一个点时有个固定的角度差值，若在此差值邻域内即可以认为识别到同个装甲板

//适合在两个枪管瞄准同一个目标的情况
//双枪管间隔的攻击
//只要在另一个间隔时间区间内并不是让另外一支枪管去发射，而是不做任何的处理就可以做到限频发射
void Interval_Attack(void)
{
    uint32_t spacing;
    //处于攻击失能模式，记录时间
    if (Robots_Control.Attack_e == ak_Disable || Robots_Control.Attack_e == cease_Fire)
    {
        Attack.beginAttack_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    else
    {
        //获取当前的时间
        Attack.ingAttack_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        //获取从开启发射时到现时刻的时间间隔
        spacing = Attack.ingAttack_time - Attack.beginAttack_time;
        //第1个时间间隔是留给左枪管
        if ((spacing / Attack.times_between_Bullets) % 2 == 0 && Attack.barrel_Select == Barrel1)
        {

            //第一个时间间隔，选择左枪管
            Driver.will_Bullet_numbers = 1; //发弹数为1
            Attack.barrel_Select = Barrel2; //重置为上枪管
        }
        //第2个时间间隔是留给右枪管
        else if ((spacing / Attack.times_between_Bullets) % 2 == 1 && Attack.barrel_Select == Barrel2)
        {
            //可以选择另外一只枪管发射，也可以不要
            Attack.barrel_Select = Barrel1; //重置上枪管
        }
    }
}
/* -----
    //把视觉的距离和底盘的移动模式作为激发条件
    //把热量和弹丸数作为子条件
-------- */

//发射模式一：定点狙击(以最高射速30m/s最高射频20Hz击打)
void FixedPoint_Snipe_Processing(void)
{
			//设置击打的频率
			Attack.times_between_Bullets = 25.0f / 2.0f; //即50ms打一颗
			//确定打击弹数
			Attack_FUN.Interval_Attack();
				
	//检测已发弹丸数
	Robots_Control_FUN.Updata_edBullet_RobotStatus();

}
//发射模式二：变频扫射
void Fluctuant_Strafe_Processing(void)
{
    //float Rate;
    //用幂函数比反比例函数的好处在于：
    //当从满热量恢复时，幂函数可以恢复到更多的热量才有可以发射的弹丸数，也即是可以有更多的热量下对应相同的射频
    double ex = 0.8;                  //幂数
    double a = 0, c = 0, numbers = 0; //a:为系数 c:为常数 numbers:为可发弹量
    //即是根据两个坐标点推出两个系数
    c = (double)Danger_Heat / 10;          //是弹热量为0即有满热量可用的时，对应的可射数量
    a = -(double)c / pow(Danger_Heat, ex); //根据发弹量为0时推出来的
    //当前热量可发射的弹数
    numbers = a * pow((double)Barrel.user_realHeat, ex) + c;
    if (numbers > 0)
    {
        //确定发射频率
        Attack.times_between_Bullets = 1000 / 2 / numbers;
        Attack_FUN.Interval_Attack();
    }
    //由于number=0时 1000/0是个无限大的值，无法使用
    else
    {
        Driver.will_Bullet_numbers = 0;
    }

}
//发射模式三:永不超热量发射（冷却值50/s,而1颗弹消耗10，那么只要5颗/1s即可,即200ms）
void Forever_NoOverHeat_Processing(void)
{
    //确定发射频率
    Attack.times_between_Bullets = 200 / 2;
    Attack_FUN.Interval_Attack();
}
//发射模式四：不发射
void Do_not_Attack(void)
{
    //发射间隔时间 10000
    Attack.times_between_Bullets = 10000;
    //需要发射弹数为0
    Driver.will_Bullet_numbers = 0;
}
//攻击函数的初始化
void Attack_Init(void)
{
    //初始化时先选择一个枪管
    Attack.barrel_Select = Barrel1;
    //初始化发射
    Shoot_FUN.Shoot_Init();
    //初始化拨盘
    Driver_FUN.Driver_Init();
}
//攻击函数的执行
void Attack_Processing(void)
{
    //先开启摩擦轮
    Shoot_FUN.Shoot_Control();
    if (Shoot.ramp_Speed.Current_Value == Shoot.ramp_Speed.Target_Value)
    {
        //拨弹的选择
        if (Robots_Control.Attack_e == ak_Disable || Robots_Control.Attack_e == cease_Fire)
        {
            Attack_FUN.Do_not_Attack();
        }
        else if (Robots_Control.Attack_e == R_ak_Sag_Nooverheat || Robots_Control.Attack_e == R_ak_Onslaught_Nooverheat || Robots_Control.Attack_e == A_ak_Nooverheat)
        {
            Attack_FUN.Forever_NoOverHeat_Processing();
        }
        else if (Robots_Control.Attack_e == A_ak_Frequency)
        {
            Attack_FUN.Fluctuant_Strafe_Processing();
        }
        else if (Robots_Control.Attack_e == A_ak_Snipe)
        {
            Attack_FUN.FixedPoint_Snipe_Processing();
        }
    }
		//执行拨弹之前要先确定一下热量是否超了
		if(Robots_Control.Heat_Status >= Greater_than_Danger)
		{
			//发射间隔时间 10000
			Attack.times_between_Bullets = 10000;
			//需要发射弹数为0
			Driver.will_Bullet_numbers = 0;
		}
    //拨盘的执行
    Driver_FUN.Driver_Control(&Driver);
}
