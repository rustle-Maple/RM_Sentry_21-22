#include "Chassis_Control.h"
#include "BREAK.h"

//编码器轮子的参数
const float Encoder_Radius = 50.0f;
const float Encoder_Perimeter = (float)Encoder_Radius * 2.0f * pai;
const float Encoder_Lines = 3500.0f;
const float Encoder_UnitLength = (float)Encoder_Perimeter / Encoder_Lines;
//三种应激状态的速度
const float Limit_Velocity = 3000.0f;			//3500
const float Cruise_Velocity = 6000.0f;		//6000
const uint16_t R_Increment_times = 300;
const uint16_t R_Lower_Limit = 6000;			//5500
const uint16_t R_Upper_Limit = 9000;			//8000
// const float Random_Velocity = 6500.0f;
const float Frenzy_Velocity = 6500.0f;
//随机运动的采样时间
const uint16_t Ran_Dir_Sam = 350;
const uint16_t Ran_Dir_Pro = 70;

int32_t B0_Location = 49834;			//这个值不要取定值，取确定云台pitch轴时的total
int32_t B0_Lenght = 100000;

/*
const uint16_t Random_Intervaltimes = 500; //500ms间隔采样
const uint8_t Random_Proportion = 50;      //随机概率占比
*/
//追击
const float Best_Distance = 4000.0f;
const float EM_Refer_Angle = 4035.0f;

Chassis_t Chassis;
//底盘函数挂钩
Chassis_FUN_t Chassis_FUN = Chassis_HOOK_FUN;

// Purse_t IMU_Purse; //陀螺仪的追击变量
//电机的追击变量
Purse_t EM_Purse;
//初始化
void EM_Purse_Init(void)
{
    EM_Purse.real_angle = &Cloud.Yaw->totalAngle;
    EM_Purse.refer_angle = EM_Refer_Angle;
    EM_Purse.delta_angle = 0.0f;
    EM_Purse.unit_conversion = (float)1 / Coded_Contact_Radian;
    EM_Purse.angle = 0.0f;
    EM_Purse.delta_distance = 0.0f;
}
//更新底盘传感器的数据
void Updata_Chassis_Sensor(void)
{
    //更新光电
    Get_PSwitch_FLAG();
    //更新编码器值
    Get_Encoder_Value(&Chassis_Encoder, &htim5);
    //更新限位开关
    Get_LSwitch_FLAG();
    //更新云台防撞柱标志位
    Prevent_Cloud_Breakdown();
		//更新底盘B0位置
		exclude_B0_disturb();
}
uint8_t Purse_Flag = 0;
//行程限制函数
void Route_limit_Processing(void)
{
    //无需限制的话（没有编码器）
//    Chassis.Route_limit.flag = 1;

    //判断轨道行程限制是否已经完成，若未完成才进行轨道行程限制
    if (Chassis.Route_limit.flag == 0)
    {
        Chassis.Route_limit.time++;
        //先判断一下是否是以巡航的速度去做行程限制
        if (fabs(Chassis.Velocity.temp_Speed) != Limit_Velocity)
        {
            //让临时速度=巡航速度
            Chassis.Velocity.temp_Speed = Limit_Velocity;
        }
        else //若处于巡航速度了那么就去执行限制行程
        {

            if (Chassis.Route_limit.left_flag == 0)
            {
                //判断左Sensor是否离线
                if (Sensor_L_Framerate.Offline_Flag == 0)
                {
                    if (Sensor_L.RawData.DIST < 40)
                    {
                        if (0)//Sensor_L.RawData.APM > 1000)
                        {
                            Chassis.Route_limit.left = Chassis.encoder->totalLine;
                        }
                        else
                        {
                            //大于200ms，保证已经启动
                            if (Chassis.Route_limit.time > 2000)
                            {
                                if (abs(Chassis.encoder->last_totalLine - Chassis.encoder->totalLine) < 100)
                                {
                                    Chassis.Route_limit.left = Chassis.encoder->totalLine;
                                }
                                if (Chassis.Route_limit.time >= 500 && Chassis.Route_limit.time % 500 == 0)
                                {
                                    Chassis.encoder->last_totalLine = Chassis.encoder->totalLine;
                                }
                            }
                        }
                    }
                }
                else
                {
                    //大于200ms，保证已经启动
                    if (Chassis.Route_limit.time > 2000)
                    {
                        if (abs(Chassis.encoder->last_totalLine - Chassis.encoder->totalLine) < 100)
                        {
                            Chassis.Route_limit.left = Chassis.encoder->totalLine;
                        }
                        if (Chassis.Route_limit.time >= 500 && Chassis.Route_limit.time % 500 == 0)
                        {
                            Chassis.encoder->last_totalLine = Chassis.encoder->totalLine;
                        }
                    }
                }
                //若右限位值不为0 说明右限位成功
                if (Chassis.Route_limit.left != 999999)
                {
                    Chassis.Route_limit.left_flag = 1;
                    Chassis.Route_limit.time = 0;
                }
            }

            if (Chassis.Route_limit.right_flag == 0)
            {
                //若右限位成功，执行左限位
                if (Chassis.Route_limit.left_flag == 1)
                {
                    //目标速度取反
                    Chassis.Velocity.temp_Speed = -Limit_Velocity;

                    //判断右Sensor是否离线
                    if (Sensor_R_Framerate.Offline_Flag == 0)
                    {
                        if (Sensor_R.RawData.DIST < 40)
                        {
                            //判断传感器的置信度
                            if (0) //Sensor_R.RawData.APM > 1000)
                            {
                                Chassis.Route_limit.right = Chassis.encoder->totalLine;
                            }
                            else //若置信度太低
                            {
                                if (Chassis.Route_limit.time > 2000)
                                {
                                    //若2000ms内的线数差<500 说明已堵转
                                    if (abs(Chassis.encoder->last_totalLine - Chassis.encoder->totalLine) < 100)
                                    {
                                        //把编码器的值赋给右限位
                                        Chassis.Route_limit.right = Chassis.encoder->totalLine;
                                    }
                                    //经过2000ms获取一次线数
                                    if (Chassis.Route_limit.time >= 500 && Chassis.Route_limit.time % 500 == 0)
                                    {
                                        Chassis.encoder->last_totalLine = Chassis.encoder->totalLine;
                                    }
                                }
                            }
                        }
                    }
                    else // Sensor离线了
                    {
                        if (Chassis.Route_limit.time > 2000)
                        {
                            if (abs(Chassis.encoder->last_totalLine - Chassis.encoder->totalLine) < 100)
                            {
                                Chassis.Route_limit.right = Chassis.encoder->totalLine;
                            }
                            if (Chassis.Route_limit.time >= 500 && Chassis.Route_limit.time % 500 == 0)
                            {
                                Chassis.encoder->last_totalLine = Chassis.encoder->totalLine;
                            }
                        }
                    }
                    if (Chassis.Route_limit.right != -999999)
                    {
                        Chassis.Route_limit.right_flag = 1;
                    }
                }
            }

            //若左右限位都成功
            if (Chassis.Route_limit.right_flag == 1 && Chassis.Route_limit.left_flag == 1)
            {
                Chassis.Route_limit.flag = 1;       //整体限位成功
                                                    //底盘行程限制清0，为了下次进行底盘限制可以顺利
                Chassis.Route_limit.time = 0;       //时间
                Chassis.Route_limit.right_flag = 0; //右限制
                Chassis.Route_limit.left_flag = 0;
            }
        }
    }
}
//前哨站幸存模式
void Outpost_survive_Processing(void)
{
    //底盘的当前距离(当前位置-参考位置) = (当前线数-参考线数)*单位线数长度
    Chassis.Location.real = (float)Chassis.encoder->totalLine; //- Chassis.Location.reference) * Encoder_UnitLength;
    //将轨道的最右侧作为目标位置
    Chassis.Location.target = (float)Chassis.Route_limit.right + 13500;// - Chassis.Location.reference) * Encoder_UnitLength;
}
//巡航模式
void Cruise_Processing(void)
{
		static uint16_t cruise_times;
		cruise_times ++;
		if(cruise_times < 2000)
		{
			//判断此时临时速度是否为巡航的速度,若不为，则置为
			if (fabs(Chassis.Velocity.temp_Speed) != Cruise_Velocity)
			{
					Chassis.Velocity.temp_Speed = Cruise_Velocity;
			}
		}
		else if(cruise_times > 2000)
		{
			Random_Processing();
		}
		
		if(cruise_times > 4500)
		{
			cruise_times = 0;
		}
		
}

//指数函数模型参数计算
void Index_Parameter_Calculate(void)
{
    /*
        公式解析：
        指数函数模型：a^x + b = y
        再用换底公式：lnx / lna = y - b
        即可得到 a = e^(lnx/(y-b))
    */
    //加速底数的计算
    Chassis.Index_VarSpe.Accelerate_base = exp((double)(log((double)(Chassis.Index_VarSpe.Upper_Limit - (Chassis.Index_VarSpe.Lower_Limit - 1))) / Chassis.Index_VarSpe.Increment_times));
    //减速底数的计算
    Chassis.Index_VarSpe.Decelerate_base = exp((double)(log((double)(Chassis.Index_VarSpe.Upper_Limit - (Chassis.Index_VarSpe.Lower_Limit - 1))) / -Chassis.Index_VarSpe.Increment_times));
}

//指数函数模型-变速运动
void Index_VariableSpeed(void)
{
    Chassis.Index_VarSpe.times++; //运行时间的累加

    //对不同自变量区间执行不同函数
    //分段函数
    if (Chassis.Index_VarSpe.times >= 0 && Chassis.Index_VarSpe.times <= R_Increment_times) //递增阶段
    {
        Chassis.Index_VarSpe.abs_Vel = pow(Chassis.Index_VarSpe.Accelerate_base, Chassis.Index_VarSpe.times) + (Chassis.Index_VarSpe.Lower_Limit - 1);
    }
    else if (Chassis.Index_VarSpe.times >= R_Increment_times && Chassis.Index_VarSpe.times <= R_Increment_times * 2)
    {
        Chassis.Index_VarSpe.abs_Vel = pow(Chassis.Index_VarSpe.Decelerate_base, (Chassis.Index_VarSpe.times - Chassis.Index_VarSpe.Increment_times)) + (Chassis.Index_VarSpe.Lower_Limit - 1); //对函数进行平移左加右减
    }

    //对自变量进行限制
    if (Chassis.Index_VarSpe.times > (2 * Chassis.Index_VarSpe.Increment_times))
    {
        Chassis.Index_VarSpe.times = 0;
    }
}

float res_d;
uint32_t aaaaaa;
//随机模式
void Random_Processing(void)
{
    //主动攻击的状态标志位
    static uint8_t fire_flag = 0;
    //百分百速度运行
    Chassis.Random.percent = 1.0f;

    //更新方向随机数
    Chassis.Random.Dir_number = Get_RandomNumbers_Range(0, 100);
		//更新时间随机数
		Chassis.Random.Time_number = Get_RandomNumbers_Range(400,650);
//		if(Robots_Control.Chassis_e == A_cs_Cruise)
//		{
//			Chassis.Random.Time_number = Get_RandomNumbers_Range(450,600);
//		}
		//3000ms进行一次模式切换的检测
		if((Chassis.Random.Time % 3000 == 0) && Chassis.Random.Time > 0)
		{
			//运行模式时间的选择
			Chassis.Random.Mode_number = Get_RandomNumbers_Range(0,10);	
			Chassis.Random.Time = 0;	//清0重新开始
		}
		
		//累计随机运动方向时间
    Chassis.Random.Dir_times++; 
		//运行随机数的总时间
		Chassis.Random.Time++;

//		//运行模式时间的选择占比20（共50）
//		if(Chassis.Random.Mode_number < 3)
//		{	
//			//定周期不定变向
//			if (Chassis.Random.Dir_times == Ran_Dir_Sam)
//			{
//        //采取变向：随机概率为50%
//        if (Chassis.Random.Dir_number < Ran_Dir_Pro)
//        {
//            Chassis.Random.Dir = -Chassis.Random.Dir; //反向
//        }
//        Chassis.Random.Dir_times = 0; //方向周期清0
//			}
//		}
//		//运行模式时间选择占比30（共50）
//		else if(Chassis.Random.Mode_number >= 3)
//		{
			res_d = abs(Chassis.Random.Dir_times - Chassis.Random.Time_number);
			//不定周期定变向
			if(res_d < 2.0f && Chassis.Random.Break_flag == 0)				//未处于刹车过程中，才需要再次改变方向
			{
				Chassis.Random.Break_flag = 1; 
				aaaaaa++;
				Chassis.Random.Dir = -Chassis.Random.Dir; //反向
				Chassis.Random.Dir_times = 0; 						//方向周期清0
			}
//		}
			if(Chassis.Random.Dir_times > 610)				//限幅：防止无法res_d条件不成立无法进入变向清0
			{
				Chassis.Random.Dir_times = 0;
			}
		
		//无受到攻击时巡航减低速度
		if(Robots_Control.Chassis_e == A_cs_Cruise)
		{
//		Chassis.Random.percent = 0.75f;
			Chassis.Random.percent = 1.0f;
		}
		
    //排除随机运动的方向与撞柱方向的冲突
		if(Chassis.CDisable.Left_flag == 1)
		{
			Chassis.Random.Dir = -1;
		}
		else if(Chassis.CDisable.Right_flag == 1)
		{
			Chassis.Random.Dir = 1;
		}
		//撞柱和刹车的冲突
		if(Chassis.CDisable.Left_flag == 1 || Chassis.CDisable.Right_flag == 1)
		{
			//变向需要刹车的清0
			Chassis.Random.Break_flag = 0;
			//刹车状态清0
			Break_static = 0;
			//刹车的目标值置中
			M2006_targe_angle = break_basic.BREAK_MID;
		}
		
/*	只有激光测距
		if(Sensor_R.RawData.DIST < 35 )//|| Chassis.encoder->totalLine < (Chassis.Route_limit.right + 7800))
		{
        Chassis.Random.Dir = 1;
    }
    else if (Sensor_L.RawData.DIST < 35 )//|| Chassis.encoder->totalLine > (Chassis.Route_limit.left - 7800))
    {
        Chassis.Random.Dir = -1;
    }
*/
		
/*
    //若处于主动进攻状态则
    //判断我方id，决定要血量的比较
    //		if(ext_game_robot_state.data.robot_id == 7)		//说明我方是红方
    //		{
    //			if(ext_game_robot_HP.data.red_7_robot_HP > ext_game_robot_HP.data.blue_7_robot_HP)
    //			{
    //				fire_flag = 1;
    //			}
    //			else
    //			{
    //				fire_flag = 0;
    //			}
    //		}
    //		else if(ext_game_robot_state.data.robot_id == 107)
    //		{
    //			if(ext_game_robot_HP.data.red_7_robot_HP < ext_game_robot_HP.data.blue_7_robot_HP)
    //			{
    //				fire_flag = 1;
    //			}
    //			else
    //			{
    //				fire_flag = 0;
    //			}
    //		}
    //		if(fire_flag == 1)
    //		{
    //			Chassis.Random.Dir = 1;			//不变向
    //			Chassis.Random.percent = 0.8f;			//以百分之80%
    //		}
*/

    //执行变速
    Index_VariableSpeed();
    //方向 + 速度
    Chassis.Velocity.temp_Speed = Chassis.Random.Dir * Chassis.Index_VarSpe.abs_Vel * Chassis.Random.percent;

    /*【21赛季】
    //先置速度为随机运动的速度
    if (fabs(Chassis.Velocity.temp_Speed) != Random_Velocity)
    {
        Chassis.Velocity.temp_Speed = Random_Velocity;
    }
    //更新随机数
    Chassis.Random.number = Get_RandomNumbers_Range(0, 100);
    //随机运动的间隔时间为500ms
    Chassis.Random.sampling++;
    if (Chassis.Random.sampling == Random_Intervaltimes)
    {
        //反向占比
        if (Chassis.Random.number >= Random_Proportion)
        {
            //反向
            Chassis.Velocity.temp_Speed = -Chassis.Velocity.temp_Speed;
        }
        Chassis.Random.sampling = 0;
    }
    */
}

//由于机械方面的适配问题，当底盘撞柱时，枪管不可以对着柱子，不然云台会撞到柱子上去
//更新云台撞柱子标志位
void Prevent_Cloud_Breakdown(void)
{

    static int CAN2_SEND_TASK_times = 0;

    //在轨道行程限制确定后
    if (Chassis.Route_limit.flag == 1)
    {
        //默认在轨道中间
        in_END = 0;
        in_MID = 1;

        //判断右Sensor是否离线
        if (Sensor_R_Framerate.Offline_Flag == 0)
        {
            //若Sensor的执置信度高，则用Sensor
            if (Sensor_R.RawData.DIST < 80)
            {
                if (Sensor_R.RawData.APM > 1000)
                {
                    in_END = 1;
                    in_MID = 0;
                }
                else
                {
                    if (Chassis_Encoder.totalLine < (Chassis.Route_limit.right + 10400))
                    {
                        in_END = 1;
                        in_MID = 0;
                    }
                }
            }
        }
        else //若Sensor离线也用编码器数据
        {
            if (Chassis_Encoder.totalLine < (Chassis.Route_limit.right + 10400))
            {
                in_END = 1;
                in_MID = 0;
            }
        }

        //判断左Sensor是否离线
        if (Sensor_L_Framerate.Offline_Flag == 0)
        {
            //若Sensor的执置信度高，则用Sensor
            if (Sensor_L.RawData.DIST < 80)
            {
                if (Sensor_L.RawData.APM > 1000)
                {
                    in_END = 1;
                    in_MID = 0;
                }
                else
                {
                    if (Chassis_Encoder.totalLine > (Chassis.Route_limit.left - 10400))
                    {
                        in_END = 1;
                        in_MID = 0;
                    }
                }
            }
        }
        else //若Sensor离线也用编码器数据
        {
            if (Chassis_Encoder.totalLine > (Chassis.Route_limit.left - 10400))
            {
                in_END = 1;
                in_MID = 0;
            }
        }
    }

    //按特定频率置发送标志位
    CAN2_SEND_TASK_times++;
    if (in_MID == 1)
    {
        if (CAN2_SEND_TASK_times % 100 == 0) //发送频率为1秒10次
        {
            send_to_C_IN_END = 1; //发送频率为1秒10次
        }
    }
    if (in_END == 1)
    {
        if (CAN2_SEND_TASK_times % 10 == 0) //发送频率为1秒100次
        {
            send_to_C_IN_END = 1; //发送频率为1秒100次
        }
    }
}

//排除B0标的干扰
//B0标位于轨道从右端开始增量为3912mm
uint8_t B0_flag = 0;
void exclude_B0_disturb(void)
{
	
	//对应挡位下的获取当时totalline的值
	if(DR16.Switch_Left == DR16_SWITCH_MID && DR16.Switch_Right == DR16_SWITCH_DOWN && DR16.ch4_DW < -600)
	{
		B0_Location = Chassis.encoder->totalLine;
	}
	
//	static int SEND_B0_FLAG_TIME = 0;
	if(Chassis.Route_limit.flag == 1)
	{
		if((Chassis.encoder->totalLine >  B0_Location - B0_Lenght / 2 ) && Chassis.encoder->totalLine <  B0_Location + B0_Lenght / 2)
		{
			B0_flag = 1;
		}
		else
		{
			B0_flag = 0;
		}
	}

//	SEND_B0_FLAG_TIME ++;
//	if(B0_flag == 1)
//	{
//		if(SEND_B0_FLAG_TIME % 100 == 0)
//		{
//			send_to_C_B0_FLAG = 1;
//		}
//	}
}


//暴走模式
void Frenzy_Processing(void)
{
    /*
    //刚进入狂暴模式
    if (Chassis.Frenzy.just_flag == 0)
    {

        //将其置为狂暴速度
        if (fabs(Chassis.Velocity.temp_Speed) != Frenzy_Velocity)
        {
            Chassis.Velocity.temp_Speed = Frenzy_Velocity;
        }
        Chassis.Frenzy.just_flag = 1;
    }
    //设置暴走模式下的速度斜坡率值
    Chassis.Frenzy.ramp_speed.Rate = 10.0f;           //当未达到目标速度就已经在行程范围内了，那就改大rate
    Chassis.Frenzy.ramp_speed.Absolute_Max = 6000.0f; //极值限幅
                                                      //斜坡当前值即是临时目标速度
    Chassis.Frenzy.ramp_speed.Current_Value = Chassis.Velocity.temp_Speed;
    //判断是否超过左端的最大行程
    if (Chassis.encoder->totalLine < Chassis.Route_limit.left)
    {
        //若超过行程的最左端，则就以斜坡慢慢将速度往回拉
        Chassis.Frenzy.ramp_speed.Target_Value = Frenzy_Velocity;
    }
    //判断是否超过行程的最右端
    else if (Chassis.encoder->totalLine > Chassis.Route_limit.right)
    {
        //则将斜坡的目标值取为负的Frenzy_Velocity
        Chassis.Frenzy.ramp_speed.Target_Value = -Frenzy_Velocity;
    }
    //进行斜坡计算，得到的结果即是临时的目标速度，斜坡的当前速度
    Chassis.Velocity.temp_Speed = Ramp_Function(&Chassis.Frenzy.ramp_speed);

    //撞到了限位开光
    if (Limit_switch.LSwitch_R_1 == 0 || Limit_switch.LSwitch_R_2 == 0)
    {
        Chassis.Velocity.temp_Speed = -Frenzy_Velocity;
    }
    else if (Limit_switch.LSwitch_L_1 == 0 || Limit_switch.LSwitch_L_2 == 0)
    {
        Chassis.Velocity.temp_Speed = Frenzy_Velocity;
    }
        */
}
/* 得Yaw轴云台可以“归中”才可用此算法，但由于开机启动的yaw轴角度是由我们自己决定的，所以即使无法“归中”也不是不行 */
//追击模式
//最佳攻击距离的确定
void Optimally_Attack_Distance(float real_Angle, float real_depth, float *b)
{
    /*
    //得到角度差值
    Chassis.Purse->delta_angle = abs(real_Angle - Chassis.Purse->refer_angle);
    //转换为弧度
    Chassis.Purse->delta_angle *= Chassis.Purse->unit_conversion;
    //将其转为三角形内角和可用于计算的角度
    Chassis.Purse->angle = Chassis.Purse->delta_angle - pai * (int)(Chassis.Purse->delta_angle / pai);

    float a = Best_Distance, c = real_depth;
    //计算出a边对应的角度余弦值
    float COS_A = cos(Chassis.Purse->angle);
    //根据余弦定理和判别公式 算出delta
    float delta = pow(a, 2) - pow(c, 2) + pow(c, 2) * pow(COS_A, 2);

    if (delta <= 0.0f) //本身无法构成三角形 delta<0  只有1个角 直角三角形 delta=0
    {
        *b = COS_A * c;
    }
    else
    {
        *b = COS_A * c - sqrt(delta); //存在两个解的，但运动只需要一个方向一个解即可
    }
        */
}
//但是以上此算法本人觉得算法存在个问题：那就是角度A，应该是当前机械角度所对应的角，还是应该取其补角，来进行计算
//这个角度受两个因素所影响，一：所选取的Yaw参考系的方向、二：最佳攻击距离a 是> 当前距离c 还是小于c
//由于有两个不确定的变量：故有4种情况
//本人觉得可以 以选取相同的坐标系，得到敌方和我方的机器人在此坐标系下的坐标值，再根据两点之间距离为最佳攻击距离的 思想去写
//待定吧
//移动时只需要改变方向，即只需改变底盘移动增量的方向
//而限位也因为底盘增量方向的改变而
//而PID的符号无需改变
void Pursue_Processing(void)
{
    /*
    //将处于狙击态的标志位置1
    Robots_Control.Snipe_Status.ing_Flag = 1;
    //先得到当前的位置距离
    Chassis.Location.real = (float)(Chassis.encoder->totalLine - Chassis.Location.reference) * Encoder_UnitLength;
    //计算出应该追击的距离
    Chassis_FUN.Optimally_Attack_Distance(*(Chassis.Purse->real_angle), VisionData_Hand.Vision_FilterData.Depth, &Chassis.Purse->delta_distance);

    //计算一下是在轨道的哪一个方向
    if ((int)(Chassis.Purse->delta_angle / pai) % 2 == 1) //将来看看这样对不对，不对看你得两种情况相反 == 0
    {
        //目标距离 = 当前距离 + 追击距离
        Chassis.Location.target = Chassis.Location.real + Chassis.Purse->delta_distance;
        //判断一下是否到达轨道两端
        if (Chassis.encoder->totalLine <= Chassis.Route_limit.left && Chassis.Purse->delta_distance <= 0) //将来这个条件也得根据实际情况去测
        {
            Chassis.Location.target = (float)(Chassis.Route_limit.left - Chassis.Location.reference) * Encoder_UnitLength;
        }
        else if (Chassis.encoder->totalLine >= Chassis.Route_limit.right && Chassis.Purse->delta_distance > 0)
        {
            Chassis.Location.target = (float)(Chassis.Route_limit.right - Chassis.Location.reference) * Encoder_UnitLength;
        }
    }
    else if ((int)(Chassis.Purse->delta_angle / pai) % 2 == 0)
    {
        Chassis.Location.target = Chassis.Location.real - Chassis.Purse->delta_distance;
        if (Chassis.encoder->totalLine <= Chassis.Route_limit.left && Chassis.Purse->delta_distance >= 0) //将来这个条件也得根据实际情况去测
        {
            Chassis.Location.target = (float)(Chassis.Route_limit.left - Chassis.Location.reference) * Encoder_UnitLength;
        }
        else if (Chassis.encoder->totalLine >= Chassis.Route_limit.right && Chassis.Purse->delta_distance < 0)
        {
            Chassis.Location.target = (float)(Chassis.Route_limit.right - Chassis.Location.reference) * Encoder_UnitLength;
        }
    }

        if(abs(VisionData_Hand.Vision_FilterData.Depth - Best_Distance) < 200)
        {
            Chassis.Location.target = Chassis.Location.real;
        }
        */
}
//胶轮运动模型的解算
void MecanumCalculate(float Vx, float V0mega, float *speed)
{

    int16_t resultantSpeed[1];
    uint8_t i; //循环次数
    int16_t Max_resultantSpeed = 0;
    float rate = 1.0f;

    Absolute_Value_Limit(&Vx, Chassis_MaxSpeed_X);
    Absolute_Value_Limit(&V0mega, Chassis_MaxSpeed_Z);

    resultantSpeed[0] = Vx + V0mega;

    // 3.单轮速度大小的分配：
    //为何要进行速度分配？
    //首先因为电机的速度是存在上限的，
    //若其中1个轮子计算出来的理论上的速度超过了电机的上限速度，
    //那么就会导致此轮子的速度达不到计算出来的理论值
    //那么它与其他轮子速度合成后实际运动的方向，就不是理论上计算得到的方向了

    // a.寻找速度大小最大值
    for (i = 0; i < 1; i++)
    {
        if (abs(resultantSpeed[i]) > Max_resultantSpeed)
        {
            Max_resultantSpeed = abs(resultantSpeed[i]);
        }
    }
    // b.速度分配比率
    if (Max_resultantSpeed > Chassis_MaxSpeed_Wheel)
    {
        rate = (float)Chassis_MaxSpeed_Wheel / Max_resultantSpeed;
    }
    // c.按理论上的占比分配实际速度
    speed[0] = resultantSpeed[0] * rate;
}

void Chassis_Init(void)
{
    //确定电机
    Chassis.EM = &M3508s[3];
    //确定编码器
    Chassis.encoder = &Chassis_Encoder;
    //传感器数据
    Chassis_FUN.Updata_Chassis_Sensor();
    //追击初始化
    Chassis_FUN.EM_Purse_Init();
    //滤波系数初始化
    Chassis.Velocity.LpfAttFactor = 1.0f;
    //确定速度增量式pid
    Chassis.Velocity.I_PID = &Chassis.EM->I_PID;
    //位置式
    Chassis.Velocity.P_PID = &Chassis.EM->P_PID;
    //确定位置的位置式pid
    Chassis.Location.P_PID = &Chassis.encoder->P_PID;
    //初始化追击结构体
    Chassis.Purse = &EM_Purse;
    //功率限制
    //    Chassis_PowerLimit.PowerRatio_Denominator = 200.0f;
    // PID的初始化
    //速度的增量式pid的初始化
    I_PID_FUN.I_PID_Parameter_Init(Chassis.Velocity.I_PID, 8.5f, 0.25f, 0.0f, 20000.0f, 0.0f, 0.0f, 0.85, 10000, -10000, 16000, -16000);
    //位置的位置式pid的初始化
    //坐标轴都是同向的呀
    //外环Kp,不可以给太小，外环输出会相对于内环测量小很多
    P_PID_FUN.P_PID_Parameter_Init(Chassis.Location.P_PID, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85, 0, -0, 4500, -4500); //实际上看你想要的目标速度是多少
    P_PID_FUN.P_PID_Parameter_Init(Chassis.Velocity.P_PID, 12.5f, 0.0f, 0.0f, 400.0f, 0.0f, 0.0f, 0.85, 4000, -4000, 16000, -16000);
		
		//刹车电机PID初始化
    P_PID_FUN.P_PID_Parameter_Init(&BREAK_ANGLE_pid, 0.3f, 0.0f, 0.0f, 5000.0f, 0.0f, 0.0f, 0.85, 10, -10, 3500, -3500); //实际上看你想要的目标速度是多少
    P_PID_FUN.P_PID_Parameter_Init(&BREAK_SPEED_pid, 3.0f, 0.01f, 0.0f, 3000.0f, 0.0f, 0.0f, 0.85, 500, -500, 4000, -4000);
				

    //判断主控是否连上服务器 - 则对应相应的底盘功率
    if (ext_power_heat_data.data.chassis_power_buffer == 200.0f)
    {
        //服务器已连接/以开始比赛
        Judge_Monitor.Connect_OffFlag = 0;
    }
    else
    {
        //服务器未连接/未开始比赛
        Judge_Monitor.Connect_OffFlag = 1;
    }

    //随机反向
    Chassis.Random.Dir = 1;
    //随机运动参数初始化
    Chassis.Index_VarSpe.Increment_times = R_Increment_times;
    Chassis.Index_VarSpe.Lower_Limit = R_Lower_Limit;
    Chassis.Index_VarSpe.Upper_Limit = R_Upper_Limit;
    //计算指数变速的底数
    Index_Parameter_Calculate();
		
		Chassis.Route_limit.left = 999999;
		Chassis.Route_limit.right = -999999;

    //		Chassis.Route_limit.flag = 1;
}
float debug_location = 0.0f;
void Chassis_Control(void)
{
		//刹车初始化
		break_init();
	
    if (Robots_Control.Chassis_e == cs_Disable)
    {
        Chassis.Velocity.targetXRaw = 0; // 0;
        Chassis.Velocity.targetZRaw = 0;
        Chassis.Velocity.temp_Speed = 0;

        Chassis.EM->OutputCurrent = 0;

        I_PID_FUN.I_PID_Parameter_Clear(Chassis.Velocity.I_PID);
        P_PID_FUN.P_PID_Parameter_Clear(Chassis.Location.P_PID);
        P_PID_FUN.P_PID_Parameter_Clear(Chassis.Velocity.P_PID);
			
				//失能
				send_to_break = 0;
				//清空变向刹车状态
				Chassis.Random.Break_flag = 0;
				//刹车状态清0
				Break_static = 0;
				//刹车目标值置为中值
				M2006_targe_angle = break_basic.BREAK_MID;	

        return;
    }
#if Pures_Debug == 1
    if (Purse_Flag == 1)
    {
        Pursue_Processing();
    }
#endif
    if (Robots_Control.Chassis_e == R_cs_Common)
    {
        Chassis.Velocity.targetZRaw = 0;
        Chassis.Velocity.targetXRaw = Chassis.Velocity.temp_Speed = DR16.ch2 * 10.0f;
    }
    else if (Robots_Control.Chassis_e == A_cs_Pathway)
    {
        Chassis_FUN.Route_limit_Processing();
    }
    else if (Robots_Control.Chassis_e == A_cs_Outpost)
    {
        Chassis_FUN.Outpost_survive_Processing();
    }
    else if (Robots_Control.Chassis_e == A_cs_Cruise)
    {
//        Chassis_FUN.Cruise_Processing();
				Chassis_FUN.Random_Processing();
    }
    else if (Robots_Control.Chassis_e == A_cs_Random)
    {
        Chassis_FUN.Random_Processing();
    }
    else if (Robots_Control.Chassis_e == A_cs_Frenzy)
    {
        Chassis_FUN.Frenzy_Processing();
    }
    else if (Robots_Control.Chassis_e == A_cs_Pursue)
    {
        Chassis_FUN.Pursue_Processing();
    }

    if (Robots_Control.Chassis_e == A_cs_Cruise || Robots_Control.Chassis_e == A_cs_Random)
    {
//        if (Sensor_R.RawData.APM > 1000) // Sensor
//        {
//            if (Chassis.Velocity.temp_Speed < 0 && Sensor_R.RawData.DIST < 50)
//            {
//                Chassis.Velocity.temp_Speed = -Chassis.Velocity.temp_Speed;
//            }
//        }
//        else // Sensor不准
//        {
//            if (Chassis.encoder->totalLine < (Chassis.Route_limit.right + 7800) && Chassis.Velocity.temp_Speed < 0)
//            {
//                Chassis.Velocity.temp_Speed = -Chassis.Velocity.temp_Speed;
//            }
//        }

//        if (Sensor_L.RawData.APM > 1000)
//        {
//            if (Chassis.Velocity.temp_Speed > 0 && Sensor_L.RawData.DIST < 50)
//            {
//                Chassis.Velocity.temp_Speed = -Chassis.Velocity.temp_Speed;
//            }
//        }
//        else
//        {
//            if (Chassis.encoder->totalLine > (Chassis.Route_limit.left - 7800) && Chassis.Velocity.temp_Speed > 0)
//            {
//                Chassis.Velocity.temp_Speed = -Chassis.Velocity.temp_Speed;
//            }
//        }
			}
        /*
        【21届哨兵】
    //判断是否识别到光电
        if (Chassis.Velocity.temp_Speed < 0 && PSwitch_FLAG.PSwitch_L == 0)
        {
            Chassis.Velocity.temp_Speed = -Chassis.Velocity.temp_Speed;
        }
        else if (Chassis.Velocity.temp_Speed > 0 && PSwitch_FLAG.PSwitch_R == 0)
        {
            Chassis.Velocity.temp_Speed = -Chassis.Velocity.temp_Speed;
        }
        */
//    }

    //确定目标速度
    Chassis.Velocity.targetXRaw = Chassis.Velocity.temp_Speed;

    //若需要不同的
    //滤波 （要在麦轮解算之前，可以防止麦轮解算后滤波导致运动模型错乱）
    Filter_IIRLPF(&Chassis.Velocity.targetXRaw, &Chassis.Velocity.targetXLPF, Chassis.Velocity.LpfAttFactor);
    Filter_IIRLPF(&Chassis.Velocity.targetZRaw, &Chassis.Velocity.targetZLPF, Chassis.Velocity.LpfAttFactor);

    //进行胶轮解算
    MecanumCalculate(Chassis.Velocity.targetXLPF, Chassis.Velocity.targetZLPF, Chassis.Velocity.Calcu_Speed);

    Chassis.EM->targetSpeed = Chassis.Velocity.Calcu_Speed[0];

    //经行pid计算
    //处于前哨战模式 和 追击模式时 是双环控制
    if (Robots_Control.Chassis_e == A_cs_Outpost || Robots_Control.Chassis_e == A_cs_Pursue || Purse_Flag == 1)
    {

#if Debug_Location_pid == 1
        Chassis.Location.target = debug_location; //(0~1600)
        Chassis.Location.real = (float)(Chassis.encoder->totalLine - Chassis.Location.reference) * Encoder_UnitLength;
#endif
        //清除单环pid
        I_PID_FUN.I_PID_Parameter_Clear(Chassis.Velocity.I_PID);

        P_PID_FUN.P_PID_Regulation(Chassis.Location.P_PID, Chassis.Location.target, Chassis.Location.real);
        P_PID_FUN.P_PID_Regulation(Chassis.Velocity.P_PID, Chassis.Location.P_PID->result, Chassis.EM->realSpeed);
        Chassis.EM->OutputCurrent = Chassis.Velocity.P_PID->result;
    }
    else //单环控制
    {
        //清除双环pid
        P_PID_FUN.P_PID_Parameter_Clear(Chassis.Location.P_PID);
        P_PID_FUN.P_PID_Parameter_Clear(Chassis.Velocity.P_PID);

        I_PID_FUN.I_PID_Regulation(Chassis.Velocity.I_PID, Chassis.EM->targetSpeed, Chassis.EM->realSpeed);
        Chassis.EM->OutputCurrent = Chassis.Velocity.I_PID->result;
    }
		
		M2006_targe_speed=P_PID_Regulation(&BREAK_ANGLE_pid,M2006_targe_angle,M3508s[BREAK_ID].totalAngle);//M2006_targe_speed应该大于0
		send_to_break=P_PID_Regulation(&BREAK_SPEED_pid,M2006_targe_speed,M3508s[BREAK_ID].realSpeed);
		
		//刹车控制
		break_control();
		
		//zhuangzhu
    if (Robots_Control.Chassis_e == A_cs_Cruise || Robots_Control.Chassis_e == A_cs_Random)
		{
			//debug
			Chassis.encoder->Offline_flag = 1;
			//编码器优先
			//左
			if(Chassis.encoder->Offline_flag == 0)
			{
				if(Chassis.encoder->totalLine > (Chassis.Route_limit.left - 6400) && Chassis.EM->realSpeed > 30)
				{
					Chassis.CDisable.Left_flag = 1;
				}
			}
			else if(Sensor_L.RawData.APM > 1000)
			{
				if(Sensor_L.RawData.DIST < 50 && Chassis.EM->realSpeed > 30)
				{
					Chassis.CDisable.Left_flag = 1;
				}
			}
			//右
			if(Chassis.encoder->Offline_flag == 0)
			{
				if(Chassis.encoder->totalLine < (Chassis.Route_limit.right  + 6400) && Chassis.EM->realSpeed < -30)
				{
					Chassis.CDisable.Right_flag = 1;
				}
			}
			else if(Sensor_R.RawData.APM > 1000)
			{
				if(Sensor_R.RawData.DIST < 50 && Chassis.EM->realSpeed < -30)
				{
					Chassis.CDisable.Right_flag = 1;
				}
			}
/*		优先级激光测距
			if(Sensor_L.RawData.APM > 1000)
			{
				if(Sensor_L.RawData.DIST < 35 && Chassis.EM->realSpeed > 30)
				{
					Chassis.CDisable.Left_flag = 1;
				}
			}
//			else if(Chassis.encoder->totalLine > (Chassis.Route_limit.left - 10400) && Chassis.EM->realSpeed > 0)
//			{
//				Chassis.CDisable.Left_flag = 1;
//			}
			if(Sensor_R.RawData.APM > 1000)
			{
				if(Sensor_R.RawData.DIST < 35 && Chassis.EM->realSpeed < -30)
				{
					Chassis.CDisable.Right_flag = 1;
				}
			}
//			else if(Chassis.encoder->totalLine < (Chassis.Route_limit.right  + 10400) && Chassis.EM->realSpeed < 0)
//			{
//				Chassis.CDisable.Right_flag = 1;
//			}
*/			
			
			if(Chassis.CDisable.Right_flag == 1 || Chassis.CDisable.Left_flag == 1)
			{
				Chassis.EM->OutputCurrent = 0;
				if(Chassis.CDisable.Left_flag == 1)
				{
					Chassis.CDisable.Left_times++;
					if(Chassis.encoder->Offline_flag == 0)
					{
						if((Chassis.encoder->totalLine < (Chassis.Route_limit.left - 7800) && Chassis.EM->realSpeed < -30) || Chassis.CDisable.Left_times > 150)
						{
							Chassis.CDisable.Left_flag = 0;
							Chassis.Velocity.temp_Speed = -fabs(Chassis.Velocity.temp_Speed);
							//轻触一下随机运动的时间累计，不让它频繁的在边缘反复变向
							Chassis.Random.Dir_times = 0;
						}
					}
					else if(Sensor_L.RawData.APM > 100)
					{
						if((Sensor_L.RawData.DIST > 30 && Chassis.EM->realSpeed < -30)|| Chassis.CDisable.Left_times > 150)
						{
							Chassis.CDisable.Left_flag = 0;
							Chassis.Velocity.temp_Speed = -fabs(Chassis.Velocity.temp_Speed);
							//轻触一下随机运动的时间累计，不让它频繁的在边缘反复变向
							Chassis.Random.Dir_times = 0;
						}
					}
					else if(Chassis.CDisable.Left_times > 150)
					{
						Chassis.CDisable.Left_flag = 0;
						Chassis.Velocity.temp_Speed = -fabs(Chassis.Velocity.temp_Speed);
						//轻触一下随机运动的时间累计，不让它频繁的在边缘反复变向
						Chassis.Random.Dir_times = 0;
					}
					/*				优先级激光测距
					if(Sensor_L.RawData.APM > 100)
					{
						if((Sensor_L.RawData.DIST > 20 && Chassis.EM->realSpeed < -30)|| Chassis.CDisable.Left_times > 150)
						{
							Chassis.CDisable.Left_flag = 0;
							Chassis.Velocity.temp_Speed = -fabs(Chassis.Velocity.temp_Speed);
							//轻触一下随机运动的时间累计，不让它频繁的在边缘反复变向
							Chassis.Random.Dir_times = 0;
						}
					}
//					else if(Chassis.encoder->totalLine > (Chassis.Route_limit.left - 7800) || Chassis.CDisable.Left_times > 300)
//					{
//							Chassis.CDisable.Left_flag = 0;
//							Chassis.Velocity.temp_Speed = -fabs(Chassis.Velocity.temp_Speed);
//					}
				}
*/				
				}
				
				if(Chassis.CDisable.Right_flag == 1)
				{
					Chassis.CDisable.Right_times++;
					if(Chassis.encoder->Offline_flag == 0)
					{
						if((Chassis.encoder->totalLine > (Chassis.Route_limit.right + 7800) && Chassis.EM->realSpeed > 30) || Chassis.CDisable.Right_times > 150)
						{
							Chassis.CDisable.Right_flag = 0;
							Chassis.Velocity.temp_Speed = fabs(Chassis.Velocity.temp_Speed);
							//轻触一下随机运动的时间累计，不让它频繁的在边缘反复变向
							Chassis.Random.Dir_times = 0;
						}
					}
					else if(Sensor_R.RawData.APM > 100)
					{
						if((Sensor_R.RawData.DIST > 30 && Chassis.EM->realSpeed > 30)|| Chassis.CDisable.Right_times > 150)
						{
							Chassis.CDisable.Right_flag = 0;
							Chassis.Velocity.temp_Speed = fabs(Chassis.Velocity.temp_Speed);
							//轻触一下随机运动的时间累计，不让它频繁的在边缘反复变向
							Chassis.Random.Dir_times = 0;
						}
					}
					else if(Chassis.CDisable.Right_times > 150)
					{
							Chassis.CDisable.Right_flag = 0;
							Chassis.Velocity.temp_Speed = fabs(Chassis.Velocity.temp_Speed);
							//轻触一下随机运动的时间累计，不让它频繁的在边缘反复变向
							Chassis.Random.Dir_times = 0;
					}
					/*			优先级激光测距
				if(Chassis.CDisable.Right_flag == 1)
				{
					Chassis.CDisable.Right_times++;
					if(Sensor_R.RawData.APM > 100)
					{
						if((Sensor_R.RawData.DIST > 20 && Chassis.EM->realSpeed > 30)|| Chassis.CDisable.Right_times > 150)
						{
							Chassis.CDisable.Right_flag = 0;
							Chassis.Velocity.temp_Speed = fabs(Chassis.Velocity.temp_Speed);
							//轻触一下随机运动的时间累计，不让它频繁的在边缘反复变向
							Chassis.Random.Dir_times = 0;
						}
					}
//					else if(Chassis.encoder->totalLine < (Chassis.Route_limit.right + 7800) || Chassis.CDisable.Right_times > 300)
//					{
//						Chassis.CDisable.Right_flag = 0;
//						Chassis.Velocity.temp_Speed = fabs(Chassis.Velocity.temp_Speed);
//					}
				}
*/			
				}	
			}

			if(Chassis.CDisable.Left_flag == 0)
			{
				Chassis.CDisable.Left_times = 0;
			}
			if(Chassis.CDisable.Right_flag == 0)
			{
				Chassis.CDisable.Right_times = 0;
			}
		}
		
    // Debug防护
    if (ChassisDisable_Debug == 1)
    {
        Chassis.EM->OutputCurrent = 0;
    }


    //若加裁判系统的底盘功率限制
    for (int i = 0; i < 1; i++)
    {
        // Chassis.Power_Currents[i] = M3508s[i].OutputCurrent; //将PID输出的值作为功率限制的输入的电流值
        Chassis.Power_Currents[i] = Chassis.EM->OutputCurrent; //将PID输出的值作为功率限制的输入的电流值
    }

    PowerLimit_Processing(&Chassis_PowerLimit, Chassis.Power_Currents, 1);

    for (int i = 0; i < 1; i++)
    {
        // M3508s[i].OutputCurrent = Chassis.Power_Currents[i]; //确定限制后各轮子的实际电流输出
        Chassis.EM->OutputCurrent = Chassis.Power_Currents[i];
    }
	
}
