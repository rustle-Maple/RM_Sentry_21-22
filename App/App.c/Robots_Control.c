#include "Robots_Control.h"
#include "Chassis_Control.h"
#include "DR16.h"
#include "Vision_Control.h"
#include "RM_JudgeSystem.h"
#include "Cloud_Control.h"
#include "Heat_Control.h"
#include "M3508.h"
#include "Math.h"

Robots_Control_t Robots_Control;

//挂钩
Robots_Control_FUN_t Robots_Control_FUN = Robots_Control_HOOK_FUN;

uint8_t ChassisDisable_Debug = 0;

//更新DR16的状态控制
void Update_DR16_RobotStatus(void)
{
	/* 一级控制 */
	if (DR16.Switch_Left == DR16_SWITCH_DOWN)
	{
		Robots_Control.Master_e = Strike; //罢工
	}
	else if (DR16.Switch_Left == DR16_SWITCH_MID)
	{
		Robots_Control.Master_e = Remote; //遥控
	}
	else if (DR16.Switch_Left == DR16_SWITCH_UP)
	{
		Robots_Control.Master_e = Automation; //自动
	}
	/* 二级控制 */
	if (DR16.Switch_Right == DR16_SWITCH_DOWN)
	{
		Robots_Control.Slave_e = chassis_Enable; //底盘
	}
	else if (DR16.Switch_Right == DR16_SWITCH_MID)
	{
		Robots_Control.Slave_e = c_Cloud_Enable; //底盘+云台
	}
	else if (DR16.Switch_Right == DR16_SWITCH_UP)
	{
		Robots_Control.Slave_e = c_c_Autoaim_Enable; //底盘+云台+自瞄
	}
	/* 子级控制 */
	if (DR16.ch4_DW == ch4_DW_MID)
	{
		Robots_Control.Fire_e = Truce; //停火
	}
	else if (DR16.ch4_DW == ch4_DW_DOWN)
	{
		Robots_Control.Fire_e = Sag; //萎靡
	}
	else if (DR16.ch4_DW == ch4_DW_UP)
	{
		Robots_Control.Fire_e = Onslaught; //猛攻			（自动模式下，强制处于此状态）
	}
	
	//额外加的调试
	if(DR16.Switch_Left == 2 && DR16.ch3 < -600)
	{
		ChassisDisable_Debug = 1;
	}
	else if(DR16.Switch_Left == 2 && DR16.ch3 > 600)
	{
		ChassisDisable_Debug = 0;
	}
	
	
	
}
/* 子级控制 */
//跟新视觉状态控制
void Update_Vision_RobotStatus(void)
{
		if (VisionData.RawData.Armour == 1 && VisionData.RawData.Depth > 0)
		{
			Robots_Control.Vision_e = AutoAim_Enable;
		}
		else
		{
			Robots_Control.Vision_e = AutoAim_Disable;
		}
}
//更新控制状态变量(需要一直更新的)
void Updata_Control_RobotStatus(void)
{
	Robots_Control.Status_t.real_Status = Robots_Control.Master_e;
	if (Robots_Control.Status_t.real_Status == Automation && Robots_Control.Status_t.last_Status != Automation)
	{
		Robots_Control.Status_t.change_Status = Enable; //（初始状态此控制状态切换也必为0/Disable）
		//行程为限制
		Chassis.Route_limit.flag = 0; //（初始状态行程限制标志位得置0）
		Chassis.Route_limit.time = 0;       //时间
    Chassis.Route_limit.right_flag = 0; //右限制
    Chassis.Route_limit.left_flag = 0;	
		//前哨战状态也的清
		Robots_Control.Outpost_Status = surviving;
		//并将其左右限位清0
		Chassis.Route_limit.right = -999999;
		Chassis.Route_limit.left = 999999;
	}
	Robots_Control.Status_t.last_Status = Robots_Control.Status_t.real_Status;
}

//uint8_t jjjj = 0;
//更新前哨站血量状态(只有在行程限制完成后才需要持续更新，且在前哨站被毁后可不更新)
void Updata_Outpost_RobotStatus(void)
{
	if (ext_game_robot_state.data.robot_id == RedSentry)
	{
		if (ext_game_robot_HP.data.red_outpost_HP == 0)
		{
			Robots_Control.Outpost_Status = Destroyed; 
		}
		else
		{
			Robots_Control.Outpost_Status = surviving;	//（初始化时为幸存状态）
		}
	}
	else if (ext_game_robot_state.data.robot_id == BlueSentry)
	{
		if (ext_game_robot_HP.data.blue_outpost_HP == 0)
		{
			Robots_Control.Outpost_Status = Destroyed;
		}
		else
		{
			Robots_Control.Outpost_Status = surviving;
		}
	}
	#if Stop_game == 1
//	Robots_Control.Outpost_Status = jjjj;
	#endif
}
//只有当前哨站永不存在了，才需要对掉血进行处理（所以只需要在前哨站不复存在了才需要更新，且由于大出血优先级最高，所以得持续检测）
void Updata_Dropblood_RobotStatus(void)
{
	Robots_Control.Harm_t.sample_time++;
	Robots_Control.Harm_t.real_HP = ext_game_robot_state.data.remain_HP;
	uint16_t increment_HP = abs(Robots_Control.Harm_t.real_HP - Robots_Control.Harm_t.last_HP);
	if (increment_HP >= Robots_Control.Harm_t.critical_value_t.small_Dropblood && increment_HP <= Robots_Control.Harm_t.critical_value_t.big_Dropblood) 
	{
		Robots_Control.Move_Time.nodropblood_Times = 0;
		Robots_Control.Harm_e = small_Dropblood; //（初始化时，必为无掉血模式）
		//调试
	//	Robots_Control.Harm_e = no_Dropblood;
	}
	else if (increment_HP > Robots_Control.Harm_t.critical_value_t.big_Dropblood)
	{
		Robots_Control.Move_Time.nodropblood_Times = 0;
		Robots_Control.Harm_e = big_Dropblood;
		//调试
		Robots_Control.Harm_e = small_Dropblood;
	}
	else if (increment_HP == 0 && Robots_Control.Move_Time.nodropblood_Times >= Robots_Control.Harm_t.critical_value_t.NoDropBlood_time)
	{
		//其实清不清都可以，清可以防止溢出，即使清了之后并不符合条件，
		//但是掉血状态还是保持上一刻的，其他掉血条件也并不符合
		Robots_Control.Move_Time.nodropblood_Times = 0;
		Robots_Control.Harm_e = no_Dropblood;
	}
	//无掉血的持续时间
	if (increment_HP == 0)
	{
		Robots_Control.Move_Time.nodropblood_Times++; //在狂暴状态执行过程中，其实也无必要清0，由于狂暴状态所处时长远小于无扣血状态的临界时长
	}
	//采样血量时间
	if (Robots_Control.Harm_t.sample_time > Robots_Control.Harm_t.critical_value_t.sample_time)
	{
		Robots_Control.Harm_t.sample_time = 0;
		Robots_Control.Harm_t.last_HP = Robots_Control.Harm_t.real_HP;
	}
}
//狂暴状态计时(只有处于前哨战不复存在时才需要)
void Updata_Frenzytimes_RobotStatus(void)
{
	//进入狂暴状态则开始累计狂暴时间
	if (Robots_Control.Chassis_e == A_cs_Frenzy)
	{
		Robots_Control.Move_Time.enter_Frenzy_Times++;
		Robots_Control.Status_t.Frenzy_Status = 1; //为什么要有此标志位？是为了血量检测时，打断狂暴状态的运行
	}
	//若狂暴状态持续时间超过2.5s
	if (Robots_Control.Move_Time.enter_Frenzy_Times >= Robots_Control.Move_Time.critical_time_t.FrenzyContinue)
	{
		//则强制改为随机模式，为了退出狂暴状态
		Robots_Control.Chassis_e = A_cs_Random;
		//清除处于狂暴状态的标志位
		Robots_Control.Status_t.Frenzy_Status = 0;
		//清楚进入狂暴状态的累计时间
		Robots_Control.Move_Time.enter_Frenzy_Times = 0;
		//将上次处于狂暴状态标志位置1
		Robots_Control.Status_t.last_Frenzy_Status = 1;
	}
	if (Robots_Control.Status_t.last_Frenzy_Status == 1)
	{
		//上次处于暴走状态，累计退出暴走状态的时间
		Robots_Control.Move_Time.exit_Frenzy_Times++;
		//判断退出暴走状态时间实是否充足
		if (Robots_Control.Move_Time.exit_Frenzy_Times >= Robots_Control.Move_Time.critical_time_t.FrenzyExit)
		{
			//充足则上次处于暴走状态给清除
			Robots_Control.Status_t.last_Frenzy_Status = 0;
			//清除退出暴走状态时间
			Robots_Control.Move_Time.exit_Frenzy_Times = 0;
			//刚进入狂暴模式
			Chassis.Frenzy.just_flag = 0; //（此变量初始化时要置0，则一开始就是刚进入狂暴状态的，才能使得能处于狂暴状态的速度）
		}
	}
}
//热量状态控制(热量状态的检测需要一直有)
void Updata_Heat_RobotStatus(void)
{
	Barrel_Heat_FUN.user_Calculate_Heat();
	if (Barrel.user_realHeat >= Danger_Heat)
	{
		Robots_Control.Heat_Status = Greater_than_Danger; //（初始状态时，要将其初始化为Small_Enough）
	}
	else if (Barrel.user_realHeat < (Danger_Heat - Snipe_critical_Heat))
	{
		Robots_Control.Heat_Status = Small_Enough;
	}
}
void Updata_edBullet_RobotStatus(void)
{
	//初始时都是未拨够
	Robots_Control.edBullet_Status = edBullet_inequacy;
	
	//拨狗20颗时才为已经拨够
	if(Driver.ed_Bullet_numbers - Driver.last_ed_Bullet_numbers >= 20)
	{
		//将这刻的弹数记录下来
		Driver.last_ed_Bullet_numbers = Driver.ed_Bullet_numbers;
		//已经以最高频最快速连续打击了20发
		Robots_Control.edBullet_Status = edBullet_adequate;
	}

}
//狙击模式时间的状态控制（只有在狙击状态才需要对狙击时间进行检测）
void Updata_Snipetime_RobotStatus(void)
{
	if(Robots_Control.Snipe_Status.ing_Flag == 1 && Robots_Control.Snipe_Status.exit_Flag == 0 && Robots_Control.edBullet_Status == edBullet_inequacy)
	{
		Robots_Control.Snipe_Status.exit_status_e = entirely_Flag;
	}
	//若处于狙击状态 并且 热量已经不足以发射20颗弹了
	if (Robots_Control.Snipe_Status.ing_Flag == 1 && Robots_Control.edBullet_Status == edBullet_adequate) //（ing_Flag 初始状态先置0，只有当其处于狙击态时才置1）
	{
		//先将处于狙击给置0
		Robots_Control.Snipe_Status.ing_Flag = 0;
		//退出狙击状态给置1
		Robots_Control.Snipe_Status.exit_Flag = 1; //（exit_Flag 初始状态也得置0，这样保证只有进入狙击态后，热量不足后才退出狙击态）
	}
	//当处于退出狙击状态了
	if (Robots_Control.Snipe_Status.exit_Flag == 1)
	{
		//累计退出狙击状态的时间
		Robots_Control.Snipe_Status.exit_times++;
	}
	 //当其退出狙击态时间大于0了，才需要去退出阶段的各个状态判，不让就无需判断，减少计算量
	if (Robots_Control.Snipe_Status.exit_times > 0)
	{
		//若退出狙击状态的时间处于前半段
		if (Robots_Control.Snipe_Status.exit_times < Robots_Control.Snipe_Status.exit_crash_time_t.latterpart_time	 //（此时间值需要初始化）
			&& Robots_Control.Snipe_Status.exit_times > Robots_Control.Snipe_Status.exit_crash_time_t.frontpart_time //（此时间值需要初始化并且初始化为0）
		)
		{
			//则将退出狙击状态置为前半段
			Robots_Control.Snipe_Status.exit_status_e = frontpart_Flag; //（初始状态需要初始为entirely_Flag，即为完全退出狙击态）
		}
		//若退出狙击状态的时间处于后半段
		else if (Robots_Control.Snipe_Status.exit_times > Robots_Control.Snipe_Status.exit_crash_time_t.latterpart_time && Robots_Control.Snipe_Status.exit_times < Robots_Control.Snipe_Status.exit_crash_time_t.entirely_time)
		{
			//则将退出狙击状态置为后半段
			Robots_Control.Snipe_Status.exit_status_e = latterpart_Flag;
		}
		//若退出狙击状态的时间大于后半段
		else if (Robots_Control.Snipe_Status.exit_times > Robots_Control.Snipe_Status.exit_crash_time_t.entirely_time)
		{
			//则将退出狙击状态置为完全退出
			Robots_Control.Snipe_Status.exit_status_e = entirely_Flag;
		}
		//当退出狙击状态为完全退出了，退出狙击状态标志位置0，退出狙击状态的时间清0
		if (Robots_Control.Snipe_Status.exit_status_e == entirely_Flag)
		{
			Robots_Control.Snipe_Status.exit_Flag = 0;
			Robots_Control.Snipe_Status.exit_times = 0;
		}
	}
}
/* 更新直接状态控制 */
void Updata_Direct_RobotStatus(void)
{
	//一级控制：失能
	if (Robots_Control.Master_e == Strike)
	{
		//直接控制：
		Robots_Control.Chassis_e = cs_Disable; //底盘失能
		Robots_Control.Cloud_e = cd_Disable;   //云台失能
		Robots_Control.Attack_e = ak_Disable;  //攻击失能
	}

	//一级控制：遥控
	else if (Robots_Control.Master_e == Remote)
	{
		//二级控控制：

		//直接控制：
		Robots_Control.Chassis_e = R_cs_Common; //底盘 遥控 普通
		Robots_Control.Cloud_e = R_cd_Common;	//云台 遥控 普通
		//子级控制


		//底盘使能（云台和自瞄失能）
		if (Robots_Control.Slave_e == chassis_Enable)
		{
			Robots_Control.Cloud_e = cd_Disable; //云台 失能
		}
		//底盘+云台使能
		else if (Robots_Control.Slave_e == c_Cloud_Enable)
		{
			Robots_Control.Cloud_e = R_cd_Common; //云台 遥控 普通
		}
		//底盘+云台+自瞄
		else if (Robots_Control.Slave_e == c_c_Autoaim_Enable)
		{
			if(Robots_Control.Vision_e == AutoAim_Enable)
			{
				Robots_Control.Cloud_e = cd_AutoAim; //云 自瞄
			}
			else 
			{
				Robots_Control.Cloud_e = R_cd_Common; //云 自瞄
			}
		}

		//只有云台无失能时才能用发射
		if (Robots_Control.Cloud_e != cd_Disable)
		{
			//若拨轮往下滑
			if (Robots_Control.Fire_e == Sag)
			{
				Robots_Control.Attack_e = R_ak_Sag_Nooverheat;
			}
			else if (Robots_Control.Fire_e == Onslaught)
			{
				Robots_Control.Attack_e = /*R_ak_Onslaught_Nooverheat*/A_ak_Frequency;
			}
			else if (Robots_Control.Fire_e == Truce)
			{
				Robots_Control.Attack_e = cease_Fire;
			}
		}
		//云台失能了，那么发射也就不能用
		else
		{
			Robots_Control.Attack_e = ak_Disable;
		}
	}

	//一级控制：自动
	else if (Robots_Control.Master_e == Automation)
	{
		/* 这里被强制执行了，导致下面条件所有的Cloud_e 并非像实际看到的一样是autoaim，所以其条件看似成立，实则不成立*/
		//云台 扫描
		if(Robots_Control.Vision_e == AutoAim_Enable)
		{
			Robots_Control.Cloud_e = cd_AutoAim;
		}
		else if(Robots_Control.Vision_e == AutoAim_Disable)
		{
			Robots_Control.Cloud_e = A_cd_Scan;
		}
		
		//底盘：
		//先判断是否是从遥控状态切换为自动状态，
		if (Robots_Control.Status_t.change_Status == Enable)
		{
			//只有这样才需要进入轨道行程限制模式
			Robots_Control.Chassis_e = A_cs_Pathway;
			//唯有在行程限制确定好后才可以使此条件为不成立
			//而进入对前哨战状态的判断
			if (Chassis.Route_limit.flag == 1)
			{
				//但限制完了就已经处于自动状态了，也没有从遥控切换到自动了
				Robots_Control.Status_t.change_Status = Disable;
			}
		}
		//只有当行程确定了，你才可以去执行前哨战幸存模式
		if (Chassis.Route_limit.flag == 1)
		{
			//只有行程确定完了才需要去判断前哨战的血量，而且前哨战不复存在了就不用去判断了
			Robots_Control_FUN.Updata_Outpost_RobotStatus();
			if (Robots_Control.Outpost_Status == surviving)
			{
				//底盘：处于前哨站幸存模式
				Robots_Control.Chassis_e = A_cs_Outpost;
				//用于调试，不要前哨战模式
//				Robots_Control.Outpost_Status = Nolonger;
			}
			else if (Robots_Control.Outpost_Status == Destroyed)
			{
				Robots_Control.Chassis_e = A_cs_Cruise;
				//必须多此状态，可以将前哨战被摧毁此条件给抵消掉，就可以顺利进入通过血量变化率去控制自走
				//又可以让前哨战被摧毁那一刻，就有相应的运动，不会说停在那等上几秒
				Robots_Control.Outpost_Status = Nolonger;
			}
		} //此刻是没有清除行程限制完成标志位的，即是下次重新进去上面的判断也没事，里面只是一些前哨战状态的确定罢了

		//只有当前哨战不复存在了，才需要更具优先级最高的血量的变化率去决定运动策略
		if (Robots_Control.Outpost_Status == Nolonger)
		{
			//判断发射机构是否断电,那么就只跑底盘，云台直接失能
//			if (M3508s[3].OffLine_Status == 1)
//			{
//				Robots_Control.Cloud_e = cd_Disable;
//			}
			//只有前哨站不复存在了，才需要去判断掉血情况
			Robots_Control_FUN.Updata_Dropblood_RobotStatus();
			//也才需要去判断狂暴的运行时间
			Robots_Control_FUN.Updata_Frenzytimes_RobotStatus();
						
			//然后分为三种状态：识别到敌人且不大出血（追击），或无识别到敌人，或识别到敌人但是处于大出血
			if (Robots_Control.Cloud_e == cd_AutoAim)
			{
				//进入狙击状态
				//只有在自瞄时才需要检测狙击时间
				Robots_Control_FUN.Updata_Snipetime_RobotStatus();
				//只有完全退出狙击态才可用进入狙击态
				if(Robots_Control.Snipe_Status.exit_status_e == entirely_Flag)
				{
					Robots_Control.Chassis_e = A_cs_Pursue;
				}
				//退出狙击态的前段时间,后端时间，都处于随机模式
				else if(Robots_Control.Snipe_Status.exit_status_e == frontpart_Flag || Robots_Control.Snipe_Status.exit_status_e == latterpart_Flag)
				{
					Robots_Control.Chassis_e = A_cs_Random;
				}
				//若在狙击状态中处于大出血，那么也重新回到狂暴状态
				if (Robots_Control.Harm_e == big_Dropblood)
				{
					//且上刻未处于狂暴状态
					if (Robots_Control.Status_t.last_Frenzy_Status == 0)
					{
						Robots_Control.Chassis_e = A_cs_Frenzy;
					}
					else //若上次处于狂暴状态了,则处于随机模式
					{
						Robots_Control.Chassis_e = A_cs_Random;
					}
				}
			}
			else if (Robots_Control.Cloud_e == A_cd_Scan)
			{
				//丢失了视野，应该把狙击态的所有标志位归为初始化，下次识别到了次啊可以重新进入
				//正处于狙击态变量置0
				Robots_Control.Snipe_Status.ing_Flag = 0;
				//也是完全退出狙击态
				Robots_Control.Snipe_Status.exit_status_e = entirely_Flag;
				//并且累计的退出狙击态时间也清0
				Robots_Control.Snipe_Status.exit_times = 0;
				//退出狙击状态给置0
				Robots_Control.Snipe_Status.exit_Flag = 0;
			
				//正常的判断掉血量 和 狂暴时长 去行走
				if (Robots_Control.Harm_e == small_Dropblood && Robots_Control.Status_t.Frenzy_Status == 0) //处于小出血 并且未处于狂暴状态中的
				{
					//处于随机模式
					Robots_Control.Chassis_e = A_cs_Random;
				}
				else if (Robots_Control.Harm_e == big_Dropblood && Robots_Control.Status_t.last_Frenzy_Status == 0) //处于大出血状态，且上刻未处于狂暴状态的
				{
					//处于狂暴模式
					Robots_Control.Chassis_e = A_cs_Frenzy;
				}
				else if (Robots_Control.Harm_e == no_Dropblood && Robots_Control.Status_t.Frenzy_Status == 0) //长时间无扣血的，并且不处于狂暴状态中的
				{
					Robots_Control.Chassis_e = A_cs_Cruise;
				}
			}
		}

		//攻击：
		//若底盘处于轨道行程限制模式
		if (Robots_Control.Chassis_e == A_cs_Pathway)
		{
			//则攻击处于停火模式
			Robots_Control.Attack_e = cease_Fire;
		}
		//只有但轨道行程限制好了之后才有前哨战幸存模式
		//从前哨战幸存模式开始，即使非Robots_Control.Chassis_e ！= A_cs_Pathway 时，即都是通过是否识别到敌人开启什么打击方式
		if(Robots_Control.Chassis_e != A_cs_Pathway)
		{
			if(Robots_Control.Cloud_e == cd_AutoAim)
			{
				//判断是处于哪种底盘的运动模式
				if(Robots_Control.Chassis_e == A_cs_Pursue)
				{
					Robots_Control.Attack_e = A_ak_Snipe;
				}
				else if(Robots_Control.Chassis_e == A_cs_Random)
				{
					//处于退出狙击状态的前半段
					if(Robots_Control.Snipe_Status.exit_status_e == frontpart_Flag)
					{
						Robots_Control.Attack_e = A_ak_Frequency;
					}
					else if(Robots_Control.Snipe_Status.exit_status_e == latterpart_Flag)
					{
						Robots_Control.Attack_e = A_ak_Nooverheat;
					}
				}
				//若快超热量了，即使瞄到了也不打
				if(Robots_Control.Heat_Status == 	Greater_than_Danger)
				{
					Robots_Control.Attack_e = cease_Fire;
				}
			}
			else if(Robots_Control.Cloud_e == A_cd_Scan)
			{
				Robots_Control.Attack_e = cease_Fire;
			}
		}

		//二级控控制：
		//底盘使能（云台+自瞄 失能）
		if (Robots_Control.Slave_e == chassis_Enable)
		{
			Robots_Control.Cloud_e = cd_Disable;
		}
		//底盘+云台使能 (自瞄 失能)
		else if (Robots_Control.Slave_e == c_Cloud_Enable)
		{
			Robots_Control.Cloud_e = A_cd_Scan;
		}
		//底盘+云台+自瞄
		else if (Robots_Control.Slave_e == c_c_Autoaim_Enable)
		{
			if(Robots_Control.Vision_e == AutoAim_Enable)
			{
				Robots_Control.Cloud_e = cd_AutoAim; //云 自瞄
			}
			else 
			{
				Robots_Control.Cloud_e = A_cd_Scan; //云 自瞄
			}
		
		}
	}
}
//控制状态的初始化
void Init_RobotStatus(void)
{
	//掉血相关的初始化
	Robots_Control.Harm_t.real_HP = Robots_Control.Harm_t.last_HP = 600.0f;	//初始状态血量要为600，不然第一次进入掉血检测的时候就会进入疯狂模式
	Robots_Control.Harm_t.critical_value_t.small_Dropblood = 10;
	Robots_Control.Harm_t.critical_value_t.big_Dropblood = 40;
	Robots_Control.Harm_t.critical_value_t.NoDropBlood_time = 8000;
	Robots_Control.Harm_t.critical_value_t.sample_time = 500;
	//狂暴状态的相关初始
	Robots_Control.Move_Time.critical_time_t.FrenzyContinue = 2500;
	Robots_Control.Move_Time.critical_time_t.FrenzyExit = 4000;
	//狙击态相关的初始化
	Robots_Control.Snipe_Status.exit_crash_time_t.frontpart_time = 0;//必为0 (范围 0 ~ ？)
	Robots_Control.Snipe_Status.exit_crash_time_t.latterpart_time = 3000; 
	Robots_Control.Snipe_Status.exit_crash_time_t.entirely_time = 6000;
}
//控制状态更新
void Updata_RobotStatus(void)
{
	/*无论何时都得随时更新状态的*/
	Update_DR16_RobotStatus();			//dr16控制
	Update_Vision_RobotStatus();		//视觉控制
	Updata_Control_RobotStatus();		//控制状态
	Updata_Heat_RobotStatus();			//热量
	Updata_Direct_RobotStatus();		//直接控制
}

