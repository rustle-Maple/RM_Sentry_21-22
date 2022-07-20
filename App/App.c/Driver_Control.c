#include "Driver_Control.h"

Driver_t Driver;

//拨盘函数挂钩
Driver_FUN_t Driver_FUN = Driver_HOOK_FUN;

//将拨盘电机的角度清0
void Driver_Reset(M2006_t *driver_EM)
{
	driver_EM->lastAngle = 0;
	if (driver_EM->realAngle - driver_EM->lastAngle < -6000)
	{
		driver_EM->turnCount = 1;
	}
	if (driver_EM->lastAngle - driver_EM->realAngle < -6000)
	{
		driver_EM->turnCount = -1;
	}
	driver_EM->totalAngle = driver_EM->realAngle + driver_EM->turnCount * 8192;
	driver_EM->lastAngle = driver_EM->realAngle;
	driver_EM->targetAngle = driver_EM->totalAngle;
}
//拨盘扭矩(即转盘转速的检测)
bool DriverTorque_Detection(Driver_t *driver)
{
	return abs(driver->driver_EM->realSpeed) < driver->StuckBullet.critical_Speed;
}

void Driver_Init(void)
{
	//初始化上下拨盘对应的电机
	Driver.driver_EM = &M2006s[2];
	//初始化其目标值为total值防炸
	Driver.driver_EM->targetAngle = Driver.driver_EM->totalAngle;
	//拨弹的方向
	Driver.turn_Dir = 1;
	//单次卡弹的临界速度
	Driver.StuckBullet.critical_Speed = 1300;
	//单次卡弹的临界时间
	Driver.StuckBullet.critical_Times = 35;
	//反卡弹增量
	Driver.StuckBullet.delta_Angle = Angle_Bulletper * Driver.turn_Dir;
	//角度环的初始化
	P_PID_FUN.P_PID_Parameter_Init(&Driver.driver_EM->A_PID_t, 0.34f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.0f, 10000.0f, -10000.0f);
	//速度环的初始化
	P_PID_FUN.P_PID_Parameter_Init(&Driver.driver_EM->S_PID_t, 3.4f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.0f, 10000.0f, -10000.0f);
}

//拨盘控制
void Driver_Control(Driver_t *driver)
{
	if (Robots_Control.Attack_e == ak_Disable || Robots_Control.Attack_e == cease_Fire)
	{
		//将当前角度作为目标角度
		//确保不打弹式不会由于拨盘停下来的惯性多拨弹
		//driver->driver_EM->targetAngle = driver->driver_EM->totalAngle;
		//将要的弹丸数为0
		driver->will_Bullet_numbers = 0;
		//拨弹关闭
		driver->begin_Flag = 0;
		//拨弹完成的标志位不清0，放在热量检测去清0吧

		//卡弹参数
		driver->StuckBullet.flag = 0;		  //清除卡弹标志位
		driver->StuckBullet.times = 0;		  //卡弹的时长清0
		driver->StuckBullet.Angle = 0;		  //卡弹的当前角度
		driver->StuckBullet.target_Angle = 0; //反卡弹的目标角度为0

		if (Robots_Control.Attack_e == ak_Disable)
		{
			//直接使电机电流值为0
			driver->driver_EM->OutputCurrent = 0;
			//清除PID过程量
			P_PID_FUN.P_PID_Parameter_Clear(&driver->driver_EM->A_PID_t);
			P_PID_FUN.P_PID_Parameter_Clear(&driver->driver_EM->S_PID_t);
			return;
		}
	}
	else
	{
		/* -------------------------- 反卡弹的执行 -------------------------------------- */
		if (driver->StuckBullet.flag == 1)
		{
			if (driver->StuckBullet.times > driver->StuckBullet.critical_Times) //反卡弹状态是否开启
			{
				if (DriverTorque_Detection(driver)) //正反扭矩过大，需要反复横跳
				{
					driver->StuckBullet.times++;
					if (((int)(driver->StuckBullet.times / driver->StuckBullet.critical_Times)) % 2 == 0 && driver->StuckBullet.dir_Flag == 1) //卡弹时间在100-150 或其倍数之间
					{
						driver->StuckBullet.dir_Flag = 0;										 //再反向
						driver->StuckBullet.target_Angle -= 4 * driver->StuckBullet.delta_Angle; //究竟是＋还是- 看实际转向!
					}
					else if (((int)(driver->StuckBullet.times / driver->StuckBullet.critical_Times)) % 2 == 1 && driver->StuckBullet.dir_Flag == 0) //卡弹时间在50-100 或其倍数之间
					{
						driver->StuckBullet.dir_Flag = 1; //再反向
						driver->StuckBullet.target_Angle += 4 * driver->StuckBullet.delta_Angle;
					}
				}
				//扭矩不过大了,反卡弹成功
				if (abs(driver->driver_EM->totalAngle - driver->StuckBullet.target_Angle) <= Angle_Bulletper) //若当前拨盘角度与反卡弹的目标角度之间的距离小于一个弹位，则OK
				{
					driver->StuckBullet.flag = 0;		  //清除卡弹标志位
					driver->StuckBullet.times = 0;		  //卡弹的时长清0
					driver->StuckBullet.Angle = 0;		  //卡弹的当前角度
					driver->StuckBullet.target_Angle = 0; //反卡弹的目标角度为0
				}
				else
				{
					//卡弹的输出
					//左拨盘
					//外环
					P_PID_FUN.P_PID_Regulation(&driver->driver_EM->A_PID_t, driver->StuckBullet.target_Angle, driver->driver_EM->totalAngle);
					//内环
					P_PID_FUN.P_PID_Regulation(&driver->driver_EM->S_PID_t, driver->driver_EM->A_PID_t.result, driver->driver_EM->realSpeed);
					driver->driver_EM->OutputCurrent = driver->driver_EM->S_PID_t.result;
					return;
				}
			}
		}

		//有拨弹丸需求
		if (driver->will_Bullet_numbers > 0)
		{
			//拨弹还未开始，也就是已拨出需求的0颗
			if (driver->begin_Flag == 0)
			{
				//拨盘角度清零，进入就绪态
				Driver_Reset(driver->driver_EM);
				//确定需要转的角度
				driver->driver_EM->targetAngle += (Angle_Bulletper * driver->will_Bullet_numbers * driver->turn_Dir);
				driver->begin_Flag = 1;
			}
			//判断进入就绪态后，是否执行完拨弹
			if (abs(driver->driver_EM->totalAngle - driver->driver_EM->targetAngle) <= Angle_Bulletper / 8) //若拨盘电机的total值与目标值的距离小于8191，则拨转完成
			{
				//计算已经拨的弹丸数
				driver->ed_Bullet_numbers += driver->will_Bullet_numbers;
				//计算剩余的弹丸数
				driver->residue_Bullet_numbers = Driver_number_max - driver->ed_Bullet_numbers;
				if(driver->residue_Bullet_numbers < 0)
				{
					driver->residue_Bullet_numbers = 0;
				}
				//需拨弹量为0
				driver->will_Bullet_numbers = 0;
				//开启下次拨弹
				driver->begin_Flag = 0;
				//拨弹完成标志位
				driver->finish_Flag = 1;

				//卡弹参数
				driver->StuckBullet.flag = 0;		  //清除卡弹标志位
				driver->StuckBullet.times = 0;		  //卡弹的时长清0
				driver->StuckBullet.Angle = 0;		  //卡弹的当前角度
				driver->StuckBullet.target_Angle = 0; //反卡弹的目标角度为0
			}

			/* ---------------- 卡弹检查 ------------------- */
			//拨盘卡弹
			if (DriverTorque_Detection(driver)) //判断是否速度过低，扭矩过大
			{
				driver->StuckBullet.times++;										//卡弹的时长++
				if (driver->StuckBullet.times > driver->StuckBullet.critical_Times) //判断是否大于单次卡弹的极限时长
				{
					driver->StuckBullet.Angle = driver->driver_EM->totalAngle; //记录卡弹时的角度
					driver->StuckBullet.target_Angle = driver->StuckBullet.Angle + 2 * driver->StuckBullet.delta_Angle;
					driver->StuckBullet.dir_Flag = 1; //反转方向标志位
					driver->StuckBullet.flag = 1;	 //开启卡弹
				}
			}
			else
			{
				if (driver->StuckBullet.times > 0)
				{
					driver->StuckBullet.times -= 10;
				}
				else
				{
					driver->StuckBullet.times = 0;
				}
			}
		}
		else
		{
			driver->driver_EM->targetAngle = driver->driver_EM->totalAngle;
		}
	}

	//拨盘转动输出
	//左拨盘
	//外环
	P_PID_FUN.P_PID_Regulation(&driver->driver_EM->A_PID_t, driver->driver_EM->targetAngle, driver->driver_EM->totalAngle);
	//内环
	P_PID_FUN.P_PID_Regulation(&driver->driver_EM->S_PID_t, driver->driver_EM->A_PID_t.result, driver->driver_EM->realSpeed);
	driver->driver_EM->OutputCurrent = driver->driver_EM->S_PID_t.result;
}


