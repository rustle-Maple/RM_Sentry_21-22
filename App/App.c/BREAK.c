#include "BREAK.h"
#include "main.h"

BREAK_e break_basic;

long M2006_targe_angle = 0;
int M2006_targe_speed = 0;
int send_to_break = 0;

int M2006_init_times = 0;            //初始化 计时
int M2006_init_change_angle = 0;     //初始化 角度变化量
int M2006_init_totalAngle_last = 0;  //初始化 上一时刻角度值

int stop_CH_OP_BC_BREAK_times=0;
bool start_use_break=0;//使用刹车,这是标志位
bool stop_CH_OP_BC_BREAK=0;

/**/
P_PID_t BREAK_ANGLE_pid;//刹车PID
P_PID_t BREAK_SPEED_pid;

uint8_t Break_static = 0;

void break_init(void) {
	
	if (DR16.Switch_Left  == 3 &&
		DR16.Switch_Right == 2) 
{
		if(DR16.ch4_DW>300)//拨下到一半
		{
	start_use_break=1;	
		}
		else
		{
	start_use_break=0;	
		}
}
	
	
  M2006_init_times++;

  if (M2006_init_times % 50 == 0)  //每100ms检测一次
  {
    M2006_init_change_angle =
        M3508s[BREAK_ID].totalAngle - M2006_init_totalAngle_last;
    M2006_init_totalAngle_last = M3508s[BREAK_ID].totalAngle;
  }

  
  
  if (DR16.Switch_Left == 1)  //左上挡/自动挡 就一定要先初始化刹车
  {
//    if (break_is_raedy == 0)  //刹车没有准备好
//      if (send_to_chassis == 0 && abs(M3508s[3].realSpeed) < 10)  //底盘已经失能
//      {
//      }
  } 
  
  
  else if (DR16.Switch_Left == 3 &&
             DR16.Switch_Right == 2)  //手动 初始化刹车,此时 摩擦轮速度为0
  {
	  /* 手动开启初始化 (单独判断遥控器)
    if (DR16.rc.ch4_DW >= 200)  //拨下
    {
      if (break_basic.STATE == 0)  //未初始化
      {
        break_basic.STATE = 1;
        M2006_init_times = 0;  //开始初始化就清0
      }
    }
	  */
	        if (break_basic.STATE == 0)  //未初始化
      {
		  if(start_use_break==1)
		  {
        break_basic.STATE = 1;
        M2006_init_times = 0;  //开始初始化就清0
			  
		  }
      }
    else if (break_basic.STATE == 1)  //刹车最大值没有准备好.正在初始化最大值
    {
      BREAK_SPEED_pid.Max_result = 5000;
      BREAK_SPEED_pid.Min_result = -5000;
      M2006_targe_angle = M3508s[BREAK_ID].totalAngle + 1999;
      if (M2006_init_times > 1000)  //给200*3=600ms 用来起步
      {
        if (abs(M2006_init_change_angle) < 200)  //角度变化小于200
        {
          //				if(abs(M3508s[BREAK_ID].realCurrent)>1500)//加入电流值判断
          break_basic.BREAK_MAX = M3508s[BREAK_ID].totalAngle;  //最大值找到了
          break_basic.STATE = 2;  //去初始化最小值
          M2006_init_times = 0;
        }
      }
    }
	else if (break_basic.STATE == 2)  //正在初始化最小值
    {
      BREAK_SPEED_pid.Max_result = 5000;
      BREAK_SPEED_pid.Min_result = -5000;
      M2006_targe_angle = M3508s[BREAK_ID].totalAngle - 1999;
      if (M2006_init_times > 1000)  //给200*3=600ms 用来起步
      {
        if (abs(M2006_init_change_angle) < 200)  //角度变化小于200
        {
          break_basic.BREAK_MIN = M3508s[BREAK_ID].totalAngle;  //最小值找到了
          break_basic.BREAK_MID =(break_basic.BREAK_MIN + break_basic.BREAK_MAX) /2;  //中间值找到了
			M2006_targe_angle=break_basic.BREAK_MID;
			BREAK_SPEED_pid.Max_result = 9000;
      BREAK_SPEED_pid.Min_result = -9000;
          break_basic.STATE = 3;  //初始化全部完成
        }
      }
    }
  }

  if (DR16.Switch_Left == 2)  //失能
  {
    M2006_init_times = 0;  //初始化计时清零
    if (break_basic.STATE == 2 ||
        break_basic.STATE == 1 || break_basic.STATE == 3)  //初始化到一半就失能了,直接从头开始初始化
    {
      break_basic.STATE = 0;
    }
  }
}


void break_control(void) 
{
	if(Robots_Control.Chassis_e == cs_Disable)
	{
		//失能
		send_to_break = 0;
		//清空变向刹车状态
		Chassis.Random.Break_flag = 0;
		//刹车状态清0
		Break_static = 0;
		//刹车目标值置为中值
		M2006_targe_angle = break_basic.BREAK_MID;	
	}
	//刹车初始化成功后
	if(break_basic.STATE == 3)
	{
		if(Robots_Control.Chassis_e == A_cs_Cruise || Robots_Control.Chassis_e == A_cs_Random)
		{
			if(Chassis.CDisable.Left_flag == 0 && Chassis.CDisable.Right_flag == 0)
			{
				//刹车的第一阶段，需要变向	Break_static：0：刹车未开始
				if(Chassis.Random.Break_flag == 1 && Break_static == 0)
				{
					Break_static = 1;											//刹车第一阶段
					if(Chassis.EM->realSpeed > 0)					//左刹车
					{
						//确定刹车电机2006的目标角度
						M2006_targe_angle = break_basic.BREAK_MAX + 3000;		//比大更大
						Break_static = 21;										//处于左刹阶段
					}
					else if(Chassis.EM->realSpeed < 0)		//右刹车
					{
						//确定刹车电机2006的目标角度
						M2006_targe_angle =  break_basic.BREAK_MIN -3000;			//比小更小 
						Break_static = 22;										//处于右杀阶段
					}
				}
				
				if(Break_static == 21)
				{
					if(M3508s[BREAK_ID].totalAngle > break_basic.BREAK_MAX + 3000)			//比最大值大一点点
					{
						send_to_break = 0;
					}
					if(Chassis.EM->realSpeed < 100)
					{
						Break_static = 31;
					}
				}
				else if(Break_static == 22)
				{
					if(M3508s[BREAK_ID].totalAngle < break_basic.BREAK_MIN - 3000)			//比最小值再小一点点
					{
						send_to_break = 0;
					}
					if(Chassis.EM->realSpeed > -100)
					{
						Break_static = 32;
					}
				}	
				
				//现在变向停留的时间太长了，观测一下功率看一下是否要速度低于（同向）某个较小值就反向使能 
				if(Break_static == 31)							//第2阶段的1类
				{
					if(Chassis.EM->realSpeed < -30)		//非0而是某个阈值
					{
						//抬起刹车
						M2006_targe_angle = break_basic.BREAK_MID - 1500;
						Break_static = 4;									
					}
				}
				else if(Break_static == 32)					//第二阶段的2类
				{
					if(Chassis.EM->realSpeed > 30)		//非0而是某个阈值
					{
						//抬起刹车
						M2006_targe_angle = break_basic.BREAK_MID + 1500;
						Break_static = 4;					
					}
				}
				
				if(Break_static == 4)						//第三阶段刹车完成
				{
					//清楚变向需要刹车标志位
					Chassis.Random.Break_flag = 0;
					//清楚刹车状态标志位
					Break_static = 0;
					//避免在变向过程中还一直计数
					Chassis.Random.Dir_times = 0;
				}
				

				if(Break_static == 1 || Break_static == 21 || Break_static == 22)
				{
					Chassis.EM->OutputCurrent = 0;			//底盘电机失能
				}
				
			}
			else if(Chassis.CDisable.Left_flag == 1 || Chassis.CDisable.Right_flag == 1)
			{
				//变向需要刹车的清0
				Chassis.Random.Break_flag = 0;
				//刹车状态清0
				Break_static = 0;
				//刹车的目标值置中
				if(Chassis.CDisable.Left_flag == 1)
				{
					M2006_targe_angle = break_basic.BREAK_MID + 1500;
				}
				else if(Chassis.CDisable.Right_flag == 1)
				{
					M2006_targe_angle = break_basic.BREAK_MID - 1500;
				}
				//避免在变向过程中还一直计数
				Chassis.Random.Dir_times = 0;
			}
		}
		else if(Robots_Control.Chassis_e == R_cs_Common)		//若处于手动状态
		{
			if(DR16.ch4_DW < -600)				//拨轮往上拨
			{
				M2006_targe_angle = break_basic.BREAK_MAX +3000; 		//往右刹车			
			}
			else if(DR16.ch4_DW > 600)
			{
				M2006_targe_angle = break_basic.BREAK_MIN -3000; 			//往左刹车
			}
			else				//其他情况下
			{
				M2006_targe_angle = break_basic.BREAK_MID;					//刹车抬起
			}
		}
		
	}
	else
	{
		Break_static = 0;
		Chassis.Random.Break_flag = 0;
	}
}



