#include "PowerLimit_Control.h"

PowerLimit_t Chassis_PowerLimit;
void PowerLimit_Calculate(PowerLimit_t* powerlimit)
{

	//wzjun_brother 的功率限制 + 自我的理解和修改
	//实际上的思想就是根据一定的比例去缩小应该输出的总电流值
	
#if PowerLimit_Way == 0
		//更新数据计时
		powerlimit->Power_DelayCNT ++ ;
			
		float Used_PowerBuffer;                   //用掉的缓冲功率
		
		if(Judge_Monitor.DataUpdate_Flag == 1)
		{
			
			//判断服务器是否连接
			if(Judge_Monitor.Connect_OffFlag == 1)
			{
				//最大功率
				powerlimit->Max_Power = 60.0f;
				//将分母置为最大功率30.0f
				powerlimit->PowerRatio_Denominator = 40.0f;
			}
			else if(Judge_Monitor.Connect_OffFlag == 0)
			{
				//最大功率
				powerlimit->Max_Power = 200.0f;
				//将分母置为最大功率200.0f;
				powerlimit->PowerRatio_Denominator = 75.0f;				//75空底盘无云台
			}
			
			//实时的获取底盘的缓冲能量
			powerlimit->PowerBuffer = ext_power_heat_data.data.chassis_power_buffer;
			//更新标志位清零
			Judge_Monitor.DataUpdate_Flag = 0;
			//更新数据计时清0
			powerlimit->Power_DelayCNT = 0;
			
			Used_PowerBuffer = powerlimit->Max_Power -  powerlimit->PowerBuffer;                    //最大的缓冲能量 - 实时的缓冲能量
		
			if(Used_PowerBuffer > 0)
			{
				powerlimit->PowerRatio = (float)powerlimit->PowerBuffer /powerlimit->PowerRatio_Denominator;    				//有缓冲能量被用掉，实际上分母需要多大，得实测一下
				if(powerlimit->PowerRatio > 1)
				{
					powerlimit->PowerRatio = 1;
				}
			}
			else 
			{
					powerlimit->PowerRatio = 1;                 //无缓冲功率被用掉时
			}
		}
	
		//当裁判系统数据丢失了，300ms都无更新数据，则强制将实时的缓冲能量置为一个很小的值
		//或根本就无帧率
		if(powerlimit->Power_DelayCNT > 300 || Judge_Monitor.Offline_Flag == 1)
		{
			powerlimit->PowerRatio = 1;					//改好不用到缓冲能量的值
		}
		
    powerlimit->SumCurrent_OUT = powerlimit->SumCurrent_IN * pow(powerlimit->PowerRatio,2);  
		
#endif
	
	
	//大连交通的功率限制（大概复现）

#if PowerLimit_Way == 1 
	
	//更新数据计时
	powerlimit->Power_DelayCNT ++ ;
	
	if(Judge_Monitor.DataUpdate_Flag == 1)
	{
		//实时的获取底盘的缓冲能量
		powerlimit->PowerBuffer = ext_power_heat_data.data.chassis_power_buffer;
		//更新标志位清零
		Judge_Monitor.DataUpdate_Flag = 0;
		//更新数据计时清0
		powerlimit->Power_DelayCNT = 0;
	}
	
	//当裁判系统数据丢失了，300ms都无更新数据，则强制将实时的缓冲能量置为一个很小的值
	if(powerlimit->Power_DelayCNT > 300 || || Judge_Monitor.FPS == 0)
	{
		powerlimit->PowerBuffer = 10.0f;				
	}
	
	//输入电流总和等于输出电流总和
	powerlimit->SumCurrent_OUT = powerlimit->SumCurrent_IN;
	
	//当实时的缓冲能量小于开启功率限制的缓冲能量值时，开启限制
	//开启限制的这个值 是 需要调试的。
	//此值太大，则不能很充分的利用缓冲能量200J，
	//此值太小，会超功率或者在加速过程中有明显的卡顿
	if(powerlimit->PowerBuffer <= Chassis_Open_LimitPowerBuf)
	{
		//电机的总输出能量等于剩余能量的平方，再乘以一个系数
		powerlimit->SumCurrent_OUT = pow(powerlimit->PowerBuffer,2) * powerlimit->PowerRatio;
		
		//输入电流总和是否小于可输出的输出电流总和
		if(powerlimit->SumCurrent_IN < powerlimit->SumCurrent_OUT)
		{
			powerlimit->SumCurrent_OUT = powerlimit->SumCurrent_IN;
		}
	}
	
#endif
	
	//哈工大的功率限制（大概复现）
#if PowerLimit_Way == 2

	/* 平时的移动速度则直接和和对功率利用的充分性挂钩 
		 不超功率而限制过大会导致机器人加速慢，制动距离长，一些崎岖地形机动性非常差
		 
		 底盘电机匀速转动时的总功率和速度有关
		 平地移动主要是启停过程中会使机器人瞬时超功率,制动过程比启动过程更容易出现超功率情况
		 
		 裁判系统获取功率：裁判系统和我们通讯频率和其采集底盘功率的频率是不一样的，确保不出现意外丢数
		 
		 当前可用功率上限 指的是在你得到此时底盘缓冲能量后在程序设定的底盘电机功率之和允许的最大值。
		 机器人实际底盘功率小于规定的底盘功率限制（80W）时，是可以回复能量缓冲的。
		 因此如果一直在程序里把机器人使用的功率上限定为80W相当于浪费了很多能量缓冲。
		 因此这个当前可用功率上限是动态的。
		 
		 如何计算当前可用功率上限，我们采用的思路是：
		 设定一个缓冲能量危险值，
		 若按照当前功率上限使用底盘功率，下一次裁判系统和我们通讯时剩余缓冲能量恰好达到危险值。
		 即：W_缓冲-W_危险值 = t_功率通讯周期*P_当前可用功率上限
		 若缓冲能量达到或者小于危险值了，还是老老实实不要超裁判系统的功率上限吧
		 
		 3508和C620电调的电流环性能非常好，我们在这里可以认为其实际电流值完美跟随其目标电流值
		 因此认为发给电调的目标电流值乘以24V（底盘供电电压）近似等于该电机的功率
		 思路一：限制目标速度
		 计算PID，获得四个电机的电流目标值，
		 其和若大于当前可用功率限制，则同比例增大/缩小（取决于加速还是减速）速度。
		 再次计算，直至总功率小于限制值或达到设定循环次数上限（此时就可以直接削弱目标电流来保证）
		 思路二：限制最后的输出电流
		 在PID计算出四个电机目标电流后，直接乘以一个系数让他们之和小于等于当前可用功率上限
		 这种方法比较简单粗暴，对功率的运用十分充分，
		 但由于四个电机的电流不是线性关系，直接乘以一个系数可能导致运动失真，轮子转动不协调，出现走不了直线的情况。
	*/
		
			//更新数据计时
		powerlimit->Power_DelayCNT ++ ;
	
		if(Judge_Monitor.DataUpdate_Flag == 1)
		{
			//实时的获取底盘的缓冲能量
			powerlimit->PowerBuffer = ext_power_heat_data.data.chassis_power_buffer;
			//更新标志位清零
			Judge_Monitor.DataUpdate_Flag = 0;
			//更新数据计时清0
			powerlimit->Power_DelayCNT = 0;
		}
		//当裁判系统数据丢失了，300ms都无更新数据，则强制将实时的缓冲能量置为一个很小的值
		//比危险值还小的值，则就一直是在限制的功率下运行
		if(powerlimit->Power_DelayCNT > 300 || || Judge_Monitor.FPS == 0)
		{
			powerlimit->PowerBuffer = Chassis_Danger_PowerBuf - 10.0f;				
		}

		//若缓冲能量小于危险值，则老老实实不要超底盘的功率上限
		if(powerlimit->PowerBuffer <= Chassis_Danger_PowerBuf)
		{
			powerlimit->Max_Power = Chassis_Max_Power;
		}
		else
		{
			//计算出底盘当前可用的最大功率
			powerlimit->Max_Power = (float)(powerlimit->PowerBuffer - Chassis_Danger_PowerBuf) / Judge_Chassis_Times;
		}
		
		//第一种方式有点麻烦，判断后得去按一定比例去缩小目标速度后再继续PID计算出输出电流值
		//用第二种方式吧，直接用可用的最大功率除以电压值得到电流值，则为可输出的电流值
		//判断当前PID算出来的电流值总和*电压值
		if(powerlimit->SumCurrent_IN * 0.001f * Chassis_Voltage > powerlimit->Max_Power)
		{
			powerlimit->SumCurrent_OUT = (float)(powerlimit->Max_Power /  Chassis_Voltage) * 1000.0f;
		}
		else
		{
			powerlimit->SumCurrent_OUT = powerlimit->SumCurrent_IN;
		}
		
#endif

}

//功率限制执行函数
void PowerLimit_Processing(PowerLimit_t* powerlimit,float* WheelCurrent,int16_t amout)
{
    float coe[4] = {0.0};               //电流的占比系数
    uint8_t i = 0;                      
		
		powerlimit->SumCurrent_IN = 0;
		
    for(i = 0 ; i < amout ;i++)
    {
        powerlimit->SumCurrent_IN += abs(WheelCurrent[i]);              //获取输入的总电流之和
    }
    for(i = 0; i < amout ;i++ )
    {
        coe[i] = (float)((float)WheelCurrent[i]/(float)(powerlimit->SumCurrent_IN));                 //计算每个电流的百分占比
    }
		
    //进行功率解算
    PowerLimit_Calculate(powerlimit);

    //得到的输出总电流后再按百分比分配给个电机
    for(i = 0; i < amout ;i++ )
    {
        WheelCurrent[i] = (float)(powerlimit->SumCurrent_OUT * coe[i]);
    }

}


