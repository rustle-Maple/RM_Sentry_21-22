#include "Snail.h"

//而Snail的控制信号行程为 400-2200μs
//而我们的PWM信号的频率为500Hz，即周期为0-2000μs，其对应的ccr值为0-1999

//要给Snail个油门值，即就是给个启动的占空比，只有给定此占空比。Snail才能启动
//要如何确定这个油门值呢？
//上电后会发出B_B_B：表示无PWM信号输入
//之后若听到BBBBBB的声音，说明给定的占空比过大要调小
//之后若没有听到声音，说明你给的占空比过小，要调大
//反复几次后，便会得到让Snail启动的占空比
//油门值
//uint16_t Accelerator_Value = 1000;			

//设置 行程 的值
//uint16_t Strock_Value = 660;					//行程的最大值一般为660	

Snail_t Snail;

//uint16_t a = 0;							//若结构体无法在运行之前设置最大行程值，则用全局变量

//Snail电机的初始化
void Snail_Init(void)
{
	//油门值
	Snail.Accelerator_Value = 1000;
	//行程值
	Snail.Strock_Value = 660;
	
	//使能TIM5的PWM模式
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	
	
	/*进入调试模式：
	
	//设置行程:
	
	//注意注意：
	//设置行程 和 转向时 ，初始化的ccr=0 即无脉冲信号
	//而最大行程值是你想要的值 比如1660
	//而最小脉冲宽度是油门值 是1000 而不是0
	
	//关于Snail电机PWM设置问题：
	//将 PWM 信号的脉宽设置到最大行程，电调与电机连接并上电。此时电机发出BB 和 BBB
	//交替的声音，间隔时间为 2 秒。在间隔时间内可以按照以下方法进行设置：
	//  a.   PWM行程校准
	//在 BB 声后的 2 秒内将PWM信号的脉宽设置到最小，直至电机发出约 1 秒的 B 声， 则
	//PWM行程校准完成。
	//  b.   电机转向切换
	//在 BBB 声后的 2 秒内将PWM信号的脉宽设置到最小，直至电机发出约 1 秒的 B 声， 表
	//示电机转向已更改。
	
	//Snail上电后要给占空比为0的信号一段时间，然后再给定占空比
	HAL_Delay(10000);
	
	//设置占空比的函数
	//__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,Snail.Accelerator_Value + Snail.Strock_Value);
	//__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,Snail.Accelerator_Value + Snail.Strock_Value);
	//__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,Snail.Accelerator_Value + Snail.Strock_Value);
	//__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,Snail.Accelerator_Value + Snail.Strock_Value);
	
	//再在调试中将
	//TIM的CCR的值设为 Strock_Value = 0;
	*/
	
}


