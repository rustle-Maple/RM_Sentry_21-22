#include "PEswitch.h"

PSwitch_t PSwitch_FLAG;


//获取光电开关的值

void Get_PSwitch_FLAG(void)
{
	//至于是左右光电位置怎么装，只要将两个变量调换位置就可了
	//还有一点，zjun师兄的代码里是上拉输入，我这里是浮空输入
	//等看了光电说明书，再说
	PSwitch_FLAG.PSwitch_R = HAL_GPIO_ReadPin(PEswitch_1_GPIO_Port,PEswitch_1_Pin);
	PSwitch_FLAG.PSwitch_L = HAL_GPIO_ReadPin(PEswitch_2_GPIO_Port,PEswitch_2_Pin);
	
}


