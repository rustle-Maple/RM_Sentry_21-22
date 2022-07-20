#include "Limit_switch.h"

Limit_switch_t Limit_switch;

void Get_LSwitch_FLAG(void)
{
    Limit_switch.LSwitch_R_1 = HAL_GPIO_ReadPin(Limit_switch_1_GPIO_Port,Limit_switch_1_Pin);
    Limit_switch.LSwitch_R_2 = HAL_GPIO_ReadPin(Limit_switch_2_GPIO_Port,Limit_switch_2_Pin);
    Limit_switch.LSwitch_L_1 = HAL_GPIO_ReadPin(Limit_switch_3_GPIO_Port,Limit_switch_3_Pin);
    Limit_switch.LSwitch_L_2 = HAL_GPIO_ReadPin(Limit_switch_4_GPIO_Port,Limit_switch_4_Pin);
}


