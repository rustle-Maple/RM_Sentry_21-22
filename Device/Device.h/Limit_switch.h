#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include "main.h"

typedef struct 
{
   uint8_t LSwitch_R_1;
   uint8_t LSwitch_R_2;
   uint8_t LSwitch_L_1;
   uint8_t LSwitch_L_2;
}Limit_switch_t;

extern Limit_switch_t Limit_switch;

void Get_LSwitch_FLAG(void);

#endif

