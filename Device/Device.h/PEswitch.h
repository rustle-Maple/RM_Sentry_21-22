#ifndef PESWITCH_H
#define PESWITCH_H

#include "main.h"

typedef struct
{
	
	uint8_t PSwitch_L;
	uint8_t PSwitch_R;
	
}PSwitch_t;

extern PSwitch_t PSwitch_FLAG;

void Get_PSwitch_FLAG(void);


#endif


