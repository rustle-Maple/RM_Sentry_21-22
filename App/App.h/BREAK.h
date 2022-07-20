#ifndef BREAK_H
#define BREAK_H

#include "main.h"
#include "M3508.h"
#include "DR16.h"

#define BREAK_ID 2
//#define M2006_TARGE_ANGLE_BIGGER M3508s[BREAK_ID].totalAngle+9999
#define M2006_TARGE_ANGLE_SMALLER M3508s[BREAK_ID].totalAngle-9999

void break_init(void);
void break_control(void);

typedef struct
{
	int BREAK_MAX;
	int BREAK_MID;
	int BREAK_MIN;
	int8_t STATE;//刹车状态 范围-128~127
	/*0 未初始化
      1 正在初始化最大值
	  2 正在初始化最小值

      3 初始化完成
      -1堵转    */
}BREAK_e;
extern BREAK_e break_basic;

extern long M2006_targe_angle;
extern int M2006_targe_speed;
extern int send_to_break ;
extern bool start_use_break;//使用刹车,这是标志位
extern int stop_CH_OP_BC_BREAK_times;
extern bool stop_CH_OP_BC_BREAK;
extern P_PID_t BREAK_ANGLE_pid;//刹车PID
extern P_PID_t BREAK_SPEED_pid;

extern uint8_t Break_static;			//刹车状态标志位
#endif


