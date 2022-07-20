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
	int8_t STATE;//ɲ��״̬ ��Χ-128~127
	/*0 δ��ʼ��
      1 ���ڳ�ʼ�����ֵ
	  2 ���ڳ�ʼ����Сֵ

      3 ��ʼ�����
      -1��ת    */
}BREAK_e;
extern BREAK_e break_basic;

extern long M2006_targe_angle;
extern int M2006_targe_speed;
extern int send_to_break ;
extern bool start_use_break;//ʹ��ɲ��,���Ǳ�־λ
extern int stop_CH_OP_BC_BREAK_times;
extern bool stop_CH_OP_BC_BREAK;
extern P_PID_t BREAK_ANGLE_pid;//ɲ��PID
extern P_PID_t BREAK_SPEED_pid;

extern uint8_t Break_static;			//ɲ��״̬��־λ
#endif


