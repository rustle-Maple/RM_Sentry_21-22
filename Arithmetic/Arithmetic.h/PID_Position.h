#ifndef PID_Position_H
#define PID_Position_H

#include "main.h"
#include "User_math.h"

#define P_PID_HOOK_FUN {\
  P_PID_Parameter_Init,\
  P_PID_Regulation,\
  P_PID_Parameter_Clear,\
}

extern uint16_t beta;
typedef struct
{
  float Kp; //比例系数
  float Ki; //积分系数
  float Kd; //微分系数

  float Target;  //目标值
  float Measure; //测量值

  float Error;     //偏差值
  float Epsilon;   //偏差检测阈值
  float max_error; //偏差的最大值
  float min_error; //偏差的最小值

  float Proportion;   //比例值
  float Integral;     //积分值
  float Differential; //微分值

  //不完全微分
  float alpha;         //不完全微分系数
  float D_Output;      //微分输出
  float D_Last_Output; //上一刻的微分输出

  float Max_antiwindup;       //抗积分饱和的输出最大值
  float Min_antiwindup;       //抗积分饱和的输出最小值

  float result;     //PID计算结构
  float Max_result; //result最大值
  float Min_result; //result最小值

  float LastError; //前一拍偏差
  float PreError;  //前两拍偏差

} P_PID_t;
typedef struct
{
	 void (*P_PID_Parameter_Init)(P_PID_t *P_PID, float Kp, float Ki, float Kd,
                          float epsilon, float max_error, float min_error,
                          float alpha,
                          float Max_antiwindup, float Min_antiwindup,
                          float Max_result, float Min_result);
  float (*P_PID_Regulation)(P_PID_t *P_PID, float target, float measure);
  void (*P_PID_Parameter_Clear)(P_PID_t *P_PID);
}P_PID_FUN_t;
extern P_PID_FUN_t P_PID_FUN;

void P_PID_Parameter_Init(P_PID_t *P_PID, float Kp, float Ki, float Kd,
                          float epsilon, float max_error, float min_error,
                          float alpha,
                          float Max_antiwindup, float Min_antiwindup,
                          float Max_result, float Min_result);
float P_PID_Regulation(P_PID_t *P_PID, float target, float measure);
void P_PID_Parameter_Clear(P_PID_t *P_PID);

#endif
