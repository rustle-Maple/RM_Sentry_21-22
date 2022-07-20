#ifndef KALMAN_H
#define KALMAN_H

#include "main.h"
#include "arm_math.h"

#define mat         arm_matrix_instance_f32 //float
#define mat_64      arm_matrix_instance_f64 //double
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32//浮点矩阵转置
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

//一阶卡尔曼结构体变量
typedef struct 
{
  //观测
  float Xk;       //状态值
  float Zk;       //测量值
  float H;        //状态观测转换矩阵
  float Vk;       //观测噪声

  //预测
  float X_forecast;       //次刻先验估计值 
  float LX_Optimal;       //上一刻的最优估计值
  float Uk;               //状态控制量
  float A;                //状态转换矩阵
  float B;                //控制矩阵
  float Wk;               //过程噪音
  float P_forecast;       //次刻先验估计值协方差
  float LP_Optimal;       //上一刻的最优估计值协方差
  float A_T;              //A矩阵的转置
  float Q;                //过程噪声协方差

  //更新
  float Kg;           //卡尔曼增益
  float H_T;          //H矩阵的转置
  float R;            //观测噪声协方差
  float X_Optimal;    //次刻的最优估计值
  float P_Optimal;    //此刻的最优估计值协方差
  float I;            //单位矩阵

}FirstOrder_Kalman_t;


#define delta_t 0.004f       //Δt：计算的周期

//二阶卡尔曼初始化结构体变量
//从变量到矩阵的过渡
typedef struct
{
  //观测
  float Xk[2];       //状态值
  float Zk[2];       //测量值
  float H[4];        //状态观测转换矩阵

  //预测
  float X_forecast[2];       //次刻先验估计值 
  float LX_Optimal[2];       //上一刻的最优估计值
  float A[4];                //状态转换矩阵
  float P_forecast[4];       //次刻先验估计值协方差
  float LP_Optimal[4];       //上一刻的最优估计值协方差
  float A_T[4];              //A矩阵的转置
	float B[2];									//控制矩阵B
	float U[1];									//控制量
  float Q[4];                //过程噪声协方差

  //更新
  float Kg[4];           //卡尔曼增益
  float H_T[4];          //H矩阵的转置
  float R[4];            //观测噪声协方差
  float X_Optimal[2];    //次刻的最优估计值
  float P_Optimal[4];    //此刻的最优估计值协方差
  float I[4];            //单位矩阵

}SecondOrder_Kalman_Init_t;

//二阶卡尔曼结构体变量
//与一阶相比，变量变为了矩阵
typedef struct 
{
  //观测
  mat Xk;       //状态值
  mat Zk;       //测量值
  mat H;        //状态观测转换矩阵

  //预测
  mat X_forecast;       //次刻先验估计值 
  mat LX_Optimal;       //上一刻的最优估计值
  mat A;                //状态转换矩阵
  mat P_forecast;       //次刻先验估计值协方差
  mat LP_Optimal;       //上一刻的最优估计值协方差
  mat A_T;              //A矩阵的转置
	mat B;								//控制矩阵
	mat U;								//控制量
  mat Q;                //过程噪声协方差

  //更新
  mat Kg;           //卡尔曼增益
  mat H_T;          //H矩阵的转置
  mat R;            //观测噪声协方差
  mat X_Optimal;    //次刻的最优估计值
  mat P_Optimal;    //此刻的最优估计值协方差
  mat I;            //单位矩阵

  //滤波完成后的返回值
  float Filter_Value[2];

}SecondOrder_Kalman_t;

void FirstOrder_Kalman_Parameter_Init(FirstOrder_Kalman_t *p, float FK_R, float FK_Q);
float FirstOrder_Kalman_Calucate(FirstOrder_Kalman_t *p, float measure);

void SencondOrder_Kalman_Parameter_Init(SecondOrder_Kalman_Init_t *IF, SecondOrder_Kalman_t *F);
float* SencondOrder_Kalman_Calucate(SecondOrder_Kalman_t *F,float InputData1,float InputData2,float InputData3);

#endif

