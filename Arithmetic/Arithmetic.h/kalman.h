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
#define mat_trans   arm_mat_trans_f32//�������ת��
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

//һ�׿������ṹ�����
typedef struct 
{
  //�۲�
  float Xk;       //״ֵ̬
  float Zk;       //����ֵ
  float H;        //״̬�۲�ת������
  float Vk;       //�۲�����

  //Ԥ��
  float X_forecast;       //�ο��������ֵ 
  float LX_Optimal;       //��һ�̵����Ź���ֵ
  float Uk;               //״̬������
  float A;                //״̬ת������
  float B;                //���ƾ���
  float Wk;               //��������
  float P_forecast;       //�ο��������ֵЭ����
  float LP_Optimal;       //��һ�̵����Ź���ֵЭ����
  float A_T;              //A�����ת��
  float Q;                //��������Э����

  //����
  float Kg;           //����������
  float H_T;          //H�����ת��
  float R;            //�۲�����Э����
  float X_Optimal;    //�ο̵����Ź���ֵ
  float P_Optimal;    //�˿̵����Ź���ֵЭ����
  float I;            //��λ����

}FirstOrder_Kalman_t;


#define delta_t 0.004f       //��t�����������

//���׿�������ʼ���ṹ�����
//�ӱ���������Ĺ���
typedef struct
{
  //�۲�
  float Xk[2];       //״ֵ̬
  float Zk[2];       //����ֵ
  float H[4];        //״̬�۲�ת������

  //Ԥ��
  float X_forecast[2];       //�ο��������ֵ 
  float LX_Optimal[2];       //��һ�̵����Ź���ֵ
  float A[4];                //״̬ת������
  float P_forecast[4];       //�ο��������ֵЭ����
  float LP_Optimal[4];       //��һ�̵����Ź���ֵЭ����
  float A_T[4];              //A�����ת��
	float B[2];									//���ƾ���B
	float U[1];									//������
  float Q[4];                //��������Э����

  //����
  float Kg[4];           //����������
  float H_T[4];          //H�����ת��
  float R[4];            //�۲�����Э����
  float X_Optimal[2];    //�ο̵����Ź���ֵ
  float P_Optimal[4];    //�˿̵����Ź���ֵЭ����
  float I[4];            //��λ����

}SecondOrder_Kalman_Init_t;

//���׿������ṹ�����
//��һ����ȣ�������Ϊ�˾���
typedef struct 
{
  //�۲�
  mat Xk;       //״ֵ̬
  mat Zk;       //����ֵ
  mat H;        //״̬�۲�ת������

  //Ԥ��
  mat X_forecast;       //�ο��������ֵ 
  mat LX_Optimal;       //��һ�̵����Ź���ֵ
  mat A;                //״̬ת������
  mat P_forecast;       //�ο��������ֵЭ����
  mat LP_Optimal;       //��һ�̵����Ź���ֵЭ����
  mat A_T;              //A�����ת��
	mat B;								//���ƾ���
	mat U;								//������
  mat Q;                //��������Э����

  //����
  mat Kg;           //����������
  mat H_T;          //H�����ת��
  mat R;            //�۲�����Э����
  mat X_Optimal;    //�ο̵����Ź���ֵ
  mat P_Optimal;    //�˿̵����Ź���ֵЭ����
  mat I;            //��λ����

  //�˲���ɺ�ķ���ֵ
  float Filter_Value[2];

}SecondOrder_Kalman_t;

void FirstOrder_Kalman_Parameter_Init(FirstOrder_Kalman_t *p, float FK_R, float FK_Q);
float FirstOrder_Kalman_Calucate(FirstOrder_Kalman_t *p, float measure);

void SencondOrder_Kalman_Parameter_Init(SecondOrder_Kalman_Init_t *IF, SecondOrder_Kalman_t *F);
float* SencondOrder_Kalman_Calucate(SecondOrder_Kalman_t *F,float InputData1,float InputData2,float InputData3);

#endif

