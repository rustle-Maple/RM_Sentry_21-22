#include "Kalman.h"

//��ʼ��һ�׿���������
void FirstOrder_Kalman_Parameter_Init(FirstOrder_Kalman_t *p, float FK_R, float FK_Q)
{
  //�۲�
  p->H = 1;  //״̬�۲�ת������

  //Ԥ��
  p->A = 1;          //״̬ת������
  p->A_T = p->A;     //A�����ת�ã�һ�׵�ת��=����
  p->Q = FK_Q;       //��������Э����

  //����
  p->H_T = p->H;    //H�����ת��
  p->R = FK_R;      //�۲�����Э����
  p->LX_Optimal = 0; //�ο̵����Ź���ֵ,һ��Ϊ0
  p->LP_Optimal = 1;//(�����ǳ�ʼ�����˿̣����൱���Ժ����һ��)�˿̵����Ź���ֵЭ���һ�����Ź���ֵЭ�����ʼ��Ϊ1������Ϊ0���������ε�����ͻ�������
  p->I = 1;         //��λ����
}

//һ�׿��������㺯��
float FirstOrder_Kalman_Calucate(FirstOrder_Kalman_t *p, float measure)
{
  //�۲�
  //�������� Z(k) = H * X(k) + V(k);
  p->Xk = measure;
  p->Zk = p->H * p->Xk + p->Vk;

  //Ԥ��
  //�������ֵ X(k|k-1) = A * X(k-1|k-1) + B * U(k);
  p->X_forecast = p->A * p->LX_Optimal + p->B * p->Uk;
  //�������ֵЭ���� P(k|k-1) = A * P(k-1|k-1) * A' + Q;
  p->P_forecast = p->A * p->LP_Optimal * p->A_T + p->Q;

  //����
  //���������� Kg(k) = (P(k|k-1) * H' / (H * P(k|k-1) * H' + R));
  p->Kg = p->P_forecast * p->H_T / (p->H * p->P_forecast * p->H_T + p->R);
  //�������ֵ X(k|k) = X(k|k-1) + Kg(k) * (Zk - H * X(k|k-1));
  p->X_Optimal = p->X_forecast + p->Kg * (p->Zk - p->H * p->X_forecast);
  //�������ֵЭ���� P(k|k) = [I - Kg(k) * H] * P(k|k-1);
  p->P_Optimal = (p->I - p->Kg * p->H) * p->P_forecast;

  //������һ�̵ĺ������ֵ
  p->LX_Optimal = p->X_Optimal;
  //������һ�̵ĺ������ֵЭ����
  p->LP_Optimal = p->P_Optimal;

  //���غ������ֵ
  return p->X_Optimal;
}



//���׿������ĳ�ʼ������
//��Ȼ˵�����кܶ�ֵ�ڼ���ʱ��û���õ���������Ϊ�˱���ԭ����������
void SencondOrder_Kalman_Parameter_Init(SecondOrder_Kalman_Init_t *IF, SecondOrder_Kalman_t *F)
{
	
  // --------------- �� ����float���� ��ת��Ϊ ���� -------------------- 
  mat_init(&F->Xk,2,1,(float*) IF->Xk);
  mat_init(&F->Zk,2,1,(float*) IF->Zk);
  mat_init(&F->H,2,2,(float*) IF->H);

  mat_init(&F->X_forecast,2,1,(float*) IF->X_forecast);
  mat_init(&F->LX_Optimal,2,1,(float*) IF->LX_Optimal);
  mat_init(&F->A,2,2,(float*) IF->A);
  mat_init(&F->P_forecast,2,2,(float*) IF->P_forecast);
  mat_init(&F->LP_Optimal,2,2,(float*) IF->LP_Optimal);
  mat_init(&F->A_T,2,2,(float*) IF->A_T);
  mat_trans(&F->A,&F->A_T);
	mat_init(&F->B,2,1,IF->B);
	mat_init(&F->U,1,1,IF->U);
  mat_init(&F->Q,2,2,(float*) IF->Q);

  mat_init(&F->Kg,2,2,(float*) IF->Kg);
  mat_init(&F->H_T,2,2,(float*) IF->H_T);
  mat_trans(&F->H,&F->H_T);
  mat_init(&F->R,2,2,(float*) IF->R);
  mat_init(&F->X_Optimal,2,1,(float*) IF->X_Optimal);
  mat_init(&F->P_Optimal,2,2,(float*) IF->P_Optimal);
  mat_init(&F->I,2,2,(float*) IF->I);

}

float* SencondOrder_Kalman_Calucate(SecondOrder_Kalman_t *F,float InputData1,float InputData2,float InputData3)
{
  float TempData_21[2] = {0.0f,0.0f};
  float TempData_22[4] = {0.0f,0.0f};
	float TempData_11[1] = {0.0f};
  mat Temp_21,Temp_22,Temp_11;

  mat_init(&Temp_21,2,1,(float*)TempData_21);
  mat_init(&Temp_22,2,2,(float*)TempData_22);
	mat_init(&Temp_11,1,1,(float*)TempData_11);

  F->Xk.pData[0] = InputData1;
  F->Xk.pData[1] = InputData2;
	F->U.pData[0] = InputData3;
	
  
  //�۲�
  //�������� Z(k) = H * X(k) + V(k);
  mat_mult(&F->H,&F->Xk,&F->Zk);
  
	//Ԥ��
  //�������ֵ X(k|k-1) = A * X(k-1|k-1)+B * U;
  mat_mult(&F->A,&F->LX_Optimal,&F->X_forecast);
	mat_mult(&F->B,&F->U,&Temp_11);
	mat_add(&F->X_forecast,&Temp_21,&F->X_forecast);
 
  //�������ֵЭ���� P(k|k-1) = A * P(k-1|k-1) * A' + Q;
  mat_mult(&F->A,&F->LP_Optimal,&F->P_forecast);
  mat_mult(&F->A_T,&F->P_forecast,&Temp_22);
  mat_add(&Temp_22,&F->Q,&F->P_forecast);
 
	//����
  //���������� Kg(k) = (P(k|k-1) * H' / (H * P(k|k-1) * H' + R));
  mat_mult(&F->P_forecast,&F->H_T,&F->Kg);
  mat_mult(&F->Kg,&F->H_T,&Temp_22);
  mat_add(&Temp_22,&F->R,&F->Kg);
  mat_inv(&F->Kg,&F->P_Optimal);
  mat_mult(&F->P_forecast,&F->H_T,&Temp_22);
	mat_mult(&Temp_22,&F->P_Optimal,&F->Kg);
  
	//�������ֵ X(k|k) = X(k|k-1) + Kg(k) * (Zk - H * X(k|k-1));
  mat_mult(&F->H,&F->X_forecast,&Temp_21);
  mat_sub(&F->Zk,&Temp_21,&F->X_Optimal);
  mat_mult(&F->Kg,&F->X_Optimal,&Temp_21);
  mat_add(&F->X_forecast,&Temp_21,&F->X_Optimal);
  
	//�������ֵЭ���� P(k|k) = [I - Kg(k) * H] * P(k|k-1);
  mat_mult(&F->Kg,&F->H,&F->P_Optimal);
  mat_sub(&F->I,&F->P_Optimal,&Temp_22);
  mat_mult(&Temp_22,&F->P_forecast,&F->P_Optimal);
	
  //������һ�̵ĺ������ֵ
  *F->LX_Optimal.pData = *F->X_Optimal.pData;
  //������һ�̵ĺ������ֵЭ����
  *F->LP_Optimal.pData = *F->P_Optimal.pData;
  //���غ������ֵ
  F->Filter_Value[0] = F->X_Optimal.pData[0];
  F->Filter_Value[1] = F->X_Optimal.pData[1];

  return F->Filter_Value;
  
}


