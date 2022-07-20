#include "Snail.h"

//��Snail�Ŀ����ź��г�Ϊ 400-2200��s
//�����ǵ�PWM�źŵ�Ƶ��Ϊ500Hz��������Ϊ0-2000��s�����Ӧ��ccrֵΪ0-1999

//Ҫ��Snail������ֵ�������Ǹ���������ռ�ձȣ�ֻ�и�����ռ�ձȡ�Snail��������
//Ҫ���ȷ���������ֵ�أ�
//�ϵ��ᷢ��B_B_B����ʾ��PWM�ź�����
//֮��������BBBBBB��������˵��������ռ�ձȹ���Ҫ��С
//֮����û������������˵�������ռ�ձȹ�С��Ҫ����
//�������κ󣬱��õ���Snail������ռ�ձ�
//����ֵ
//uint16_t Accelerator_Value = 1000;			

//���� �г� ��ֵ
//uint16_t Strock_Value = 660;					//�г̵����ֵһ��Ϊ660	

Snail_t Snail;

//uint16_t a = 0;							//���ṹ���޷�������֮ǰ��������г�ֵ������ȫ�ֱ���

//Snail����ĳ�ʼ��
void Snail_Init(void)
{
	//����ֵ
	Snail.Accelerator_Value = 1000;
	//�г�ֵ
	Snail.Strock_Value = 660;
	
	//ʹ��TIM5��PWMģʽ
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	
	
	/*�������ģʽ��
	
	//�����г�:
	
	//ע��ע�⣺
	//�����г� �� ת��ʱ ����ʼ����ccr=0 ���������ź�
	//������г�ֵ������Ҫ��ֵ ����1660
	//����С������������ֵ ��1000 ������0
	
	//����Snail���PWM�������⣺
	//�� PWM �źŵ��������õ�����г̣�����������Ӳ��ϵ硣��ʱ�������BB �� BBB
	//��������������ʱ��Ϊ 2 �롣�ڼ��ʱ���ڿ��԰������·����������ã�
	//  a.   PWM�г�У׼
	//�� BB ����� 2 ���ڽ�PWM�źŵ��������õ���С��ֱ���������Լ 1 ��� B ���� ��
	//PWM�г�У׼��ɡ�
	//  b.   ���ת���л�
	//�� BBB ����� 2 ���ڽ�PWM�źŵ��������õ���С��ֱ���������Լ 1 ��� B ���� ��
	//ʾ���ת���Ѹ��ġ�
	
	//Snail�ϵ��Ҫ��ռ�ձ�Ϊ0���ź�һ��ʱ�䣬Ȼ���ٸ���ռ�ձ�
	HAL_Delay(10000);
	
	//����ռ�ձȵĺ���
	//__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,Snail.Accelerator_Value + Snail.Strock_Value);
	//__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,Snail.Accelerator_Value + Snail.Strock_Value);
	//__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,Snail.Accelerator_Value + Snail.Strock_Value);
	//__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,Snail.Accelerator_Value + Snail.Strock_Value);
	
	//���ڵ����н�
	//TIM��CCR��ֵ��Ϊ Strock_Value = 0;
	*/
	
}


