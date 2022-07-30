#include "BREAK.h"
#include "main.h"

BREAK_e break_basic;

long M2006_targe_angle = 0;
int M2006_targe_speed = 0;
int send_to_break = 0;

int M2006_init_times = 0;            //��ʼ�� ��ʱ
int M2006_init_change_angle = 0;     //��ʼ�� �Ƕȱ仯��
int M2006_init_totalAngle_last = 0;  //��ʼ�� ��һʱ�̽Ƕ�ֵ

int stop_CH_OP_BC_BREAK_times=0;
bool start_use_break=0;//ʹ��ɲ��,���Ǳ�־λ
bool stop_CH_OP_BC_BREAK=0;

/**/
P_PID_t BREAK_ANGLE_pid;//ɲ��PID
P_PID_t BREAK_SPEED_pid;

uint8_t Break_static = 0;

void break_init(void) {
	
	if (DR16.Switch_Left  == 3 &&
		DR16.Switch_Right == 2) 
{
		if(DR16.ch4_DW>300)//���µ�һ��
		{
	start_use_break=1;	
		}
		else
		{
	start_use_break=0;	
		}
}
	
	
  M2006_init_times++;

  if (M2006_init_times % 50 == 0)  //ÿ100ms���һ��
  {
    M2006_init_change_angle =
        M3508s[BREAK_ID].totalAngle - M2006_init_totalAngle_last;
    M2006_init_totalAngle_last = M3508s[BREAK_ID].totalAngle;
  }

  
  
  if (DR16.Switch_Left == 1)  //���ϵ�/�Զ��� ��һ��Ҫ�ȳ�ʼ��ɲ��
  {
//    if (break_is_raedy == 0)  //ɲ��û��׼����
//      if (send_to_chassis == 0 && abs(M3508s[3].realSpeed) < 10)  //�����Ѿ�ʧ��
//      {
//      }
  } 
  
  
  else if (DR16.Switch_Left == 3 &&
             DR16.Switch_Right == 2)  //�ֶ� ��ʼ��ɲ��,��ʱ Ħ�����ٶ�Ϊ0
  {
	  /* �ֶ�������ʼ�� (�����ж�ң����)
    if (DR16.rc.ch4_DW >= 200)  //����
    {
      if (break_basic.STATE == 0)  //δ��ʼ��
      {
        break_basic.STATE = 1;
        M2006_init_times = 0;  //��ʼ��ʼ������0
      }
    }
	  */
	        if (break_basic.STATE == 0)  //δ��ʼ��
      {
		  if(start_use_break==1)
		  {
        break_basic.STATE = 1;
        M2006_init_times = 0;  //��ʼ��ʼ������0
			  
		  }
      }
    else if (break_basic.STATE == 1)  //ɲ�����ֵû��׼����.���ڳ�ʼ�����ֵ
    {
      BREAK_SPEED_pid.Max_result = 5000;
      BREAK_SPEED_pid.Min_result = -5000;
      M2006_targe_angle = M3508s[BREAK_ID].totalAngle + 1999;
      if (M2006_init_times > 1000)  //��200*3=600ms ������
      {
        if (abs(M2006_init_change_angle) < 200)  //�Ƕȱ仯С��200
        {
          //				if(abs(M3508s[BREAK_ID].realCurrent)>1500)//�������ֵ�ж�
          break_basic.BREAK_MAX = M3508s[BREAK_ID].totalAngle;  //���ֵ�ҵ���
          break_basic.STATE = 2;  //ȥ��ʼ����Сֵ
          M2006_init_times = 0;
        }
      }
    }
	else if (break_basic.STATE == 2)  //���ڳ�ʼ����Сֵ
    {
      BREAK_SPEED_pid.Max_result = 5000;
      BREAK_SPEED_pid.Min_result = -5000;
      M2006_targe_angle = M3508s[BREAK_ID].totalAngle - 1999;
      if (M2006_init_times > 1000)  //��200*3=600ms ������
      {
        if (abs(M2006_init_change_angle) < 200)  //�Ƕȱ仯С��200
        {
          break_basic.BREAK_MIN = M3508s[BREAK_ID].totalAngle;  //��Сֵ�ҵ���
          break_basic.BREAK_MID =(break_basic.BREAK_MIN + break_basic.BREAK_MAX) /2;  //�м�ֵ�ҵ���
			M2006_targe_angle=break_basic.BREAK_MID;
			BREAK_SPEED_pid.Max_result = 9000;
      BREAK_SPEED_pid.Min_result = -9000;
          break_basic.STATE = 3;  //��ʼ��ȫ�����
        }
      }
    }
  }

  if (DR16.Switch_Left == 2)  //ʧ��
  {
    M2006_init_times = 0;  //��ʼ����ʱ����
    if (break_basic.STATE == 2 ||
        break_basic.STATE == 1 || break_basic.STATE == 3)  //��ʼ����һ���ʧ����,ֱ�Ӵ�ͷ��ʼ��ʼ��
    {
      break_basic.STATE = 0;
    }
  }
}


void break_control(void) 
{
	if(Robots_Control.Chassis_e == cs_Disable)
	{
		//ʧ��
		send_to_break = 0;
		//��ձ���ɲ��״̬
		Chassis.Random.Break_flag = 0;
		//ɲ��״̬��0
		Break_static = 0;
		//ɲ��Ŀ��ֵ��Ϊ��ֵ
		M2006_targe_angle = break_basic.BREAK_MID;	
	}
	//ɲ����ʼ���ɹ���
	if(break_basic.STATE == 3)
	{
		if(Robots_Control.Chassis_e == A_cs_Cruise || Robots_Control.Chassis_e == A_cs_Random)
		{
			if(Chassis.CDisable.Left_flag == 0 && Chassis.CDisable.Right_flag == 0)
			{
				//ɲ���ĵ�һ�׶Σ���Ҫ����	Break_static��0��ɲ��δ��ʼ
				if(Chassis.Random.Break_flag == 1 && Break_static == 0)
				{
					Break_static = 1;											//ɲ����һ�׶�
					if(Chassis.EM->realSpeed > 0)					//��ɲ��
					{
						//ȷ��ɲ�����2006��Ŀ��Ƕ�
						M2006_targe_angle = break_basic.BREAK_MAX + 3000;		//�ȴ����
						Break_static = 21;										//������ɲ�׶�
					}
					else if(Chassis.EM->realSpeed < 0)		//��ɲ��
					{
						//ȷ��ɲ�����2006��Ŀ��Ƕ�
						M2006_targe_angle =  break_basic.BREAK_MIN -3000;			//��С��С 
						Break_static = 22;										//������ɱ�׶�
					}
				}
				
				if(Break_static == 21)
				{
					if(M3508s[BREAK_ID].totalAngle > break_basic.BREAK_MAX + 3000)			//�����ֵ��һ���
					{
						send_to_break = 0;
					}
					if(Chassis.EM->realSpeed < 100)
					{
						Break_static = 31;
					}
				}
				else if(Break_static == 22)
				{
					if(M3508s[BREAK_ID].totalAngle < break_basic.BREAK_MIN - 3000)			//����Сֵ��Сһ���
					{
						send_to_break = 0;
					}
					if(Chassis.EM->realSpeed > -100)
					{
						Break_static = 32;
					}
				}	
				
				//���ڱ���ͣ����ʱ��̫���ˣ��۲�һ�¹��ʿ�һ���Ƿ�Ҫ�ٶȵ��ڣ�ͬ��ĳ����Сֵ�ͷ���ʹ�� 
				if(Break_static == 31)							//��2�׶ε�1��
				{
					if(Chassis.EM->realSpeed < -30)		//��0����ĳ����ֵ
					{
						//̧��ɲ��
						M2006_targe_angle = break_basic.BREAK_MID - 1500;
						Break_static = 4;									
					}
				}
				else if(Break_static == 32)					//�ڶ��׶ε�2��
				{
					if(Chassis.EM->realSpeed > 30)		//��0����ĳ����ֵ
					{
						//̧��ɲ��
						M2006_targe_angle = break_basic.BREAK_MID + 1500;
						Break_static = 4;					
					}
				}
				
				if(Break_static == 4)						//�����׶�ɲ�����
				{
					//���������Ҫɲ����־λ
					Chassis.Random.Break_flag = 0;
					//���ɲ��״̬��־λ
					Break_static = 0;
					//�����ڱ�������л�һֱ����
					Chassis.Random.Dir_times = 0;
				}
				

				if(Break_static == 1 || Break_static == 21 || Break_static == 22)
				{
					Chassis.EM->OutputCurrent = 0;			//���̵��ʧ��
				}
				
			}
			else if(Chassis.CDisable.Left_flag == 1 || Chassis.CDisable.Right_flag == 1)
			{
				//������Ҫɲ������0
				Chassis.Random.Break_flag = 0;
				//ɲ��״̬��0
				Break_static = 0;
				//ɲ����Ŀ��ֵ����
				if(Chassis.CDisable.Left_flag == 1)
				{
					M2006_targe_angle = break_basic.BREAK_MID + 1500;
				}
				else if(Chassis.CDisable.Right_flag == 1)
				{
					M2006_targe_angle = break_basic.BREAK_MID - 1500;
				}
				//�����ڱ�������л�һֱ����
				Chassis.Random.Dir_times = 0;
			}
		}
		else if(Robots_Control.Chassis_e == R_cs_Common)		//�������ֶ�״̬
		{
			if(DR16.ch4_DW < -600)				//�������ϲ�
			{
				M2006_targe_angle = break_basic.BREAK_MAX +3000; 		//����ɲ��			
			}
			else if(DR16.ch4_DW > 600)
			{
				M2006_targe_angle = break_basic.BREAK_MIN -3000; 			//����ɲ��
			}
			else				//���������
			{
				M2006_targe_angle = break_basic.BREAK_MID;					//ɲ��̧��
			}
		}
		
	}
	else
	{
		Break_static = 0;
		Chassis.Random.Break_flag = 0;
	}
}



