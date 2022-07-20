#include "PID_Position.h"

P_PID_FUN_t P_PID_FUN = P_PID_HOOK_FUN;

void P_PID_Parameter_Init(P_PID_t *P_PID, float Kp, float Ki, float Kd,
                          float epsilon, float max_error, float min_error,
                          float alpha,
                          float Max_antiwindup, float Min_antiwindup,
                          float Max_result, float Min_result)
{
    P_PID->Kp = Kp;
    P_PID->Ki = Ki;
    P_PID->Kd = Kd;

    P_PID->Target = 0;
    P_PID->Measure = 0;
    P_PID->Error = P_PID->Target - P_PID->Measure;

    P_PID->Epsilon = epsilon;
    P_PID->max_error = max_error;
    P_PID->min_error = min_error;

    P_PID->Proportion = 0;
    P_PID->Integral = 0;
    P_PID->Differential = 0;

    P_PID->alpha = alpha;
    P_PID->D_Output = 0;

    P_PID->Max_antiwindup = Max_antiwindup;
    P_PID->Min_antiwindup = Min_antiwindup;

    P_PID->result = 0;
    P_PID->Max_result = Max_result;
    P_PID->Min_result = Min_result;

    P_PID->LastError = P_PID->Error;
    P_PID->PreError = P_PID->LastError;
    P_PID->D_Last_Output = 0;
}


//积分分离系数β的确定
static uint16_t P_PID_BetaGeneration(float error, float epsilon)
{
    uint16_t beta = 0;
    if (abs(error) <= epsilon)
    {
        beta = 1;
    }
    return beta;
}

/*
//变积分系数
static float P_PID_Variable_Integral_Coefficient(float error, float max_error, float min_error)
{
    float factor = 0.0f;

    if (abs(error) <= min_error)
    {
        factor = 1.0f;
    }
    else if (abs(error) > max_error)
    {
        factor = 0.0f;
    }
    else
    {
        factor = 0.5f; //可以是函数关系式
    }

    return factor;
}
*/
float P_PID_Regulation(P_PID_t *P_PID, float target, float measure)
{
    uint16_t beta; //积分分离系数
    //float factor; //变积分参数

    P_PID->Target = target;   //目标值
    P_PID->Measure = measure; //测量值

    P_PID->Error = target - measure; //偏差值 = 目标值 - 测量值

    P_PID->Proportion = P_PID->Error;                      //比例值
    P_PID->Differential = P_PID->Error - P_PID->LastError; //微分值

    //不完全微分（一阶低通滤波）
    //其中α的取值是一个0~1之间的数。两个极限值，在0时其实就是没有滤波的普通微分环节；而取1时，则没有微分作用
    //不完全的微分它使得在偏差作阶跃式变化时出现的输出瞬时跳变得到一定程度的缓和
    //微分信号的引入可以改善系统的动态特性，但也易引入高频干扰，在误差扰动突变的时候尤其显出微分项的不足
    //标准微分会产生短时间跳变，实际系统不能完全响应，效果不好，会震荡
    P_PID->D_Output = P_PID->Kd * (1 - P_PID->alpha) * P_PID->Differential + P_PID->alpha * P_PID->D_Last_Output;

    /*	
    //抗积分饱和
    //没有抗积分饱和的话，积分还是一直叠加，反调时，就会导致需要较长时间去退出积分饱和区，造成滞后
    if (P_PID->result > P_PID->Max_result)
    {
        if (P_PID->Error <= 0)
        {
            P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2;
        }
    }
    else if (P_PID->result < P_PID->Min_result)
    {
        if (P_PID->Error >= 0)
        {
            P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2;
        }
    }
    else
    {
        //梯形积分
        P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2; //积分值
    }
	*/

    //（本人认为）无论是积分分离还是变积分，其都要是偏差过大时就不对积分项进行叠加，
    //若是积分项仍然继续叠加，而只是对输出进行是否加入I控制的话，
    //下一次需要加入I控制时，其积分项也是累积了过大的偏差
    //但是这样会造成积分不连续，目前不知道会有什么问题，利与弊不清楚
    //积分分离系数的确定
    //积分分离是：偏差较大时取消积分；偏差较小时引入积分
    beta = P_PID_BetaGeneration(P_PID->Error, P_PID->Epsilon);
    //变积分参数的确定
    //变积分是：改变积分的累加速度，偏差较小时，强化积分；偏差较大时，削弱积分
    //factor = P_PID_Variable_Integral_Coefficient(P_PID->Error, P_PID->max_error, P_PID->min_error);

    if (beta == 0) //执行PD控制
    {
        P_PID->result = P_PID->Kp * P_PID->Proportion + P_PID->D_Output; //PD计算结果
    }
    else//执行PID控制
    {

        //抗积分饱和
        //抗积分饱和的阈值不可过大也不可过小，
        //过大会导致处于超调区域的面积很大，此面积也就是积分饱和区
        //而要退出此积分饱和区的就跟你的时间和积分系数有关
        //若你想得到更快的收敛，而积分系数又不宜过大时，就要将抗积分饱和的阈值调小
        //过小则积分起不到很大作用
        //没有抗积分饱和的话，积分还是一直叠加，反调时，就会导致需要较长时间去退出积分饱和区，造成滞后
        if (P_PID->Integral > P_PID->Max_antiwindup)
        {
            if (P_PID->Error <= 0)
            {
                P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2;
            }
        }
        else if (P_PID->Integral < P_PID->Min_antiwindup)
        {
            if (P_PID->Error >= 0)
            {
                P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2;
            }
        }
        else
        {
            //梯形积分
            //从微积分的角度来说，当微分分到无限小时，矩形积分于梯形积分是没有区别的
            //但实际上采样周期不可能无限小，而且不可能是连续的，
            //那么采样周期越大，那么矩形近似于实际曲线的偏差就会越大
            //而梯形积分就更接近实际曲线
            P_PID->Integral += (P_PID->Error + P_PID->LastError) / 2; //积分值
        }

        P_PID->result = P_PID->Kp * P_PID->Proportion + P_PID->Ki * P_PID->Integral + P_PID->D_Output; //PID计算结果
    }

    // 输出限幅:
    // 单纯的抗积分饱和并未能很好的解决输出超过实际最大值，而导致反调时需要较长时间去退出输出饱和区，而导致反调滞后的问题
    // 这是因为虽然积分饱和而不再叠加输出，但是偏差是存在的，PD控制仍然会促进输出。
    if (P_PID->result > P_PID->Max_result)
    {
        P_PID->result = P_PID->Max_result;
    }
    else if (P_PID->result < P_PID->Min_result)
    {
        P_PID->result = P_PID->Min_result;
    }

    P_PID->PreError = P_PID->LastError; //前两拍偏差
    P_PID->LastError = P_PID->Error;    //前一拍偏差

    P_PID->D_Last_Output = P_PID->D_Output; //将这次的微分输出作为上次的微分输出

    return P_PID->result;
}

//切换pid时的清除
void P_PID_Parameter_Clear(P_PID_t *P_PID)
{
	P_PID->LastError = 0;				//清除上次的偏差值
	P_PID->PreError = 0;				//清除上上次的偏差值
	P_PID->Integral = 0;				//清除积分累计
	P_PID->D_Last_Output = 0;		//清除上次的微分输出
	P_PID->result = 0;					//清除最终输出
}

