/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】chiusir
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2020年4月10日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】Hightec4.9.3/Tasking6.3及以上版本
【Target 】 TC264DA
【Crystal】 20.000Mhz
【SYS PLL】 200MHz
基于iLLD_1_0_1_11_0底层程序
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "stdio.h"
#include "LQ_PID.h"
#include "LQ_MotorServo.h"
#include "LQ_CCU6.h"
#include "LQ_GPT12_ENC.h"
#include "LQ_TFT18.h"
#include "LQ_Atom_Motor.h"
#include "LQ_GTM.h"
#include "LQ_GPIO_LED.h"
#include "LQ_STM.h"
#include "LQ_GPIO_KEY.h"
#include "string.h"
#include "math.h"

unsigned short Duty;

/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：限幅函数
 *  参数说明：
  * @param    amt   ： 参数
  * @param    low   ： 最低值
  * @param    high  ： 最高值
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float constrain_float(float amt, float low, float high)
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// pid参数初始化函数
void PidInit(pid_param_t * pid)
{
	pid->kp        = 10;
	pid->ki        = 1;
	pid->kd        = 1;
	pid->imax      = 0;
	pid->out_p     = 0;
	pid->out_i     = 0;
	pid->out_d     = 0;
	pid->out       = 0;
	pid->integrator= 0;   //积分
	pid->last_error= 0;
	pid->last_derivative   = 0;
	pid->last_t    = 0;
	pid->qiwan_speed = 1000;
	pid->shiji_speed = 0;
}

/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：pid位置式控制器输出
 *  参数说明：
  * @param    pid     pid参数
  * @param    error   pid输入误差
 *  函数返回：PID输出结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float PidLocCtrl(pid_param_t * pid, float error)
{
	/* 累积误差 */
	pid->integrator += error;

	/* 误差限幅 */
	constrain_float(pid->integrator, -pid->imax, pid->imax);


	pid->out_p = pid->kp * error;
	pid->out_i = pid->ki * pid->integrator;
	pid->out_d = pid->kd * (error - pid->last_error);

	pid->last_error = error;

	pid->out = pid->out_p + pid->out_i + pid->out_d;

	return pid->out;
}
/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：pid增量式控制器输出
 *  参数说明：
  * @param    pid     pid参数
  * @param    error   pid输入误差
 *  函数返回：PID输出结果   注意输出结果已经包涵了上次结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float PidIncCtrl(pid_param_t * pid, float error)
{

	pid->out_p = pid->kp * (error - pid->last_error);
	pid->out_i = pid->ki * error;
	pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);

	pid->last_derivative = error - pid->last_error;
	pid->last_error = error;

	pid->out += pid->out_p + pid->out_i + pid->out_d;

	return pid->out;
}

float FHL_PID(pid_param_t * pid, float error)
{
    float last_error=0;

    //计算误差
//    error = pid->qiwan_speed - pid->shiji_speed;
    //计算积分项
    pid->integrator += error;
    //限制积分
    if(pid->integrator > 5500)
    pid->integrator = 5500;
    if(pid->integrator < 500)
    pid->integrator = 500;
    //计算微分项
    pid->last_derivative = error - last_error;
    //计算PID的输出
    pid->out = pid->kp* error + pid->ki* pid->integrator + pid->kd* pid->last_derivative;
    //PID输出限制
    if(pid->out > 6500)
        pid->out = 6500;
    if(pid->out < 2000)
        pid->out = 2000;
    //传入这次误差给下次误差
    last_error = error;

    return pid->out;
}



void Text_PID(pid_param_t * pid)
{
    unsigned short duty=5000;
    char txt[16];
    char tbt[32];
    int encValue5 = 0, error;

    TFTSPI_Init(1);
    TFTSPI_CLS(u16BLUE);
    TFTSPI_P8X16Str(2, 0, "Gumax_PID",u16WHITE,u16BLACK);
    GPIO_KEY_Init();

    ATOM_PWM_InitConfig(ATOMPWM0, 5000, 12500);
    ATOM_PWM_InitConfig(ATOMPWM1, 5000, 12500);

    pid_param_t PID;
    PidInit(&PID);

    while(1)
    {
        ENC_InitConfig(ENC6_InPut_P20_3, ENC6_Dir_P20_0);//读取编码器数据
        encValue5 = ENC_GetCounter(ENC6_InPut_P20_3);

        pid->qiwan_speed = 1000;
        pid->shiji_speed = encValue5;
        error = pid->qiwan_speed - pid->shiji_speed;

        duty = FHL_PID(&PID,error);
        if(duty > 8000)
            duty = 8000;
        if(duty < 500)
            duty = 500;

        sprintf(tbt, "error: %05d;", error);
        TFTSPI_P8X16Str(0, 6, tbt,u16WHITE,u16BLACK);       //字符串显示

        ATOM_PWM_SetDuty(ATOMPWM0, duty, 12500);//驱动四个电机
        ATOM_PWM_SetDuty(ATOMPWM1, 5000, 12500);
//        ATOM_PWM_SetDuty(ATOMPWM2, duty, 12500);
//        ATOM_PWM_SetDuty(ATOMPWM3, 5000, 12500);

        sprintf(txt, "PWM: %05d;", duty);
        TFTSPI_P8X16Str(0, 3, txt,u16WHITE,u16BLACK);       //字符串显示

        sprintf(tbt, "Enc5: %05d;", encValue5);
        TFTSPI_P8X16Str(0, 5, tbt,u16WHITE,u16BLACK);       //字符串显示
        LED_Ctrl(LED0,RVS);        //电平翻转,LED闪烁
//        delayms(100);

    }

}



