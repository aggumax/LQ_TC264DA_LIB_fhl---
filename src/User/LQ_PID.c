/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��chiusir
��E-mail  ��chiusir@163.com
������汾��V1.0
�������¡�2020��4��10��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��Hightec4.9.3/Tasking6.3�����ϰ汾
��Target �� TC264DA
��Crystal�� 20.000Mhz
��SYS PLL�� 200MHz
����iLLD_1_0_1_11_0�ײ����
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
 *  �������ƣ�float constrain_float(float amt, float low, float high)
 *  ����˵�����޷�����
 *  ����˵����
  * @param    amt   �� ����
  * @param    low   �� ���ֵ
  * @param    high  �� ���ֵ
 *  �������أ���
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
float constrain_float(float amt, float low, float high)
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// pid������ʼ������
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
	pid->integrator= 0;   //����
	pid->last_error= 0;
	pid->last_derivative   = 0;
	pid->last_t    = 0;
	pid->qiwan_speed = 1000;
	pid->shiji_speed = 0;
}

/*************************************************************************
 *  �������ƣ�float constrain_float(float amt, float low, float high)
 *  ����˵����pidλ��ʽ���������
 *  ����˵����
  * @param    pid     pid����
  * @param    error   pid�������
 *  �������أ�PID������
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
float PidLocCtrl(pid_param_t * pid, float error)
{
	/* �ۻ���� */
	pid->integrator += error;

	/* ����޷� */
	constrain_float(pid->integrator, -pid->imax, pid->imax);


	pid->out_p = pid->kp * error;
	pid->out_i = pid->ki * pid->integrator;
	pid->out_d = pid->kd * (error - pid->last_error);

	pid->last_error = error;

	pid->out = pid->out_p + pid->out_i + pid->out_d;

	return pid->out;
}
/*************************************************************************
 *  �������ƣ�float constrain_float(float amt, float low, float high)
 *  ����˵����pid����ʽ���������
 *  ����˵����
  * @param    pid     pid����
  * @param    error   pid�������
 *  �������أ�PID������   ע���������Ѿ��������ϴν��
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
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

    //�������
//    error = pid->qiwan_speed - pid->shiji_speed;
    //���������
    pid->integrator += error;
    //���ƻ���
    if(pid->integrator > 5500)
    pid->integrator = 5500;
    if(pid->integrator < 500)
    pid->integrator = 500;
    //����΢����
    pid->last_derivative = error - last_error;
    //����PID�����
    pid->out = pid->kp* error + pid->ki* pid->integrator + pid->kd* pid->last_derivative;
    //PID�������
    if(pid->out > 6500)
        pid->out = 6500;
    if(pid->out < 2000)
        pid->out = 2000;
    //������������´����
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
        ENC_InitConfig(ENC6_InPut_P20_3, ENC6_Dir_P20_0);//��ȡ����������
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
        TFTSPI_P8X16Str(0, 6, tbt,u16WHITE,u16BLACK);       //�ַ�����ʾ

        ATOM_PWM_SetDuty(ATOMPWM0, duty, 12500);//�����ĸ����
        ATOM_PWM_SetDuty(ATOMPWM1, 5000, 12500);
//        ATOM_PWM_SetDuty(ATOMPWM2, duty, 12500);
//        ATOM_PWM_SetDuty(ATOMPWM3, 5000, 12500);

        sprintf(txt, "PWM: %05d;", duty);
        TFTSPI_P8X16Str(0, 3, txt,u16WHITE,u16BLACK);       //�ַ�����ʾ

        sprintf(tbt, "Enc5: %05d;", encValue5);
        TFTSPI_P8X16Str(0, 5, tbt,u16WHITE,u16BLACK);       //�ַ�����ʾ
        LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
//        delayms(100);

    }

}



