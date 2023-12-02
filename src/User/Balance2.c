#include "Balance2.h"

#include <IfxCpu.h>
#include <IfxScuCcu.h>
#include <IfxScuWdt.h>
#include <IfxStm.h>
#include <IfxStm_reg.h>
#include <stdio.h>

#include "LQ_ADC_test.h"
#include "LQ_Atom_Motor.h"
#include "LQ_CAMERA.h"
#include "LQ_EEPROM_TEST.h"
#include "LQ_FFT_TEST.h"
#include "LQ_GPIO_ExInt.h"
#include "LQ_GPIO_KEY.h"
#include "LQ_GPIO_LED.h"
#include "LQ_GPT_mini512.h"
#include "LQ_I2C_9AX.h"
#include "LQ_I2C_VL53.h"
#include "LQ_ICM20602.h"
#include "LQ_OLED096.h"
#include "LQ_STM_Timer.h"
#include "LQ_TFT18.h"
#include "LQ_Tim_InputCature.h"
#include "LQ_Tom_Servo.h"
#include "LQ_UART_Bluetooth.h"
#include "include.h"
#include "LQ_STM.h"
#include "LQ_UART.h"
#include "LQ_MotorServo.h"
#include <LQ_IIC_Gyro.h>
#include "LQ_TFT2.h"
#include "LQ_BLDC.h"
#include "LQ_Balance.h"
#include "LQ_CCU6.h"
#include "LQ_ADC.h"
#include "IfxGtm_PinMap.h"
#include "LQ_MPU6050_DMP.h"
#include "LQ_SOFTI2C.h"
#include "LQ_CAMERA.h"
#include "BD_1202V2.h"
#include "LQ_GTM.h"
#include "LQ_GPT12_ENC.h"

#include "Mycode.h"
#include "image_8.h"

/*数值定义*/
int enc_h=0;
short duty_FHL = 0;//舵机增减pwm值
short duty_FHL2= 0;

int enc_DonLian=0;

/****************************
 @ PID初始化
   包括了所有用到的参数
*****************************/
void PID_init(FHL_PID_DJ *pid)
{
    pid->kp1 = 40;
    pid->ki1 = 0;
    pid->kd1 = 10;
    pid->jifen1 =0;
    pid->weifen1=0;
    pid->out1=0;

    pid->kp2 = 0;
    pid->ki2 = 0;
    pid->kd2 = 0;
    pid->jifen2 =0;
    pid->weifen2=0;
    pid->out2=0;

    pid->kps = 0;
    pid->kis = 0;
    pid->kds = 0;
    pid->jifens =0;
    pid->weifens=0;
    pid->outs=0;

    pid->kpjd = 0;
    pid->kijd = 0;
    pid->kdjd = 0;
    pid->jifenjd =0;
    pid->weifenjd=0;
    pid->outjd=0;

}

/*********************************************************
 @ 舵机的转向PID函数
 @ 输出：舵机的转向变化值
**********************************************************/
float FHL_PID1_DJ(FHL_PID_DJ *pid,float Pitch,float qiwan)
{
    float error;
    float Last_error=0;
    error = qiwan - Pitch;

    pid->jifen1 +=error;
    if(pid->jifen1>20) pid->jifen1=20;
    if(pid->jifen1<-20) pid->jifen1=-20;
    pid->weifen1 = error -Last_error;

    pid->out1 = pid->kp1*error + pid->ki1*pid->jifen1 + pid->weifen1*pid->kd1;
    if(pid->out1 > 330)pid->out1=330;
    if(pid->out1 < -330)pid->out1=-330;
    error = Last_error;

    return pid->out1;
}

float FHL_PID2_DJ(FHL_PID_DJ *pid,float shiji,float qiwan)
{
    float error;
    error = qiwan - shiji;

    pid->jifen2 +=error;
    if(pid->jifen2 > 20) pid->jifen2=20;
    if(pid->jifen2 < -20) pid->jifen2=-20;
    pid->weifen2=error;

    pid->out2 = pid->kp2*error + pid->ki2*pid->jifen2 + pid->kd2*pid->weifen2;
    return pid->out2;
}

void FHL_servo(void)
{
    short duty_FHLk=1950;
    short duty_FHLn=4800;
    ENC_InitConfig(ENC2_InPut_P33_7, ENC2_Dir_P33_6);
    enc_h=ENC_GetCounter(ENC2_InPut_P33_7);
    LQ_DMP_Read();

    FHL_PID_DJ PID;
    PID_init(&PID);

    if(Pitch > 23 || Pitch< -23){
        duty_FHLk = 1950;
        duty_FHLn = 0;
    }

    duty_FHLk -= FHL_PID1_DJ(&PID, Pitch, 2.0);
    ATOM_PWM_SetDuty(ATOMSERVO1, duty_FHLk, 100);

//    duty_FHLn += FHL_PID2_DJ(&PID, enc_h, 500);
    ATOM_PWM_SetDuty(ATOMPWM0, duty_FHLn, 12500);

}

/********************************************
@平衡函数的速度环
@enc_Donlian：动量轮的数值
@qiwan:期望值
*********************************************/
int16 Speed(FHL_PID_DJ *pid, int enc_Donlian, int qiwan)
{
    float error;
    error = qiwan - enc_Donlian;
    float last_error;

    pid->jifens +=error;
    if(pid->jifens > 20) pid->jifens=20;
    if(pid->jifens < -20) pid->jifens=-20;
    pid->weifens = last_error;

    pid->outs = error*pid->kps + pid->jifens*pid->kis + pid->weifens*pid->kds;
    last_error = error;

    return pid->outs;
}

/********************************************
@平衡函数的角度环
@enc_Donlian：角度Pitch的数值
@qiwan:期望值
*********************************************/
int16 Jiaodu(FHL_PID_DJ *pid, int Pitch, int qiwan)
{
    float error;
    error = qiwan - Pitch;
    float last_error;

    pid->jifenjd +=error;
    if(pid->jifenjd > 20) pid->jifenjd=20;
    if(pid->jifenjd < -20) pid->jifenjd=-20;
    pid->weifenjd = last_error;

    pid->outjd = error*pid->kpjd + pid->jifenjd*pid->kijd + pid->weifenjd*pid->kdjd;
    last_error = error;

    return pid->outjd;
}





























