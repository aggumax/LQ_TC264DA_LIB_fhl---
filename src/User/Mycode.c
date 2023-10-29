#include "Mycode.h"
#include "LQ_GPT12_ENC.h"
#include "LQ_CCU6.h"
#include "LQ_MPU6050_DMP.h"
#include "LQ_GPT_mini512.h"

uint8  Start_Flag2=0;                    //启动标志

/*****PID数值定义*****/
#define DUOJI_Kp      0;
#define DUOJI_Ki      0;
#define DUOJI_Kd      0;                //舵机PID参数

float PINHEN_KP = 0, PINHEN_KI = 0, PINHEN_KD = 0;//平衡环PID调节,动量轮，pitch角度环
float D_SPEED_KP=0, D_SPEED_KI=0,D_SPEED_KD=0;    //动量轮电机速度环PID控制

/*****数值定义*****/
int PWM_D, PWM_S;                       //PWM_D是动量轮电机控制左右倾斜，S是动量轮电机速度环正反馈
int encValue_D = 0;                     //储存动量轮的编码器数值
float Pitch_LINGDIAN = 1.5, Pitch_error2 = 0.00;//规定pitch的角度零点 1.5
float Pitch_ERROR = 0.00;               //Pitch偏差值


unsigned char Flag_Stop2 = 0;


void Balance_FHL(void)
{
    LQ_DMP_Read();                                   //陀螺仪数据读取，pitch左正右负
    ENC_InitConfig(ENC6_InPut_P20_3, ENC6_Dir_P20_0);//读取编码器数据
    encValue_D = ENC_GetCounter(ENC6_InPut_P20_3);    //动量轮的数值

    /*动量轮控制*/
    PWM_D = Balance_X(Pitch,Pitch_ERROR,gyro[0]);
    PWM_S = SPEED_Control(encValue_D);
}


/*******************
平衡PID控制函数，角度环
Angle:  Pitch的角度
Angle_Zero:  Pitch的偏差值
Gyro;  Gyro[]
 ******************/
float Balance_X(float Angle,float Angle_Zero,float Gyro)
{
    float PWM, Bias;
    static float error;
    Bias=Angle-Angle_Zero;      //获取偏差
    error+=Bias;                //偏差累积
    if(error>+30) error=+30;
    if(error>-30) error=-30;    //积分限幅
    PWM = PINHEN_KP*Bias + PINHEN_KI*error + PINHEN_KD*Gyro;//获取最终数值

    return PWM;
}


/*******************
动量轮速度PI控制,速度正反馈环
encValueD;  编码器的数值(速度环)
 ******************/
float SPEED_Control(int encValueD)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = encValueD;                                                  //速度滤波
    Encoder *= 0.7;                                                           //一阶低通滤波器
    Encoder += Encoder_Least*0.3;                                             //一阶低通滤波器
    Encoder_Integral += Encoder;                                              //积分出位移
    if(Encoder_Integral > +2000) Encoder_Integral = +2000;                    //积分限幅
    if(Encoder_Integral < -2000) Encoder_Integral = -2000;                    //积分限幅
    Velocity = Encoder * D_SPEED_KP + Encoder_Integral * D_SPEED_KI/100;      //获取最终数值
    if(Flag_Stop2==1) Encoder_Integral=0,Encoder=0,Velocity=0;                 //停止时参数清零
    return Velocity;
}


