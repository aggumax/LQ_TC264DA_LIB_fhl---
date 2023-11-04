#include "Mycode.h"
#include "stdio.h"

#include "LQ_Balance.h"
#include "LQ_GPT12_ENC.h"
#include "LQ_CCU6.h"
#include "LQ_MPU6050_DMP.h"
#include "LQ_GPT_mini512.h"
#include "LQ_Atom_Motor.h"
#include "LQ_TFT18.h"
#include "LQ_PID.h"
#include "LQ_GTM.h"
#include "LQ_MotorServo.h"
#include "LQ_Atom_Motor.h"

/*****PID数值定义*****/
#define DUOJI_Kp      0;    //舵机PID参数
#define DUOJI_Ki      0;
#define DUOJI_Kd      0;
/*平衡环PID调节,动量轮，pitch角度环*/
float PINHEN_KP = 0;       
float PINHEN_KI = 0;
float PINHEN_KD = 0;
/*动量轮电机速度环PID控制*/
float D_SPEED_KP=0;         
float D_SPEED_KI=0;
/*串级PID的角速度环*/
float  JSD_kp=3000;               
float  JSD_ki=0;
float  JSD_kd=60;
float  JSD_pwm_out =0;
float  JSD_jifen   =0;
float  JSD_weifen  =0;


/*****数值定义*****/
int PWM_D, PWM_S;                       //PWM_D是动量轮电机控制左右倾斜，S是动量轮电机速度环正反馈
int encValue_D = 0;                     //储存动量轮的编码器数值
int encValue_H = 0;                     //储存电机的编码器数值
float Pitch_LINGDIAN = 1.5, Pitch_error2 = 0.00;//规定pitch的角度零点 1.5
float Pitch_ERROR = 0.00;               //Pitch偏差值
short MotorDutyQ = 0;                   //动量轮电机驱动占空比数值
short MotorDutyH = 0;                   //电机驱动占空比数值
// int DUOJI_PWM;                          //舵机的PWM
short  Velocity2;                        // 速度，定时周期内为60个脉冲，龙邱带方向512编码器
unsigned short Dduty=0;                  //动量轮pwm值
unsigned short Fduty=0;                  //舵机pwm值


/*****标识定义*****/
unsigned char Stop_Flag = 0;           //停车标识
uint8  Start_Flag2=0;                  //启动标志
#define Servo_Mid  950                //舵机自行中值
int fanxian_flag;                      //倾斜方向标志

void Balance_FHL_Bingji(void)
{
    LQ_DMP_Read();                                   //陀螺仪数据读取，pitch左正右负
//    ENC_InitConfig(ENC6_InPut_P20_3, ENC6_Dir_P20_0);//读取编码器数据
    encValue_D = ENC_GetCounter(ENC6_InPut_P20_3);    //动量轮的数值
    encValue_H = ENC_GetCounter(ENC5_InPut_P10_3);

    /*动量轮控制*/
    Pitch_ERROR = Pitch_LINGDIAN - Pitch;
    PWM_D = X_balance_Control(Pitch,Pitch_ERROR,gyro[0]);    //动量轮平衡控制
    PWM_S = -Velocity_Control(-encValue_D);              //动量轮速度环正反馈
    MotorDutyQ = -(PWM_D-PWM_S);
    /*动量轮限幅*/
    if(MotorDutyQ>8000)PWM_D=8000;
    else if(MotorDutyQ<-8000)PWM_D=-8000;
    else if(MotorDutyQ<-0) MotorDutyQ -=800;      //死区
    else if(MotorDutyQ>0) MotorDutyQ +=800;

    if((MotorDutyQ<1000)&&(MotorDutyQ>-1000))
        MotorDutyQ=0;
    /*舵机控制*/

    /*电机控制*/
    MotorDutyH = SBB_Get_MotorPI(encValue_H, Velocity2)/2;
    /*停车控制*/
    if((Pitch > 23) || (Pitch < -23)) //摔倒判断
        Stop_Flag = 1;
    if(Stop_Flag == 1)               //停车
    {
        MotorDutyQ=0;
        MotorDutyH=0;
    }

    /*输出*/
//    ServoCtrl();
    MotorCtrl(MotorDutyQ, MotorDutyH);
}

/********************************
串级平衡函数
********************************/
void Balance_FHL_Chuangji(void)
{
    float shiji_Angle;
    float cha;

    encValue_D = ENC_GetCounter(ENC6_InPut_P20_3);    //动量轮的数值
    LQ_DMP_Read();               //读取pitch，左正右负
    if(Pitch>Pitch_LINGDIAN) fanxian_flag=0;  //左倾判断
    if(Pitch<Pitch_LINGDIAN) fanxian_flag=1;  //右倾判断

    if(fanxian_flag == 0)
    {
        shiji_Angle =-Pitch;
        Dduty = Balance_PID1(Pitch_LINGDIAN, shiji_Angle);
        cha = Pitch - Pitch_LINGDIAN;
        Fduty= cha * 10;
    }
    if(fanxian_flag == 1)
    {
        shiji_Angle =Pitch;
        Dduty = Balance_PID1(Pitch_LINGDIAN, shiji_Angle);   
        cha = Pitch_LINGDIAN - Pitch;
        Fduty= cha * 10;
    }

    if(Fduty > 1090) Fduty = 1090;//舵机数值限幅
    if(Fduty < 810) Fduty = 810;

    Motor_konzhi(Dduty);
    Servo_konzhi(Fduty);
}

/****************************************************************
串级角速度环
qiwan_Angle:期望陀螺仪保持的角度
shiji_Angle:陀螺仪实际的角度
****************************************************************/
unsigned short Balance_PID1(float qiwan_Angle, float shiji_Angle)
{
    float last_error=0, error;
    error = qiwan_Angle - shiji_Angle;

    JSD_jifen +=error;
    if(JSD_jifen > 8000) JSD_jifen=8000;
    if(JSD_jifen < 100) JSD_jifen=100;

    JSD_weifen = error - last_error;

    JSD_pwm_out = JSD_kp* error + JSD_ki* JSD_jifen + JSD_kd* JSD_weifen;
    if(JSD_pwm_out > 8000) JSD_pwm_out=8000;
    if(JSD_pwm_out < 0)    JSD_pwm_out=0;
    
    return JSD_pwm_out;
}
/*************************************
电机控制(正反转)
输入:电机的pwm值
**************************************/
void Motor_konzhi(unsigned short motor)
{
    ATOM_PWM_InitConfig(ATOMPWM0, 5000, 12500);
    ATOM_PWM_InitConfig(ATOMPWM1, 5000, 12500);

    if(fanxian_flag == 0)
    ATOM_PWM_SetDuty(ATOMPWM1, motor, 12500);//电机左转
    if(fanxian_flag == 1)
    ATOM_PWM_SetDuty(ATOMPWM0, motor, 12500);//电机右转
}

void Servo_konzhi(int panduan)
{
    ATOM_PWM_InitConfig(ATOMSERVO1, Servo_Mid, 100);//舵机频率为100HZ，初始值为1.5ms中值
	// ATOM_PWM_InitConfig(ATOMSERVO2, Servo_Mid, 100);//舵机理论范围为：0.5ms--2.5ms，大多舵机实际比这个范围小
    
    // ATOM_PWM_SetDuty(ATOMSERVO2, duty, 100);//驱动两个舵机
	ATOM_PWM_SetDuty(ATOMSERVO1, duty, 100);

}


