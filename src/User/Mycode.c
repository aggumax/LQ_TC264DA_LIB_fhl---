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
#include "LQ_GPIO_KEY.h"
#include "LQ_STM.h"

/*****PID数值定义*****/
/*串级PID的角速度环*/
float  JSD_kp=1100;//1100
float  JSD_ki=0;
float  JSD_kd=0;
/*串级PID的角度环*/
float  JD_kp=0;//5
float  JD_ki=0;//1
float  JD_kd=0;
/*串级PID的速度环*/
float  SD_kp=0;
float  SD_ki=0;
float  SD_kd=0;


/*****数值定义*****/
int PWM_D, PWM_S;                       //PWM_D是动量轮电机控制左右倾斜，S是动量轮电机速度环正反馈
int encValue_D = 0;                     //储存动量轮的编码器数值
int encValue_H = 0;                     //储存电机的编码器数值
float Pitch_LINGDIAN = 0.5, Pitch_error2 = 0.00;//规定pitch的角度零点 1.5
float Pitch_ERROR = 0.00;               //Pitch偏差值
short MotorDutyQ = 0;                   //动量轮电机驱动占空比数值
short MotorDutyH = 0;                   //电机驱动占空比数值
// int DUOJI_PWM;                          //舵机的PWM
short  Velocity2;                        // 速度，定时周期内为60个脉冲，龙邱带方向512编码器
unsigned short Dduty=0;                  //动量轮pwm值
unsigned short Fduty=0;                  //舵机pwm值
unsigned short Hduty=6000;
float GYRO = 0;
float YB=0;


/*****标识定义*****/
int Stop_Flag = 0;           //停车标识
//uint8  Start_Flag2=0;                  //启动标识
int  Start_Flag2=0;                  //启动标识
#define Servo_Mid  950                //舵机自行中值
int fanxian_flag;                      //倾斜方向标志


/********************************
串级平衡函数
********************************/
void Balance_FHL_Chuangji(void)
{
//    float shiji_Angle;
//    float cha;
    char txt[16];
    GPIO_KEY_Init();
    ENC_InitConfig(ENC4_InPut_P02_8, ENC4_Dir_P33_5);
    encValue_D = ENC_GetCounter(ENC4_InPut_P02_8);    //动量轮的数值
    LQ_DMP_Read();               //读取pitch，左正右负

    if(Pitch>Pitch_LINGDIAN) fanxian_flag=0;  //左倾判断
    if(Pitch<Pitch_LINGDIAN) fanxian_flag=1;  //右倾判断

    if(KEY_Read(KEY2)==0)//按下KEY2键
        YB +=10;
    if(KEY_Read(KEY1)==0)//按下KEY1键
        YB -=10;
    if(KEY_Read(KEY0)==0)//按下KEY0键
        JSD_ki = YB;

    if(fanxian_flag == 0)
    {
        GYRO = -gyro[0];
        Dduty = Balance_PID_CJ(Balance_PID_CJJD(0,-Pitch),GYRO);
    }
    if(fanxian_flag == 1)
    {
        GYRO = gyro[0];
        Dduty = Balance_PID_CJ(Balance_PID_CJJD(0, Pitch),GYRO);
    }

    sprintf((char*)txt,"encValue:%05d",encValue_D);//
    TFTSPI_P8X16Str(0,3,txt,u16WHITE,u16BLACK);
    sprintf((char*)txt,"KP:%05f",JSD_kp);//
    TFTSPI_P8X16Str(0,5,txt,u16WHITE,u16BLACK);
    sprintf((char*)txt,"YB:%f",YB);//
    TFTSPI_P8X16Str(0,6,txt,u16WHITE,u16BLACK);


    Stop_Flag = Down_flag();
    if(Stop_Flag == 1) Dduty=0;              //停车

    Motor_konzhi(Dduty);
}

/*****************************************************************
串级角速度环
@qiwan_Angle:期望陀螺仪保持的角速度
@shiji_Angle:陀螺仪实际的角速度

@输出：电机的PWM值
******************************************************************/
unsigned short Balance_PID_CJ(float qiwan_Angle, float shiji_Angle)
{
    float last_error=0, error;
    error = qiwan_Angle - shiji_Angle;
    float  JSD_pwm_out =0;
    float  JSD_jifen   =0;
    float  JSD_weifen  =0;

    JSD_jifen +=error;
    if(JSD_jifen > 2000) JSD_jifen=2000;
    if(JSD_jifen < 0) JSD_jifen=0;

    JSD_weifen = error - last_error;

    JSD_pwm_out = JSD_kp* error + JSD_ki* JSD_jifen + JSD_kd* JSD_weifen;
    if(JSD_pwm_out > 30000) JSD_pwm_out=30000;
    if(JSD_pwm_out < 0)    JSD_pwm_out=0;
    last_error = error;
    
    return JSD_pwm_out;
}

/**********************************************
串级角度环
@qiwan：期望车维持的角度
@shiji：实际车的角度

@输出：为角速度环的期望值
***********************************************/
unsigned short Balance_PID_CJJD(float qiwan, float shiji)
{
    float last_error=0, error;
    float  JD_out =0;
    float  JD_jifen   =0;
    float  JD_weifen  =0;
    error = qiwan - shiji;

    JD_jifen +=error;
    if(JD_jifen > 2000) JD_jifen=2000;
    if(JD_jifen < 0) JD_jifen=0;

    JD_weifen = error - last_error;

    JD_out = JD_kp* error + JD_ki* JD_jifen + JD_kd* JD_weifen;
    if(JD_out > 8000) JD_out=8000;
    if(JD_out < 0)    JD_out=0;
    last_error = error;

    return JD_out;
}
/**********************************************
串级速度环
@qiwan：0
@shiji：编码器的测量值

@输出：为角度环的期望值
***********************************************/
unsigned short Balance_PID_CJSD(float qiwan, float shiji)
{
    float last_error=0,error;
    float SD_out=0;
    float SD_jifen=0;
    float SD_weifen=0;
    error =qiwan - shiji;

    SD_jifen +=error;
    if(SD_jifen > 2000) SD_jifen=2000;
    if(SD_jifen < 0) SD_jifen=0;

    SD_weifen = error - last_error;

    SD_out = SD_kp* error + SD_ki* SD_jifen + SD_kd* SD_weifen;
    if(SD_out > 8000) SD_out=8000;
    if(SD_out < 0)    SD_out=0;
    last_error = error;

    return SD_out;
}

/*************************************
电机控制(正反转)
输入:电机的pwm值
**************************************/
void Motor_konzhi(unsigned short motor)
{
    ATOM_PWM_InitConfig(ATOMPWM0, 0, 12500);
    ATOM_PWM_InitConfig(ATOMPWM1, 0, 12500);

    if(fanxian_flag == 0)
    ATOM_PWM_SetDuty(ATOMPWM1, motor, 12500);//电机左转
    if(fanxian_flag == 1)
    ATOM_PWM_SetDuty(ATOMPWM0, motor, 12500);//电机右转
}

void Servo_konzhi(unsigned short panduan)
{
	// ATOM_PWM_InitConfig(ATOMSERVO2, Servo_Mid, 100);//舵机理论范围为：0.5ms--2.5ms，大多舵机实际比这个范围小
    
    // ATOM_PWM_SetDuty(ATOMSERVO2, duty, 100);//驱动两个舵机
	ATOM_PWM_SetDuty(ATOMSERVO1, panduan, 100);

}

/*************
 摔倒判断
**************/
int Down_flag()
{
    int flag;
    if(Pitch>23 || Pitch<-23)
        flag=1;
    else flag=0;
//    delayms(20);

    return flag;
}

void Balance_DJ(void)
{

    LQ_DMP_Read();
    Hduty = 6000;
//    GPIO_KEY_Init();
//    if(KEY_Read(KEY0)==0)
//        Hduty -=100;
//    if(KEY_Read(KEY2)==0)
//        Hduty +=100;
//    if(KEY_Read(KEY1)==0)
//        Hduty = 5000;

    if(Pitch>0)Fduty = 1950+Pitch*15;
    if(Pitch<0)Fduty = 1950+Pitch*15;

    Stop_Flag = Down_flag();
    if(Stop_Flag == 1) Hduty=0;              //停车
    ATOM_PWM_SetDuty(ATOMPWM2, Hduty, 12500);

    Servo_konzhi(Fduty);

}


