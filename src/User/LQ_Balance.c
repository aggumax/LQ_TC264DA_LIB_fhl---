#include "LQ_Balance.h"
#include "../APP/LQ_MPU6050_DMP.h"
#include <Platform_Types.h>
#include <stdio.h>
#include "../User/LQ_MotorServo.h"
#include "../APP/LQ_GPIO_KEY.h"
#include "../APP/LQ_TFT18.h"
#include "../Driver/LQ_ADC.h"
#include "../Driver/LQ_CCU6.h"
#include "../Driver/LQ_STM.h"
#include "LQ_MotorServo.h"
#include "../APP/LQ_IIC_Gyro.h"
#include "../Driver/LQ_SOFTI2C.h"
#include "../Driver/LQ_STM.h"
#include "../Driver/LQ_GPT12_ENC.h"
#include "../Driver/LQ_GTM.h"
#include "../User/LQ_PID.h"
#include "LQ_CAMERA.h"

#include "Mycode.h"
uint8  Start_Flag=0;                    //启动标志
//直流电机
//float X_Balance_KP=1900.001,X_Balance_KI=3.2001,X_Balance_KD=60.001;  // 平衡环PID  Pitch轴角度环PID参数 动量轮
//float X_Velocity_KP=90,X_Velocity_KI=100;//动量轮电机速度环PI参数
//无刷电机

// 直立
//float X_Balance_KP=3650,X_Balance_KI=0,X_Balance_KD=70;   // 平衡环PID  Pitch轴角度环PID参数 动量轮
//float X_Velocity_KP=30,X_Velocity_KI=30;//动量轮电机速度环PI参数

// 行进
float X_Balance_KP=3750, X_Balance_KI=0, X_Balance_KD=65;   // 平衡环PID  Pitch轴角度环PID参数 动量轮
float X_Velocity_KP=20,X_Velocity_KI=110;//动量轮电机速度环PI参数


#define Motor_Kp     13              // 后轮电机PID参数
#define Motor_Ki     20               // 后轮电机PID参数

#define Balance_Kp   90                // 舵机PID参数 32
#define Balance_Kd   0.03            // 舵机PID参数    0.03
#define Balance_Ki   0.002          // 舵机PID参数 0.002

float Pitch_Zero=5.5,Pitch_error=0.00;//设置Pitch轴角度零点 5.8
float Zero_error = 0.00;              // Pitch偏差值
int PWM_X,PWM_accel;                  // PWM中间量
short  PWMMotor, PWMServo;            // 电机舵机PMW变量中值
short MotorDutyA = 0;                // 飞轮电机驱动占空比数值
short MotorDutyB = 0;                // 后轮电机驱动占空比数值
short  Motor_Bias, Motor_Last_Bias, Motor_Integration; // 电机所用参数
float   Integration;

unsigned char Flag_Stop = 0, Flag_Show, Flag_Imu;          // 停车，显示，IMU完成标志
//extern int Vbat;
//extern short BLDCduty;
extern sint16 TempAngle;
extern sint16 OFFSET0;      //最远处，赛道中心值综合偏移量
extern sint16 OFFSET1;      //第二格
extern sint16 OFFSET2;      //最近，第三格
extern sint16 TXV;          //梯形的左高度，右高度
short  Velocity = 20;                 // 速度，定时周期内为60个脉冲，龙邱带方向512编码器

//void Motor_HHH(sint32 motor)
//{
//    if(motor > 0)
//    {
//        ATOM_PWM_SetDuty(MOTOR1_P, motor, MOTOR_FREQUENCY);
//    }
//    else {
//        ATOM_PWM_SetDuty(MOTOR1_N, motor, MOTOR_FREQUENCY);
//    }
//
//}

void Balance(void)
{
//    int  Servo_PWM;                             // 舵机PID
    /* 获取编码器值 */
    ECPULSE2 = ENC_GetCounter(ENC4_InPut_P02_8); // 后轮反馈     母板编码器2
    LQ_DMP_Read();//pitch 左负右正
//    Seek_Road();  // 获取赛道颜色偏差
//    TempAngle = OFFSET2+OFFSET2+OFFSET1;    // 计算赛道颜色偏差值
//    Zero_error =(Pitch_Zero-(float)TempAngle/1200);  // 计算倾斜角度偏差值
    /////// 动量轮控制//////////
    PWM_X = X_balance_Control(Pitch,Pitch_Zero,gyro[0]);// 动量轮电机控制左右倾角
    PWM_accel = - Velocity_Control(-ECPULSE1);          // 动量轮电机速度环正反馈
    if(PWM_X>8000)PWM_X = 8000;
    else if(PWM_X<-8000)PWM_X = -8000;
    if(PWM_accel>8000) PWM_accel=8000;
    else if(PWM_accel<-8000) PWM_accel=-8000;
    MotorDutyA = -(PWM_X-PWM_accel);//BLDCduty= Velocity_Momentum(distence,ECPULSE1);

    if(MotorDutyA>8000) MotorDutyA=8000;        // 动量轮电机限幅
    else if(MotorDutyA<-8000) MotorDutyA=-8000; // 动量轮电机限幅
    else if(MotorDutyA<0) MotorDutyA -=2000;    // 死区
    else if(MotorDutyA>0) MotorDutyA+=2000;      // 死区
    ///////// 舵机电机控制///////
//    if( Servo_PWM < - Servo_Delta)    Servo_PWM = - Servo_Delta;  // 舵机角度限制
//    else if(Servo_PWM > Servo_Delta)  Servo_PWM =   Servo_Delta;  // 舵机角度限制
//    PWMServo = Servo_Center_Mid - TempAngle /13;                      // 转换为舵机控制PWM
//    MotorDutyB = SBB_Get_MotorPI(ECPULSE2, Velocity)/5;           // 电机增量式PI控制
    if((Pitch > 23) || (Pitch < -23))       // 摔倒停车判断
       Flag_Stop = 1;
    if(Flag_Stop == 1)                       // 停车
    {
        MotorDutyA = 0;                      // 电机关闭
        MotorDutyB = 0;                      // 电机关闭
        Integration = 0;                     // 积分参数归零
    }
//    ServoCtrl(PWMServo);                    // 舵机控制
//    MotorCtrl(MotorDutyA,0);                // 后轮电机控制
//    Motor_HHH(MotorDutyA);
    Motor_konzhi(MotorDutyA);

}


/**************************************************************************
X轴平衡PID控制,角度环
**************************************************************************/
float X_balance_Control(float Angle,float Angle_Zero,float Gyro)
{
     float PWM,Bias;
     static float error;
     Bias=Angle-Angle_Zero;                                            //获取偏差
     error+=Bias;                                                      //偏差累积
     if(error>+30) error=+30;                                          //积分限幅
     if(error<-30) error=-30;                                          //积分限幅
     PWM=X_Balance_KP*Bias + X_Balance_KI*error + Gyro*X_Balance_KD;   //获取最终数值
     return PWM;
}

/**************************************************************************
动量轮速度PI控制,速度正反馈环
**************************************************************************/
float Velocity_Control(int encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = encoder;                                                  //速度滤波
    Encoder *= 0.7;                                                           //一阶低通滤波器
    Encoder += Encoder_Least*0.3;                                             //一阶低通滤波器
    Encoder_Integral += Encoder;                                              //积分出位移
    if(Encoder_Integral > +2000) Encoder_Integral = +2000;                    //积分限幅
    if(Encoder_Integral < -2000) Encoder_Integral = -2000;                    //积分限幅
    Velocity = Encoder * X_Velocity_KP + Encoder_Integral * X_Velocity_KI/100;//获取最终数值
    if(Flag_Stop==1) Encoder_Integral=0,Encoder=0,Velocity=0;                //停止时参数清零
    return Velocity;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【函数名】int SBB_Get_BalancePID(float Angle,float Gyro)
【功  能】舵机打角控制PID
【参数值】float Angle,电单车左右倾角
【参数值】float Gyro 电单车左右角速度
【返回值】小车平衡，舵机转向PID
【作  者】chiusir
【最后更新】2021年1月22日
【软件版本】V1.0
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int SBB_Get_BalancePID(float Angle,float Gyro)
{
    float  Bias;
    int SBB_BalancePID;
    Bias = Angle - Zero_error;     // 求出平衡的角度中值和此时横滚角的偏差
    Integration += Bias;           // 积分
    if(Integration<-380)      Integration=-380; //限幅
    else if(Integration>380)  Integration= 380; //限幅
    //===计算平衡控制的舵机PWM  PID控制 kp是P系数 ki是I系数 kd是D系数
    SBB_BalancePID = Balance_Kp * Bias + Balance_Ki*Integration + Balance_Kd*Gyro;
    return SBB_BalancePID;
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【函数名】int SBB_Get_MotorPI (int Encoder,int Target)
【功  能】电机控制增量式PI
【参数值】int Encoder 编码器脉采集的冲数
【参数值】int Target  期望脉冲数
【返回值】电机PWM
【作  者】chiusir
【最后更新】2021年1月22日
【软件版本】V1.0
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int SBB_Get_MotorPI (int Encoder,int Target)
{
    static int Pwm;
    Motor_Bias = Encoder - Target;            // 计算偏差
    Pwm += (Motor_Kp * (Motor_Bias - Motor_Last_Bias) + Motor_Ki * Motor_Bias);
    // ==增量式PI控制器
   if(Pwm > 8000) Pwm = 8000;               // 限幅
    else if(Pwm < -8000)Pwm = -8000;         // 限幅
    Motor_Last_Bias = Motor_Bias;            // 保存上一次偏差
    return Pwm;                              // 增量输出
}



//void Balance_FHL(void)
//{
//    //左正右负
//}


