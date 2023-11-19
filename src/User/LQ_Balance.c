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
uint8  Start_Flag=0;                    //������־
//ֱ�����
//float X_Balance_KP=1900.001,X_Balance_KI=3.2001,X_Balance_KD=60.001;  // ƽ�⻷PID  Pitch��ǶȻ�PID���� ������
//float X_Velocity_KP=90,X_Velocity_KI=100;//�����ֵ���ٶȻ�PI����
//��ˢ���

// ֱ��
//float X_Balance_KP=3650,X_Balance_KI=0,X_Balance_KD=70;   // ƽ�⻷PID  Pitch��ǶȻ�PID���� ������
//float X_Velocity_KP=30,X_Velocity_KI=30;//�����ֵ���ٶȻ�PI����

// �н�
float X_Balance_KP=3750, X_Balance_KI=0, X_Balance_KD=65;   // ƽ�⻷PID  Pitch��ǶȻ�PID���� ������
float X_Velocity_KP=20,X_Velocity_KI=110;//�����ֵ���ٶȻ�PI����


#define Motor_Kp     13              // ���ֵ��PID����
#define Motor_Ki     20               // ���ֵ��PID����

#define Balance_Kp   90                // ���PID���� 32
#define Balance_Kd   0.03            // ���PID����    0.03
#define Balance_Ki   0.002          // ���PID���� 0.002

float Pitch_Zero=5.5,Pitch_error=0.00;//����Pitch��Ƕ���� 5.8
float Zero_error = 0.00;              // Pitchƫ��ֵ
int PWM_X,PWM_accel;                  // PWM�м���
short  PWMMotor, PWMServo;            // ������PMW������ֵ
short MotorDutyA = 0;                // ���ֵ������ռ�ձ���ֵ
short MotorDutyB = 0;                // ���ֵ������ռ�ձ���ֵ
short  Motor_Bias, Motor_Last_Bias, Motor_Integration; // ������ò���
float   Integration;

unsigned char Flag_Stop = 0, Flag_Show, Flag_Imu;          // ͣ������ʾ��IMU��ɱ�־
//extern int Vbat;
//extern short BLDCduty;
extern sint16 TempAngle;
extern sint16 OFFSET0;      //��Զ������������ֵ�ۺ�ƫ����
extern sint16 OFFSET1;      //�ڶ���
extern sint16 OFFSET2;      //�����������
extern sint16 TXV;          //���ε���߶ȣ��Ҹ߶�
short  Velocity = 20;                 // �ٶȣ���ʱ������Ϊ60�����壬���������512������

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
//    int  Servo_PWM;                             // ���PID
    /* ��ȡ������ֵ */
    ECPULSE2 = ENC_GetCounter(ENC4_InPut_P02_8); // ���ַ���     ĸ�������2
    LQ_DMP_Read();//pitch ������
//    Seek_Road();  // ��ȡ������ɫƫ��
//    TempAngle = OFFSET2+OFFSET2+OFFSET1;    // ����������ɫƫ��ֵ
//    Zero_error =(Pitch_Zero-(float)TempAngle/1200);  // ������б�Ƕ�ƫ��ֵ
    /////// �����ֿ���//////////
    PWM_X = X_balance_Control(Pitch,Pitch_Zero,gyro[0]);// �����ֵ�������������
    PWM_accel = - Velocity_Control(-ECPULSE1);          // �����ֵ���ٶȻ�������
    if(PWM_X>8000)PWM_X = 8000;
    else if(PWM_X<-8000)PWM_X = -8000;
    if(PWM_accel>8000) PWM_accel=8000;
    else if(PWM_accel<-8000) PWM_accel=-8000;
    MotorDutyA = -(PWM_X-PWM_accel);//BLDCduty= Velocity_Momentum(distence,ECPULSE1);

    if(MotorDutyA>8000) MotorDutyA=8000;        // �����ֵ���޷�
    else if(MotorDutyA<-8000) MotorDutyA=-8000; // �����ֵ���޷�
    else if(MotorDutyA<0) MotorDutyA -=2000;    // ����
    else if(MotorDutyA>0) MotorDutyA+=2000;      // ����
    ///////// ����������///////
//    if( Servo_PWM < - Servo_Delta)    Servo_PWM = - Servo_Delta;  // ����Ƕ�����
//    else if(Servo_PWM > Servo_Delta)  Servo_PWM =   Servo_Delta;  // ����Ƕ�����
//    PWMServo = Servo_Center_Mid - TempAngle /13;                      // ת��Ϊ�������PWM
//    MotorDutyB = SBB_Get_MotorPI(ECPULSE2, Velocity)/5;           // �������ʽPI����
    if((Pitch > 23) || (Pitch < -23))       // ˤ��ͣ���ж�
       Flag_Stop = 1;
    if(Flag_Stop == 1)                       // ͣ��
    {
        MotorDutyA = 0;                      // ����ر�
        MotorDutyB = 0;                      // ����ر�
        Integration = 0;                     // ���ֲ�������
    }
//    ServoCtrl(PWMServo);                    // �������
//    MotorCtrl(MotorDutyA,0);                // ���ֵ������
//    Motor_HHH(MotorDutyA);
    Motor_konzhi(MotorDutyA);

}


/**************************************************************************
X��ƽ��PID����,�ǶȻ�
**************************************************************************/
float X_balance_Control(float Angle,float Angle_Zero,float Gyro)
{
     float PWM,Bias;
     static float error;
     Bias=Angle-Angle_Zero;                                            //��ȡƫ��
     error+=Bias;                                                      //ƫ���ۻ�
     if(error>+30) error=+30;                                          //�����޷�
     if(error<-30) error=-30;                                          //�����޷�
     PWM=X_Balance_KP*Bias + X_Balance_KI*error + Gyro*X_Balance_KD;   //��ȡ������ֵ
     return PWM;
}

/**************************************************************************
�������ٶ�PI����,�ٶ���������
**************************************************************************/
float Velocity_Control(int encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = encoder;                                                  //�ٶ��˲�
    Encoder *= 0.7;                                                           //һ�׵�ͨ�˲���
    Encoder += Encoder_Least*0.3;                                             //һ�׵�ͨ�˲���
    Encoder_Integral += Encoder;                                              //���ֳ�λ��
    if(Encoder_Integral > +2000) Encoder_Integral = +2000;                    //�����޷�
    if(Encoder_Integral < -2000) Encoder_Integral = -2000;                    //�����޷�
    Velocity = Encoder * X_Velocity_KP + Encoder_Integral * X_Velocity_KI/100;//��ȡ������ֵ
    if(Flag_Stop==1) Encoder_Integral=0,Encoder=0,Velocity=0;                //ֹͣʱ��������
    return Velocity;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����������int SBB_Get_BalancePID(float Angle,float Gyro)
����  �ܡ������ǿ���PID
������ֵ��float Angle,�絥���������
������ֵ��float Gyro �絥�����ҽ��ٶ�
������ֵ��С��ƽ�⣬���ת��PID
����  �ߡ�chiusir
�������¡�2021��1��22��
������汾��V1.0
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int SBB_Get_BalancePID(float Angle,float Gyro)
{
    float  Bias;
    int SBB_BalancePID;
    Bias = Angle - Zero_error;     // ���ƽ��ĽǶ���ֵ�ʹ�ʱ����ǵ�ƫ��
    Integration += Bias;           // ����
    if(Integration<-380)      Integration=-380; //�޷�
    else if(Integration>380)  Integration= 380; //�޷�
    //===����ƽ����ƵĶ��PWM  PID���� kp��Pϵ�� ki��Iϵ�� kd��Dϵ��
    SBB_BalancePID = Balance_Kp * Bias + Balance_Ki*Integration + Balance_Kd*Gyro;
    return SBB_BalancePID;
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����������int SBB_Get_MotorPI (int Encoder,int Target)
����  �ܡ������������ʽPI
������ֵ��int Encoder ���������ɼ��ĳ���
������ֵ��int Target  ����������
������ֵ�����PWM
����  �ߡ�chiusir
�������¡�2021��1��22��
������汾��V1.0
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int SBB_Get_MotorPI (int Encoder,int Target)
{
    static int Pwm;
    Motor_Bias = Encoder - Target;            // ����ƫ��
    Pwm += (Motor_Kp * (Motor_Bias - Motor_Last_Bias) + Motor_Ki * Motor_Bias);
    // ==����ʽPI������
   if(Pwm > 8000) Pwm = 8000;               // �޷�
    else if(Pwm < -8000)Pwm = -8000;         // �޷�
    Motor_Last_Bias = Motor_Bias;            // ������һ��ƫ��
    return Pwm;                              // �������
}



//void Balance_FHL(void)
//{
//    //�����Ҹ�
//}


