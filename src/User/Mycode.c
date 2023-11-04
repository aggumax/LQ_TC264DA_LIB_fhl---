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

/*****PID��ֵ����*****/
#define DUOJI_Kp      0;    //���PID����
#define DUOJI_Ki      0;
#define DUOJI_Kd      0;
/*ƽ�⻷PID����,�����֣�pitch�ǶȻ�*/
float PINHEN_KP = 0;       
float PINHEN_KI = 0;
float PINHEN_KD = 0;
/*�����ֵ���ٶȻ�PID����*/
float D_SPEED_KP=0;         
float D_SPEED_KI=0;
/*����PID�Ľ��ٶȻ�*/
float  JSD_kp=3000;               
float  JSD_ki=0;
float  JSD_kd=60;
float  JSD_pwm_out =0;
float  JSD_jifen   =0;
float  JSD_weifen  =0;


/*****��ֵ����*****/
int PWM_D, PWM_S;                       //PWM_D�Ƕ����ֵ������������б��S�Ƕ����ֵ���ٶȻ�������
int encValue_D = 0;                     //���涯���ֵı�������ֵ
int encValue_H = 0;                     //�������ı�������ֵ
float Pitch_LINGDIAN = 1.5, Pitch_error2 = 0.00;//�涨pitch�ĽǶ���� 1.5
float Pitch_ERROR = 0.00;               //Pitchƫ��ֵ
short MotorDutyQ = 0;                   //�����ֵ������ռ�ձ���ֵ
short MotorDutyH = 0;                   //�������ռ�ձ���ֵ
// int DUOJI_PWM;                          //�����PWM
short  Velocity2;                        // �ٶȣ���ʱ������Ϊ60�����壬���������512������
unsigned short Dduty=0;                  //������pwmֵ
unsigned short Fduty=0;                  //���pwmֵ


/*****��ʶ����*****/
unsigned char Stop_Flag = 0;           //ͣ����ʶ
uint8  Start_Flag2=0;                  //������־
#define Servo_Mid  950                //���������ֵ
int fanxian_flag;                      //��б�����־

void Balance_FHL_Bingji(void)
{
    LQ_DMP_Read();                                   //���������ݶ�ȡ��pitch�����Ҹ�
//    ENC_InitConfig(ENC6_InPut_P20_3, ENC6_Dir_P20_0);//��ȡ����������
    encValue_D = ENC_GetCounter(ENC6_InPut_P20_3);    //�����ֵ���ֵ
    encValue_H = ENC_GetCounter(ENC5_InPut_P10_3);

    /*�����ֿ���*/
    Pitch_ERROR = Pitch_LINGDIAN - Pitch;
    PWM_D = X_balance_Control(Pitch,Pitch_ERROR,gyro[0]);    //������ƽ�����
    PWM_S = -Velocity_Control(-encValue_D);              //�������ٶȻ�������
    MotorDutyQ = -(PWM_D-PWM_S);
    /*�������޷�*/
    if(MotorDutyQ>8000)PWM_D=8000;
    else if(MotorDutyQ<-8000)PWM_D=-8000;
    else if(MotorDutyQ<-0) MotorDutyQ -=800;      //����
    else if(MotorDutyQ>0) MotorDutyQ +=800;

    if((MotorDutyQ<1000)&&(MotorDutyQ>-1000))
        MotorDutyQ=0;
    /*�������*/

    /*�������*/
    MotorDutyH = SBB_Get_MotorPI(encValue_H, Velocity2)/2;
    /*ͣ������*/
    if((Pitch > 23) || (Pitch < -23)) //ˤ���ж�
        Stop_Flag = 1;
    if(Stop_Flag == 1)               //ͣ��
    {
        MotorDutyQ=0;
        MotorDutyH=0;
    }

    /*���*/
//    ServoCtrl();
    MotorCtrl(MotorDutyQ, MotorDutyH);
}

/********************************
����ƽ�⺯��
********************************/
void Balance_FHL_Chuangji(void)
{
    float shiji_Angle;
    float cha;

    encValue_D = ENC_GetCounter(ENC6_InPut_P20_3);    //�����ֵ���ֵ
    LQ_DMP_Read();               //��ȡpitch�������Ҹ�
    if(Pitch>Pitch_LINGDIAN) fanxian_flag=0;  //�����ж�
    if(Pitch<Pitch_LINGDIAN) fanxian_flag=1;  //�����ж�

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

    if(Fduty > 1090) Fduty = 1090;//�����ֵ�޷�
    if(Fduty < 810) Fduty = 810;

    Motor_konzhi(Dduty);
    Servo_konzhi(Fduty);
}

/****************************************************************
�������ٶȻ�
qiwan_Angle:���������Ǳ��ֵĽǶ�
shiji_Angle:������ʵ�ʵĽǶ�
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
�������(����ת)
����:�����pwmֵ
**************************************/
void Motor_konzhi(unsigned short motor)
{
    ATOM_PWM_InitConfig(ATOMPWM0, 5000, 12500);
    ATOM_PWM_InitConfig(ATOMPWM1, 5000, 12500);

    if(fanxian_flag == 0)
    ATOM_PWM_SetDuty(ATOMPWM1, motor, 12500);//�����ת
    if(fanxian_flag == 1)
    ATOM_PWM_SetDuty(ATOMPWM0, motor, 12500);//�����ת
}

void Servo_konzhi(int panduan)
{
    ATOM_PWM_InitConfig(ATOMSERVO1, Servo_Mid, 100);//���Ƶ��Ϊ100HZ����ʼֵΪ1.5ms��ֵ
	// ATOM_PWM_InitConfig(ATOMSERVO2, Servo_Mid, 100);//������۷�ΧΪ��0.5ms--2.5ms�������ʵ�ʱ������ΧС
    
    // ATOM_PWM_SetDuty(ATOMSERVO2, duty, 100);//�����������
	ATOM_PWM_SetDuty(ATOMSERVO1, duty, 100);

}


