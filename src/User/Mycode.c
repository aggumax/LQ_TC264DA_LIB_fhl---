#include "Mycode.h"
#include "LQ_GPT12_ENC.h"
#include "LQ_CCU6.h"
#include "LQ_MPU6050_DMP.h"
#include "LQ_GPT_mini512.h"

uint8  Start_Flag2=0;                    //������־

/*PID��ֵ����*/
#define DUOJI_Kp      0;
#define DUOJI_Ki      0;
#define DUOJI_Kd      0;                //���PID����

float PINHEN_KP = 0, PINHEN_KI = 0, PINHEN_KD = 0;//ƽ�⻷PID����,�����֣�pitch�ǶȻ�
float D_SPEED_KP=0, D_SPEED_KI=0,D_SPEED_KD=0;    //�����ֵ���ٶȻ�PID����

/*��ֵ����*/
int PWM_D, PWM_S;                       //PWM_D�Ƕ����ֵ������������б��S�Ƕ����ֵ���ٶȻ�������
int encValue_D = 0;                     //���涯���ֵı�������ֵ
float Pitch_LINGDIAN = 1.5, Pitch_error2 = 0.00;//�涨pitch�ĽǶ���� 1.5
float Pitch_ERROR = 0.00;               //Pitchƫ��ֵ


void Balance_FHL(void)
{
    LQ_DMP_Read();                                   //���������ݶ�ȡ��pitch�����Ҹ�
    ENC_InitConfig(ENC6_InPut_P20_3, ENC6_Dir_P20_0);//��ȡ����������
    encValue_D = ENC_GetCounter(ENC6_InPut_P20_3);    //�����ֵ���ֵ

    /*�����ֿ���*/
    PWM_D = Balance_X(Pitch,Pitch_ERROR,gyro[0]);
//    PWM_S =
}


/*******************
ƽ��PID���ƺ������ǶȻ�
 ******************/
float Balance_X(float Angle,float Angle_Zero,float Gyro)
{
    float PWM, Bias;
    static float error;
    Bias=Angle-Angle_Zero;      //��ȡƫ��
    error+=Bias;                //ƫ���ۻ�
    if(error>+30) error=+30;
    if(error>-30) error=-30;    //�����޷�
    PWM = PINHEN_KP*Bias + PINHEN_KI*error + PINHEN_KD*Gyro;//��ȡ������ֵ

    return PWM;
}


/*******************
�������ٶ�PI����,�ٶ���������
 ******************/
float SPEED_Control(int encValueD)
{

}


