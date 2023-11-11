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

/*****PID��ֵ����*****/
/*����PID�Ľ��ٶȻ�*/
float  JSD_kp=1100;//1100
float  JSD_ki=0;
float  JSD_kd=0;
/*����PID�ĽǶȻ�*/
float  JD_kp=0;//5
float  JD_ki=0;//1
float  JD_kd=0;
/*����PID���ٶȻ�*/
float  SD_kp=0;
float  SD_ki=0;
float  SD_kd=0;


/*****��ֵ����*****/
int PWM_D, PWM_S;                       //PWM_D�Ƕ����ֵ������������б��S�Ƕ����ֵ���ٶȻ�������
int encValue_D = 0;                     //���涯���ֵı�������ֵ
int encValue_H = 0;                     //�������ı�������ֵ
float Pitch_LINGDIAN = 0.5, Pitch_error2 = 0.00;//�涨pitch�ĽǶ���� 1.5
float Pitch_ERROR = 0.00;               //Pitchƫ��ֵ
short MotorDutyQ = 0;                   //�����ֵ������ռ�ձ���ֵ
short MotorDutyH = 0;                   //�������ռ�ձ���ֵ
// int DUOJI_PWM;                          //�����PWM
short  Velocity2;                        // �ٶȣ���ʱ������Ϊ60�����壬���������512������
unsigned short Dduty=0;                  //������pwmֵ
unsigned short Fduty=0;                  //���pwmֵ
unsigned short Hduty=6000;
float GYRO = 0;
float YB=0;


/*****��ʶ����*****/
int Stop_Flag = 0;           //ͣ����ʶ
//uint8  Start_Flag2=0;                  //������ʶ
int  Start_Flag2=0;                  //������ʶ
#define Servo_Mid  950                //���������ֵ
int fanxian_flag;                      //��б�����־


/********************************
����ƽ�⺯��
********************************/
void Balance_FHL_Chuangji(void)
{
//    float shiji_Angle;
//    float cha;
    char txt[16];
    GPIO_KEY_Init();
    ENC_InitConfig(ENC4_InPut_P02_8, ENC4_Dir_P33_5);
    encValue_D = ENC_GetCounter(ENC4_InPut_P02_8);    //�����ֵ���ֵ
    LQ_DMP_Read();               //��ȡpitch�������Ҹ�

    if(Pitch>Pitch_LINGDIAN) fanxian_flag=0;  //�����ж�
    if(Pitch<Pitch_LINGDIAN) fanxian_flag=1;  //�����ж�

    if(KEY_Read(KEY2)==0)//����KEY2��
        YB +=10;
    if(KEY_Read(KEY1)==0)//����KEY1��
        YB -=10;
    if(KEY_Read(KEY0)==0)//����KEY0��
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
    if(Stop_Flag == 1) Dduty=0;              //ͣ��

    Motor_konzhi(Dduty);
}

/*****************************************************************
�������ٶȻ�
@qiwan_Angle:���������Ǳ��ֵĽ��ٶ�
@shiji_Angle:������ʵ�ʵĽ��ٶ�

@����������PWMֵ
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
�����ǶȻ�
@qiwan��������ά�ֵĽǶ�
@shiji��ʵ�ʳ��ĽǶ�

@�����Ϊ���ٶȻ�������ֵ
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
�����ٶȻ�
@qiwan��0
@shiji���������Ĳ���ֵ

@�����Ϊ�ǶȻ�������ֵ
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
�������(����ת)
����:�����pwmֵ
**************************************/
void Motor_konzhi(unsigned short motor)
{
    ATOM_PWM_InitConfig(ATOMPWM0, 0, 12500);
    ATOM_PWM_InitConfig(ATOMPWM1, 0, 12500);

    if(fanxian_flag == 0)
    ATOM_PWM_SetDuty(ATOMPWM1, motor, 12500);//�����ת
    if(fanxian_flag == 1)
    ATOM_PWM_SetDuty(ATOMPWM0, motor, 12500);//�����ת
}

void Servo_konzhi(unsigned short panduan)
{
	// ATOM_PWM_InitConfig(ATOMSERVO2, Servo_Mid, 100);//������۷�ΧΪ��0.5ms--2.5ms�������ʵ�ʱ������ΧС
    
    // ATOM_PWM_SetDuty(ATOMSERVO2, duty, 100);//�����������
	ATOM_PWM_SetDuty(ATOMSERVO1, panduan, 100);

}

/*************
 ˤ���ж�
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
    if(Stop_Flag == 1) Hduty=0;              //ͣ��
    ATOM_PWM_SetDuty(ATOMPWM2, Hduty, 12500);

    Servo_konzhi(Fduty);

}


