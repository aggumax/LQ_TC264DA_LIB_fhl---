#include "LQ_BLDC.h"
#include "LQ_TFT18.h"
#include "LQ_ADC.h"
#include "LQ_GTM.h"
#include "LQ_CCU6.h"
#include "LQ_GPIO.h"
#include "LQ_GPIO_KEY.h"
#include "LQ_GPIO_LED.h"
#include "LQ_MotorServo.h"
#include "LQ_GPT12_ENC.h"
#include <stdio.h>


#define BLDC_PWM_FREQUENCY    ATOM_PWM_MAX
BLDC_MANAGER g_sBLDCMag;//��ˢ�ṹ������
short BLDCduty=800;//��ˢ���pwm
float y_vel_prev=0;//��ͨ�˲�
float Speed_KP=0;
float Speed_KI=0;
uint16_t IA = 0,IB = 0,IC = 0;//�����������
int Move_distance=20;//Ŀ���ٶ�
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void MotorInit(uint16 freq)
@����˵������ˢ������ţ�pwm��ʼ��
@����˵����pwm Ƶ��
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����BLDC_MotorInit(1000-1);
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void BLDC_MotorInit(uint16 freq)
{
    ATOM_PWM_InitConfig(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0, MOTOR_FREQUENCY);
    ATOM_PWM_InitConfig(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0, MOTOR_FREQUENCY);
    ATOM_PWM_InitConfig(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0, MOTOR_FREQUENCY);


    PIN_InitConfig(P32_4, PIN_MODE_OUTPUT, 0);
    PIN_InitConfig(P22_3, PIN_MODE_OUTPUT, 0);
    PIN_InitConfig(P20_8, PIN_MODE_OUTPUT, 0);


    g_sBLDCMag.Electricity_flag =TRUE;  //��ʼ�������ж������������ϵ���
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void BLDC_HallInit(void);
@����˵������ʼ���������ż��������ӿ�
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע���������ӿ�ͬʱΪ��������״̬��ȡ�ӿ�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void BLDC_HallInit(void)
{
   PIN_InitConfig(P13_0, IfxPort_Mode_inputPullUp, 1);
   ENC_InitConfig(ENC5_InPut_P10_3, ENC5_Dir_P10_1);//BC��ʹ��
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�uint8_t Get_Hall_State(void);
@����˵������ȡ����״̬
@����˵������
@�������أ�hall_state������״̬��
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
uint8_t Get_Hall_State(void)
{
  uint8_t hall_state = 0;

  if(PIN_Read(P13_0)) //F11/* ��ȡ���������� U ��״̬ */
  {
    hall_state |= 0x01U << 0;
  }
  if(PIN_Read(P10_1)) //F12/* ��ȡ���������� V ��״̬ */
  {
    hall_state |= 0x01U << 1;
  }
  if(PIN_Read(P10_3)) //F13 /* ��ȡ���������� W ��״̬ */
  {
    hall_state |= 0x01U << 2;
  }
  return hall_state;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void BLDC_Motor_Hall_Run(int16_t motor_duty);
@����˵������ˢ��������������л���
@����˵����motor_duty�����ռ�ձȣ�
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void BLDC_Motor_Hall_Run(int16_t motor_duty)
{
  /* ��ȡ��������������״̬,��Ϊ��������� */
  g_sBLDCMag.step = Get_Hall_State();
  g_sBLDCMag.duty = motor_duty;
  if(motor_duty>0)LQ_BLDCCorotation();//��ʱ�뷽��
  else if(motor_duty==0)LQ_BLDCStop();
  else if(motor_duty<0)LQ_BLDCReversal();
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void LQ_BLDCCorotation(void);
@����˵����������ˢ�����ת
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_BLDCCorotation(void)
{
    switch(g_sBLDCMag.step) //U+ U-  V+ V-  W+ W-
    {
    case 1:    /* U+ W- 05*//*//��ͨA+ C-,�ض�A- C+ B- B+*/
        PIN_Write(P20_8,0);   //�ض�A-//20.8//pwm6
        PIN_Write(P32_4,0);   //�ض�B-//32.4//PWM2
        ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0,MOTOR_FREQUENCY);  //�ض�B+//23.1//PWM1
        ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0,MOTOR_FREQUENCY);  //�ض�C+//21.2//pwm3
      /**********��ͨIO,��A- C+********************/
        ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, g_sBLDCMag.duty,MOTOR_FREQUENCY); //��A+//21.5//pwm5
        PIN_Write(P22_3,1);                             //��C-//22.3//pwm4
      break;

    case 2:     /* U- V+ 12*//*A- B+*/
      PIN_Write(P32_4,0);   //�ض�B-
      PIN_Write(P22_3,0);   //�ض�C-
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0,MOTOR_FREQUENCY);  //�ض�A+
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0,MOTOR_FREQUENCY);  //�ض�C+
      /**********��ͨIO,��A- C+********************/
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, g_sBLDCMag.duty,MOTOR_FREQUENCY); //��B+
      PIN_Write(P20_8,1);                             //��A-
      break;

    case 3:    /* V+ W- 25*//*B+ C-*/
      PIN_Write(P20_8,0);   //�ض�A-
      PIN_Write(P32_4,0);   //�ض�B-
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0,MOTOR_FREQUENCY);  //�ض�A+
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0,MOTOR_FREQUENCY);  //�ض�C+
      /**********��ͨIO,��A- C+********************/
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, g_sBLDCMag.duty,MOTOR_FREQUENCY); //��B+
      PIN_Write(P22_3,1);                             //��C-
      break;
    case 4:     /*  V- W+ 34*//*B- C+*/
      PIN_Write(P20_8,0);   //�ض�A-
      PIN_Write(P22_3,0);   //�ض�C-
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0,MOTOR_FREQUENCY);  //�ض�A+
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0,MOTOR_FREQUENCY);  //�ض�B+
      /**********��ͨIO,��A- C+********************/
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, g_sBLDCMag.duty,MOTOR_FREQUENCY); //��C+
      PIN_Write(P32_4,1);                             //��B-
      break;
    case 5:     /* U+  V- 03*//*A+ B-*/
      PIN_Write(P20_8,0);   //�ض�A-
      PIN_Write(P22_3,0);   //�ض�C-
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0,MOTOR_FREQUENCY);  //�ض�B+
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0,MOTOR_FREQUENCY);  //�ض�C+
      /**********��ͨIO,��A- C+********************/
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, g_sBLDCMag.duty,MOTOR_FREQUENCY); //��A+
      PIN_Write(P32_4,1);                             //��B-
      break;
    case 6:     /* U- W+ 14*//*A- C+*/
      PIN_Write(P22_3,0);   //�ض�C-
      PIN_Write(P32_4,0);   //�ض�B-
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0,MOTOR_FREQUENCY);  //�ض�B+
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0,MOTOR_FREQUENCY);  //�ض�A+
      /**********��ͨIO,��A- C+********************/
      ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, g_sBLDCMag.duty,MOTOR_FREQUENCY); //��C+
      PIN_Write(P20_8,1);                             //��A-


      break;

    default:
      LQ_BLDCStop();
      break;
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void LQ_BLDCReversal(void);
@����˵����������ˢ�����ת
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_BLDCReversal(void)
{
    //step=7-step;// ���ݻ����Ĺ��ɿ�֪�� REV = 7 - FWD;
    switch(g_sBLDCMag.step)
    { /*U- W+ 14*//*A- C+ */
      case 1:  //��ͨA- C+,�ض�A- C- A+ B+
          PIN_Write(P22_3,0);   //�ض�C-
          PIN_Write(P32_4,0);   //�ض�B-
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0,MOTOR_FREQUENCY);  //�ض�B+
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0,MOTOR_FREQUENCY);  //�ض�A+
          /**********��ͨIO,��A- C+********************/
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0-g_sBLDCMag.duty,MOTOR_FREQUENCY); //��C+
          PIN_Write(P20_8,1);                             //��A-
        break;
      case 2:    /* A+  B- 03*/
          PIN_Write(P20_8,0);   //�ض�A-
          PIN_Write(P22_3,0);   //�ض�C-
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0,MOTOR_FREQUENCY);  //�ض�B+
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0,MOTOR_FREQUENCY);  //�ض�C+
          /**********��ͨIO,��A- C+********************/
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0-g_sBLDCMag.duty,MOTOR_FREQUENCY); //��A+
          PIN_Write(P32_4,1);                             //��B-
        break;
      case 3:   /* V- W+ 34*//*B- C+*/
          PIN_Write(P20_8,0);   //�ض�A-
          PIN_Write(P22_3,0);   //�ض�C-
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0,MOTOR_FREQUENCY);  //�ض�A+
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0,MOTOR_FREQUENCY);  //�ض�B+
          /**********��ͨIO,��A- C+********************/
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0-g_sBLDCMag.duty,MOTOR_FREQUENCY); //��C+
          PIN_Write(P32_4,1);                             //��B-
        break;
      case 4:    /* V+ W- 25*//*B+ C-*/
          PIN_Write(P20_8,0);   //�ض�A-
          PIN_Write(P32_4,0);   //�ض�B-
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0,MOTOR_FREQUENCY);  //�ض�A+
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0,MOTOR_FREQUENCY);  //�ض�C+
          /**********��ͨIO,��A- C+********************/
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0-g_sBLDCMag.duty,MOTOR_FREQUENCY); //��B+
          PIN_Write(P22_3,1);                             //��C-
        break;
      case 5:    /* U- V+ 12*//*A- B+*/
          PIN_Write(P32_4,0);   //�ض�B-
          PIN_Write(P22_3,0);   //�ض�C-
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0,MOTOR_FREQUENCY);  //�ض�A+
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0,MOTOR_FREQUENCY);  //�ض�C+
          /**********��ͨIO,��A- C+********************/
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0-g_sBLDCMag.duty,MOTOR_FREQUENCY); //��B+
          PIN_Write(P20_8,1);                             //��A-
        break;
      case 6:    /* U+ W- 05*//*A+ C-*/
          PIN_Write(P20_8,0);   //�ض�A-//20.8//pwm6
          PIN_Write(P32_4,0);   //�ض�B-//32.4//PWM2
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0,MOTOR_FREQUENCY);  //�ض�B+//23.1//PWM1
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0,MOTOR_FREQUENCY);  //�ض�C+//21.2//pwm3
        /**********��ͨIO,��A- C+********************/
          ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0-g_sBLDCMag.duty,MOTOR_FREQUENCY); //��A+//21.5//pwm5
          PIN_Write(P22_3,1);                             //��C-//22.3//pwm4
        break;

      default:
        LQ_BLDCStop();
        break;
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void LQ_BLDCStop(void);
@����˵�����ر���ˢ�����ֹͣ����
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_BLDCStop(void)
{
//    PIN_Write(P22_3,0);   //�ض�C-
//    PIN_Write(P32_4,0);   //�ض�B-
//    ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0,MOTOR_FREQUENCY);  //�ض�B+
//    ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0,MOTOR_FREQUENCY);  //�ض�A+
//    /**********guanduan********************/
//    ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0,MOTOR_FREQUENCY); //�ض�C+
//    PIN_Write(P20_8,0);                             //�ض�A-
    ATOM_PWM_SetDuty(IfxGtm_ATOM0_6_TOUT42_P23_1_OUT, 0,MOTOR_FREQUENCY);   //�ض�C-
    ATOM_PWM_SetDuty(IfxGtm_ATOM0_5_TOUT40_P32_4_OUT, 0,MOTOR_FREQUENCY); //�ض�B-
    ATOM_PWM_SetDuty(IfxGtm_ATOM0_0_TOUT53_P21_2_OUT, 0,MOTOR_FREQUENCY);  //�ض�B+
    ATOM_PWM_SetDuty(IfxGtm_ATOM0_4_TOUT50_P22_3_OUT, 0,MOTOR_FREQUENCY);  //�ض�A+
    /**********��ͨIO,��A- C+********************/
    ATOM_PWM_SetDuty(IfxGtm_ATOM0_7_TOUT64_P20_8_OUT, 0,MOTOR_FREQUENCY); //��C+
    ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0,MOTOR_FREQUENCY);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�float LPF_velocity(float x);
@����˵����һ�׵�ͨ�˲���
@����˵�����������˲���ֵ
@�������أ������˲�����
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
float LPF_velocity(float x)
{
  float y = 0.8*y_vel_prev + 0.2*x;
  y_vel_prev=y;
  return y;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�int LQ_BLDCCurrent_detection(void);
@����˵����������⣬������·
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_BLDCCurrent_detection(void)
{
    g_sBLDCMag.counts ++;
    IA = (ADC_ReadAverage(ADC0,10)-2048)*3.3*100/4095/5/0.03;   //U    ����100�����㹫ʽ��(IA-2048)*3.3*100/4095/5/0.03��IA/2    //2048�ǲο���ѹ1.65
    IB = (ADC_ReadAverage(ADC1,10)-2048)*3.3*100/4095/5/0.03;  //V
    IC = (ADC_ReadAverage(ADC2,10)-2048)*3.3*100/4095/5/0.03;  //W    ����A������������ر������գ�B���֮��C����С��
    IA= LPF_velocity(IA);  //һ�׵�ͨ�˲�
    IB= LPF_velocity(IB);
    IC= LPF_velocity(IC);
    while((g_sBLDCMag.Electricity_flag ==TRUE)&&((IA/100>5.0)||(IB/100>5.0)||(IC/100>5.0)))
    {
      LQ_BLDCStop();
      if(KEY_Read(KEY1)==0){
        g_sBLDCMag.Electricity_flag =FALSE;
        break;
      }
    }
    if(g_sBLDCMag.counts > 1000)//����2s����Ϊ���������ϲ��ڼ����������
      g_sBLDCMag.counts=0,g_sBLDCMag.Electricity_flag =FALSE;
//      while((IA/100>2.5)&&(IB/100>2.5)&&(IC/100>2.5)&&(ECPULSE2==0))
//      {
//          LQ_BLDCStop();//��ת��ֹͣ���
//          if(KEY_Read(KEY1)==0)break;
//      }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void LQ_BLDCShow(void);
@����˵������ˢ����������չʾ
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_BLDCShow(void)
{
    char txt[16];
    float Bat = 0;
    if(KEY_Read(DSW0)) TFTSPI_P8X16Str(1, 2, "BEMF Mode", u16WHITE, u16BLACK);
    else               TFTSPI_P8X16Str(1, 2, "HALL Mode", u16WHITE, u16BLACK);
    sprintf(txt, "HALL:%02X", Get_Hall_State());
    TFTSPI_P8X16Str(1, 4, txt, u16WHITE, u16BLACK);
    sprintf(txt, "PWM: %05d;", BLDCduty);
    TFTSPI_P8X16Str(1, 5, txt, u16WHITE, u16BLACK);       //�ַ�����ʾ

    Bat = (float)ADC_ReadAverage(ADC7,10)*5*4/4096-0.45;  //��ص�ѹ��⣬
    sprintf(txt, "Bat:%.2f ",Bat);
    TFTSPI_P8X16Str(1, 3, txt, u16WHITE, u16RED);

    sprintf(txt, "I:%02d.%02d %02d.%02d %02d.%02d", IA/100,IA%100,IB/100,IB%100,IC/100,IC%100);
    TFTSPI_P8X16Str(1, 7, txt, u16WHITE, u16BLACK);

    sprintf(txt, "speed:%02d ",ECPULSE2);
    TFTSPI_P8X16Str(1, 1, txt, u16RED, u16BLUE);

    sprintf(txt, "Servo duty:%04d ", ServoDuty);
    TFTSPI_P8X16Str(1, 6, txt, u16BLACK, u16GREEN); //��ʾ����ʵ�����������Ա�������


}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void TestMotorBLDC (void);
@����˵������ˢ������Ժ���
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TestMotorBLDC (void)
{
    BLDC_HallInit();//��ʼ���������ţ���ȡ����ֵ�Լ����Եı���ֵ
    TFTSPI_CLS(u16BLUE);    //����
    TFTSPI_P8X16Str(2, 0, "LQ BLDC Test", u16RED, u16BLUE);
    BLDC_MotorInit(MOTOR_FREQUENCY-1);
    ADC_InitConfig(ADC7, 80000);//��ص�ѹ�ɼ�
    ADC_InitConfig(ADC0, 80000);
    ADC_InitConfig(ADC1, 80000);
    ADC_InitConfig(ADC2, 80000);
    CCU6_InitConfig(CCU60, CCU6_Channel0, 500000);  // 50ms����һ���ж�
    CCU6_InitConfig(CCU60, CCU6_Channel1, 20);  // 50ms����һ���ж�
    while(1){
//        ServoCtrl(ServoDuty);
        LQ_BLDCShow();
//        ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 4500, 12500);
        while ((KEY_Read(KEY0) == 0))      //����KEY0����ռ�ձȼ�С
        {
          if (BLDCduty > -MOTOR_FREQUENCY)
            BLDCduty -= 500;
          while ((KEY_Read(KEY0) == 0));
        }
        while((KEY_Read(KEY2) == 0))      //����KEY2����ռ�ձȼӴ�
        {
          if (BLDCduty < MOTOR_FREQUENCY)     //��ռ�ձ�Ϊ12500
            BLDCduty += 500;
          while((KEY_Read(KEY2) == 0));
        }
        while(KEY_Read(KEY1) == 0)      //����KEY1����ռ�ձ���ֵ
        {
          if(BLDCduty>0)BLDCduty = -3500;
          else BLDCduty = 3500;
          while(KEY_Read(KEY1) == 0);
        }
        LED_Ctrl(LED0, RVS);       //��ƽ��ת,LED��˸
    }
}









