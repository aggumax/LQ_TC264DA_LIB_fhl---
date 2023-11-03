/*
 * 风火轮组改动代码
 */
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

App_Cpu0 g_AppCpu0;                       // brief CPU 0 global data
IfxCpu_mutexLock mutexCpu0InitIsOk = 1;   // CPU0 初始化完成标志位
volatile char mutexCpu0TFTIsOk=0;         // CPU1 0占用/1释放 TFT
int Vat=0;
int Vbat=0;
extern sint16 TempAngle;
/*************************************************************************
*  功能说明：CPU0主函数
*  修改时间：2023年10月26日
*************************************************************************/
int core0_main (void)
{
    char txt[16];
//    unsigned char  cnt=0;

    // 关闭CPU总中断
	IfxCpu_disableInterrupts();

	// 关闭看门狗，如果不设置看门狗喂狗需要关闭
	IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
	IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

	// 读取总线频率
	g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
	g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
	g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
	g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);


    TFTSPI_Init(1);
    TFTSPI_CLS(u16BLACK);
//    TFTSPI_Show_Logo(0,37);       /
    TFTSPI_P8X16Str(3,4,"FengHuoLun",u16WHITE,u16BLACK);
    // 按键初始化
	GPIO_KEY_Init();
	// LED灯所用P10.6和P10.5初始化
	GPIO_LED_Init();
	BLDC_HallInit();//初始化霍尔引脚及编码器接口ֵ
    BLDC_MotorInit(MOTOR_FREQUENCY-1);
    MotorInit();
    ADC_InitConfig(ADC7, 80000);//ADC初始化函数
    ADC_InitConfig(ADC0, 80000);
    ADC_InitConfig(ADC1, 80000);
    ADC_InitConfig(ADC2, 80000);

    IIC_Init();                  // IIC初始化
    delayms(100);
    Gyro_Chose();          // 读取陀螺仪的设备ID
    MPU6050_Init();              // MPU6050初始化
    LQ_DMP_Init();               //MPU6050的DMP初始化
    EncInit();                  //编码器初始化
    ServoInit();                //舵机PWM初始化
    /* ����ͷ��ʼ�� */
    //CAMERA_Init(50);
//    CCU6_InitConfig(CCU60, CCU6_Channel0, 500000);  // 调电机的时候注释掉
//    CCU6_InitConfig(CCU60, CCU6_Channel1, 20);      // ��ˢ�������
//    CCU6_InitConfig(CCU61, CCU6_Channel0, 5000);    // ��������
    // 串口P14.0管脚输出,P14.1输入，波特率115200
	UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 115200);
	TFTSPI_CLS(u16BLACK);         //清屏
	// 开启CPU总中断
	IfxCpu_enableInterrupts();

	// 通知CPU1，CPU0初始化完成
	IfxCpu_releaseMutex(&mutexCpu0InitIsOk);
	// 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示ʾ
	mutexCpu0TFTIsOk=0;         // CPU1： 0占用/1释放 TFT

//	LQ_GPT_4mini512TFT();  //读取并显示编码器的值
	Text_PID();
//  LQ_Atom_Motor_8chPWM();
//	LQ_ATom_Servo_2chPWM();
//	BD1202_test();

	/********************************************************************************

无刷电机接线：
      母板接驱动板：
              PWM :P23.1->pwm1  P32.4->pwm2  P21.2->pwm3  22.3->pwm4  P21.5->pwm5  P20.8->pwm6

      驱动板接电机(驱动板：LQ2304/2136  电机：LQ2830无刷电机)：
                  A->黄线  B->黑线  C->红线

      霍尔接线: P13_0 -> HA（绿色数据线）
               P10_3 -> HB（棕色数据线）
               P10_1 -> HC（橙色数据线）
	*******************************************************************************/

	//  TestMotorBLDC();
//	Balance();



    while (1)	//��ѭ��
    {
        if(Camera_Flag == 2)//摄像头函数
          {
              // ��ȡ����ʹ�õ�����
              Get_Use_Image();
              //�������ͷ�ɼ���ɱ�־λ  �����������򲻻��ٴβɼ�����
              Camera_Flag = 0;
              Get_Bin_Image(0);
              TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) Bin_Image);
          }

        TFTSPI_P8X16Str(0, 0, "GUMAX_DMP", u16BLACK, u16YELLOW);

//        Vbat = ADC_Read(ADC7);  // ����Դ��ѹ
//        Vat = Vbat * 11 / 25;   // ��ѹת��\

        sprintf((char*)txt,"Yaw:%.02f",Yaw);//偏航角
        TFTSPI_P8X16Str(0,2,txt,u16WHITE,u16BLACK);//
        sprintf((char*)txt,"Roll:%.02f",Roll);//俯仰角
        TFTSPI_P8X16Str(0,3,txt,u16WHITE,u16BLACK);
        sprintf((char*)txt,"Pitch:%.02f",Pitch);//倾斜角

        TFTSPI_P8X16Str(0,4,txt,u16WHITE,u16BLACK);
        printf("%.2f,%.2f,%.2f\n", Roll,Pitch,Yaw);
//        UART_PutStr(UART0, &Yaw);

    }
}


//        sprintf((char*)txt,"T:%06d",TempAngle);
//        TFTSPI_P8X16Str(0,2,txt,u16RED,u16BLACK);// �ַ�����ʾ
//        sprintf((char*)txt,"E2:%05d M2:%05d",ECPULSE2,MotorDutyB);
//        TFTSPI_P8X16Str(0,3,txt,u16RED,u16BLACK);// �ַ�����ʾ
//        sprintf((char*)txt,"%06d",gyro[0]);
//        TFTSPI_P8X16Str(0,4,txt,u16RED,u16BLACK);// �ַ�����ʾ
//        sprintf((char*)txt,"A %06d",PWM_X+PWM_accel);
//        TFTSPI_P8X16Str(0,6,txt,u16RED,u16BLACK);// �ַ�����ʾ
//        sprintf(txt, "Bat:%d.%02dV V:%04d", Vat / 100, Vat % 100,Vbat);  // ��ʾ��ص�ѹ
//        TFTSPI_P8X16Str(0,5,txt,u16WHITE,u16BLACK);// �ַ�����ʾ
//        sprintf(txt, "I:%02d.%02d %02d.%02d %02d.%02d", IA/100,IA%100,IB/100,IB%100,IC/100,IC%100);
//        TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);

