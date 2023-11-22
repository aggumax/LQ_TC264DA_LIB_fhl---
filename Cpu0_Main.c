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
#include "LQ_GTM.h"
#include "LQ_GPT12_ENC.h"

#include "Mycode.h"
#include "Balance2.h"
#include "image_8.h"

App_Cpu0 g_AppCpu0;                       // brief CPU 0 global data
IfxCpu_mutexLock mutexCpu0InitIsOk = 1;   // CPU0 初始化完成标志位
volatile char mutexCpu0TFTIsOk=0;         // CPU1 0占用/1释放 TFT
int Vat=0;
int Vbat=0;
extern sint16 TempAngle;
int kaishi=0;
/*************************************************************************
*  功能说明：CPU0主函数
*  修改时间：2023年11月4日
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
    delayms(20);
    // 按键初始化
	GPIO_KEY_Init();
	// LED灯所用P10.6和P10.5初始化
	GPIO_LED_Init();
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
    // 串口P14.0管脚输出,P14.1输入，波特率115200
	UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 115200);
	TFTSPI_CLS(u16BLACK);         //清屏
	// 开启CPU总中断
	IfxCpu_enableInterrupts();

	// 通知CPU1，CPU0初始化完成
	IfxCpu_releaseMutex(&mutexCpu0InitIsOk);
	// 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示ʾ
	mutexCpu0TFTIsOk=0;         // CPU1： 0占用/1释放 TFT
    /*中断函数*/
//    CCU6_InitConfig(CCU61, CCU6_Channel0,5000);//5ms
//    ATOM_PWM_InitConfig(ATOMPWM2, 5000, 12500);
//    ATOM_PWM_InitConfig(ATOMSERVO1, 1950, 100);//舵机频率为100HZ，初始值为1.5ms中值

//	Text_PID();
//  LQ_Atom_Motor_8chPWM();
    // LQ_ATom_Servo_2chPWM();
	Test_CAMERA();
//	CAMERA_Init(50);
    while (1)	//主循环
    {
//        TFTSPI_P8X16Str(3, 0, "GUMAX_FHL", u16BLACK, u16YELLOW);
        LQ_DMP_Read();
//        PINGHENG_BJ(Pitch);
//        Balance();
//        if(KEY_Read(KEY0)==0) kaishi=1;
//        if(KEY_Read(KEY1)==0) kaishi=0;
//        if(kaishi)Balance_FHL_Chuangji();
//        FHL_servo();
//        sprintf((char*)txt,"Yaw:%.02f",Yaw);//偏航角
//        TFTSPI_P8X16Str(0,1,txt,u16BLACK,u16WHITE);//
//        sprintf((char*)txt,"MotorDuty111:%.02f",MotorDuty111);//俯仰角
//        TFTSPI_P8X16Str(0,5,txt,u16BLACK,u16WHITE);
//        sprintf((char*)txt,"Pitch:%.02f",Pitch);//倾斜角
//        TFTSPI_P8X16Str(0,1,txt,u16WHITE,u16BLACK);
//        sprintf((char*)txt,"enc:%05d",enc_222);//倾斜角
//        TFTSPI_P8X16Str(0,3,txt,u16WHITE,u16BLACK);
//        float GG =gyro[0];
//        sprintf((char*)txt,"gyro:%.02f",GG);//
//        TFTSPI_P8X16Str(0,2,txt,u16WHITE,u16BLACK);


//        TFTSPI_P8X16Str(12,9,"@FHL",u16WHITE,u16BLACK);
//        image_process();
    }
}


