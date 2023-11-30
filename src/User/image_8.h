#ifndef _IMAGE_8_H
#define _IMAGE_8_H
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
////数据类型声明（方便移植——移植的时候可以删掉，改成你自己的）
typedef   signed          char int8;
typedef   signed short     int int16;
typedef   signed           int int32;
//typedef unsigned          char uint8;
//typedef unsigned short     int uint16;
//typedef unsigned           int uint32;

//颜色定义  因为有先例，连颜色都改不来，我直接放这了
#define uesr_RED     0XF800    //红色
#define uesr_GREEN   0X07E0    //绿色
#define uesr_BLUE    0X001F    //蓝色

//宏定义
#define user_image_h    60//图像高度  60
#define user_image_w    120//图像宽度 100
//白点黑点定义
#define white_pixel 255 //白点
#define black_pixel 0   //黑点

#define bin_jump_num    1//跳过的点数
#define border_max  user_image_w-2 //边界最大值
#define border_min  1   //边界最小值


extern void image_process(uint8(*mt9v03x_image)[user_image_w]); //直接在中断或循环里调用此程序就可以循环执行了

#endif /*_IMAGE_H*/

