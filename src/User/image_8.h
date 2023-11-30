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
////��������������������ֲ������ֲ��ʱ�����ɾ�����ĳ����Լ��ģ�
typedef   signed          char int8;
typedef   signed short     int int16;
typedef   signed           int int32;
//typedef unsigned          char uint8;
//typedef unsigned short     int uint16;
//typedef unsigned           int uint32;

//��ɫ����  ��Ϊ������������ɫ���Ĳ�������ֱ�ӷ�����
#define uesr_RED     0XF800    //��ɫ
#define uesr_GREEN   0X07E0    //��ɫ
#define uesr_BLUE    0X001F    //��ɫ

//�궨��
#define user_image_h    60//ͼ��߶�  60
#define user_image_w    120//ͼ���� 100
//�׵�ڵ㶨��
#define white_pixel 255 //�׵�
#define black_pixel 0   //�ڵ�

#define bin_jump_num    1//�����ĵ���
#define border_max  user_image_w-2 //�߽����ֵ
#define border_min  1   //�߽���Сֵ


extern void image_process(uint8(*mt9v03x_image)[user_image_w]); //ֱ�����жϻ�ѭ������ô˳���Ϳ���ѭ��ִ����

#endif /*_IMAGE_H*/

