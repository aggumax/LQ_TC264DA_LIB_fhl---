#ifndef __MYCODE_
#define __MYCODE_

void Balance_FHL_Bingji(void);
void Balance_FHL_Chuangji(void);

unsigned short Balance_PID_CJ(float qiwan_Angle, float shiji_Angle);
unsigned short Balance_PID_CJJD(float qiwan, float shiji);
unsigned short Balance_PID_CJSD(float qiwan, float shiji);
int Down_flag();

void Motor_konzhi(unsigned short motor);
void Servo_konzhi(unsigned short panduan);
void Balance_DJ(void);

#endif
