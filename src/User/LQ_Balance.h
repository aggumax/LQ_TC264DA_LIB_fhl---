/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】ZYF/chiusir
【E-mail  】chiusir@163.com
【软件版本】V1.1 版权所有，单位使用请先联系授权
【最后更新】2020年10月28日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】AURIX Development Studio1.2.2及以上版本
【Target 】 TC264DA/TC264D
【Crystal】 20.000Mhz
【SYS PLL】 200MHz
________________________________________________________________
基于iLLD_1_0_1_11_0底层程序,

使用例程的时候，建议采用没有空格的英文路径，
除了CIF为TC264DA独有外，其它的代码兼容TC264D
本库默认初始化了EMEM：512K，如果用户使用TC264D，注释掉EMEM_InitConfig()初始化函数。
工程下\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c第164行左右。
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef SRC_APPSW_TRICORE_MAIN_LQ_BALANCE_H_
#define SRC_APPSW_TRICORE_MAIN_LQ_BALANCE_H_
extern int Vat;
extern int Vbat;
extern short MotorDutyB;      //电机驱动占空比数值
extern float Zero_error;
extern float Pitch_Zero,Pitch_error;//设置Pitch轴角度零点
extern int PWM_X,PWM_accel;                  // PWM中间量
extern short MotorDutyA;                // 飞轮电机驱动占空比数值
extern volatile char mutexCpu0TFTIsOk;         // CPU1 0占用/1释放 TFT
extern int Target_Speed;
void Balance(void);
float yijielvbo_P(float angle_m,float gyro_m);
/**************************************************************************
Y轴平衡PID控制,角度环
**************************************************************************/
float X_balance_Control(float Angle,float Angle_Zero,float Gyro);
/**************************************************************************
速度PI控制,速度正反馈环
**************************************************************************/
float Velocity_Control(int encoder);
int SBB_Get_BalancePID(float Angle,float Gyro);
int SBB_Get_MotorPI (int Encoder,int Target);
int Velocity_Momentum(int Target,int encoder);

#endif /* SRC_APPSW_TRICORE_MAIN_LQ_SMARTCAR_H_ */
