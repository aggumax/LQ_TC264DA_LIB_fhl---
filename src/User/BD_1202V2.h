/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】chiusir
【E-mail】chiusir@163.com
【软件版本】V1.2 版权所有，单位使用请先联系授权
【最后更新】2023年6月6日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】AURIX Development Studio1.6版本
【Target 】 TC264DA/TC264D
【Crystal】 20.000Mhz
【SYS PLL】 200MHz
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#ifndef __LQ_GPS_BD_H
#define __LQ_GPS_BD_H

#define true 1
#define false 0

typedef struct SaveData
{
  char GPS_Buffer[128];         //完整数据
  char isGetData;                   //是否获取到GPS数据
  char isParseData;             //是否解析完成
  char UTCTime[11];               //UTC时间
  char latitude[11];                //纬度
  char N_S[2];                    //N/S
  char longitude[12];               //经度
  char E_W[2];                    //E/W
  char isUsefull;                   //定位信息是否有效
} _SaveData;

typedef struct UseData  //实际的差值
{
  int lat_err;          //纬度差
  int lon_err;          //经度差
  int lat_centre;       //弧形赛道中心点纬度
  int lon_centre;       //弧形赛道中心点经度
  float radius;         //圆弧的半径
  float distan_next;    //与下一个目标点的距离
  float distan_centre;  //与圆心的距离
  float lat_cent_err;   //与中心的纬度差
  float lon_cent_err;   //与中心的经度差
  float lat_next_err;   //下一个点的纬度差
  float lon_next_err;   //下一个点的经度差

} _UseData;

extern _SaveData Save_Data;

void parseGpsBuffer(void);
void BD1202_test(void);


#endif
