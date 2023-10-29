#include "include.h"
#include "stdio.h"
#include "string.h"
#include "BD_1202V2.h"

#include "LQ_UART.h"
#include "LQ_TFT18.h"
#include "LQ_GPIO_LED.h"
#include "LQ_STM.h"

_SaveData Save_Data;
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void parseGpsBuffer(void);
@功能说明：数据解析
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@备    注：将串口收到的数据分门别类的保存到结构体中
@           $GNRMC,130416.000,A,2236.91843,N,11359.19128,E,0.001,306.28,090720,,,A*4F
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void parseGpsBuffer(void)
{
  char *subString;
  char *subStringNext;
  char i = 0;
  if (Save_Data.isGetData)      //如果字符串不为空
  {
    for (i = 0 ; i <= 6 ; i++)  //循环7次
    {
      if (i == 0)
      {
        if ((subString = strchr(Save_Data.GPS_Buffer, ',')) == NULL)  //没有检测到逗号
          while(1);             //解析错误
      }
      else  //检测到逗号，返回逗号的位置
      {
        subString++;    //位置加1（定位到逗号的后一位）
        if ((subStringNext = strchr(subString, ',')) != NULL)//定位下一个逗号的位置
        {
          char usefullBuffer[2];
          switch(i)
          {
          case 1:memcpy(Save_Data.UTCTime, subString, subStringNext - subString);       //两个逗号之间为 时间信息 并转换成北京时间
          Save_Data.UTCTime[1] = Save_Data.UTCTime[1]+ 8;
          if(Save_Data.UTCTime[1] > '9')
          {
            Save_Data.UTCTime[0]++;
            if(Save_Data.UTCTime[0] == '3')
              Save_Data.UTCTime[0] = '0';
            Save_Data.UTCTime[1] = (Save_Data.UTCTime[1] % '9') + '0'-1;
          }
          break;//结束switch
          case 2:memcpy(usefullBuffer,            subString, subStringNext - subString);break; //数据是否有效标志
          case 3:memcpy(Save_Data.latitude,       subString, subStringNext - subString);break;  //获取纬度信息
          case 4:memcpy(Save_Data.N_S,            subString, subStringNext - subString);break;  //获取N/S
          case 5:memcpy(Save_Data.longitude,      subString, subStringNext - subString);break;  //获取经度信息
          case 6:memcpy(Save_Data.E_W,            subString, subStringNext - subString);break;  //获取E/W
          default:break;
          }
          subString = subStringNext;    //下一个逗号位置给第一个指针，
          Save_Data.isParseData = true; //手动给真值，（数据是否解析完成）
          //          if(usefullBuffer[0] == 'A')
          //            Save_Data.isUsefull = true;
          //          else if(usefullBuffer[0] == 'V')
          //            Save_Data.isUsefull = false;
        }
        else
        {
          while(1); //解析错误
        }
      }
    }
  }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：char Get_InitData(int* Lon_Z, int* Lon_X, int* Lat_Z, int* Lat_X);
@功能说明：经纬度原始值
@参数说明：Lon_Z（经度整数），Lon_X（经度小数），Lat_Z（纬度整数）,Lat_X（纬度小数）
@函数返回：无
@修改时间：2022/02/24
@备    注：将结构体中的字符串转换成数据，有两种方式，#if 1：以小数点为分界。#if 0：自己手动调节
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

char Get_InitData(int* Lon_Z, int* Lon_X, int* Lat_Z, int* Lat_X)
{
#if 0
  //判断纬度值中是否有‘.'有:说明有数据 例如：3946.99715  变化范围在后六位中变化，最后一位不稳定可舍去。取值为3946和99715
  if((strstr(Save_Data.latitude, ".")) != NULL)
  {
    sscanf(Save_Data.latitude, "%d.%d",&(* Lat_Z), &(* Lat_X));
  }
  else
    return 0;
  //经度原理同上 例如：11628.32198  取值为11628 和 32198
  if((strstr(Save_Data.longitude, ".")) != NULL)
  {
    sscanf(Save_Data.longitude, "%d.%d",&(* Lon_Z), &(* Lon_X));
  }
  else
    return 0;
  return 1;
#else
  unsigned num=0;
  //判断纬度值中是否有‘.'有:说明有数据 例如：3946.99715  变化范围在后六位中变化，最后一位不稳定可舍去。
  if((strstr(Save_Data.latitude, ".")) != NULL)
  {
    *Lat_Z = (Save_Data.latitude[0] - '0')*100 + (Save_Data.latitude[1] - '0')*10 + (Save_Data.latitude[2] - '0');
    num    = (Save_Data.latitude[3] - '0')*100000 + (Save_Data.latitude[5] - '0')*10000 + (Save_Data.latitude[6] - '0')*1000 + \
             (Save_Data.latitude[7] - '0')*100 + (Save_Data.latitude[8] - '0')*10 + (Save_Data.latitude[9] - '0');
    *Lat_X = num;       //取值为394 和 699715
  }
  else
    return 0;
  //经度原理同上 例如：11628.32198
  if((strstr(Save_Data.longitude, ".")) != NULL)
  {
    *Lon_Z = (Save_Data.longitude[0] - '0')*1000 + (Save_Data.longitude[1] - '0')*100 + \
             (Save_Data.longitude[2] - '0')*10 + (Save_Data.longitude[3] - '0');
    num    = (Save_Data.longitude[4] - '0')*100000 + (Save_Data.longitude[6] - '0')*10000 + (Save_Data.longitude[7] - '0')*1000 + \
             (Save_Data.longitude[8] - '0')*100 + (Save_Data.longitude[9] - '0')*10 + (Save_Data.longitude[10] - '0');
    *Lon_X = num;       //取值为1162 和 832198
  }
  else
    return 0;
  return 1;
#endif
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void BD1202_test(void);
@功能说明：北斗1202测试函数
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@备    注：
注意再移植的过程中需要配置工程文件，不然浮点型数据精度不够!!!!!!
1.右击工程->Properties
2.选择：C/C++ Build  ->  Settings  ->  TASKING C/C++ Compiler  ->  Fliating-Point
3.在右侧框框中：“Floating-point model:”选择“1 - Precise”
4.单击“Apply”应用
5.选择：C/C++ Build  ->  Settings  ->  TASKING Linker
6.在右侧框框中：“Command line pattern:”中修改“-lcs_fpu”为“-lc_fpu”
7.单击“Apply and Close”应用并关闭
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void BD1202_test(void)
{
  char txt[30];
//  int Lon_IZ=0, Lon_IX=0, Lat_IZ=0, Lat_IX=0;
  double Lon=0.0, Lat=0.0, tim=0.0;

  TFTSPI_Init(1);        //TFT初始化  0:横屏  1：竖屏
  TFTSPI_CLS(u16BLUE);   //蓝色屏幕

  //串口初始化为
  UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 115200);
  //灯初始化
  GPIO_LED_Init();
  sprintf(txt, "BD Test");
  TFTSPI_P8X16Str(0,0,txt,u16WHITE,u16BLACK);// 字符串显示
  while(1)
  {
#if 1
      /*解析方法1：简单粗暴 （还挺好用的,获取到的数据直接就是double型数据，数据需要除以100）*/
      if (Save_Data.isGetData)      //如果字符串不为空
      {
          sscanf(Save_Data.GPS_Buffer, "$BDRMC,%lf,A,%lf,N,%lf,E,", &tim, &Lat, &Lon);

          sprintf(txt, "Parsing BeiDou");
          TFTSPI_P8X16Str(0,1,txt,u16RED,u16YELLOW);// 字符串显示

      }
      sprintf(txt, "Type:double");
      TFTSPI_P8X16Str(0,3,txt, u16WHITE,u16BLACK);          //时间
      sprintf(txt, "T:%f",tim);
      TFTSPI_P8X16Str(0,4,txt, u16GREEN,u16BLACK);          //时间
      sprintf(txt, "N:%f",Lat/100);             // 转化成数据
      TFTSPI_P8X16Str(0,5,txt, u16YELLOW,u16BLACK);
      sprintf(txt, "E:%f",Lon/100);             // 数据
      TFTSPI_P8X16Str(0,6,txt, u16YELLOW,u16BLACK);
      LED_Ctrl(LED2,RVS);// LED翻转闪烁
      delayms(200);
#else

    /*解析方法2：比较通用*/
    parseGpsBuffer();

    sprintf(txt, "T:%s",Save_Data.UTCTime);             //时间
    TFTSPI_P8X16Str(0,2,txt, u16RED,u16BLACK);
    sprintf(txt, "N:%s",Save_Data.latitude);            // 纬度原始值
    TFTSPI_P8X16Str(0,3,txt, u16RED,u16BLACK);
    sprintf(txt, "E:%s",Save_Data.longitude);           // 经度原始值
    TFTSPI_P8X16Str(0,4,txt, u16RED,u16BLACK);
    printf("N:%s\nE:%s\n",Save_Data.latitude, Save_Data.longitude);


    if(Get_InitData(&Lon_IZ, &Lon_IX, &Lat_IZ, &Lat_IX) == 0) //获取原始经纬度值
    {
      Lon_IZ=0;Lon_IX=0;Lat_IZ=0;Lat_IX=0;
    }
    sprintf(txt, "N:%d.%d",Lat_IZ, Lat_IX);             // 转化成数据
    TFTSPI_P8X16Str(0,6,txt, u16RED,u16BLACK);
    sprintf(txt, "E:%d.%d",Lon_IZ, Lon_IX);             // 数据
    TFTSPI_P8X16Str(0,7,txt, u16RED,u16BLACK);
    LED_Ctrl(LEDALL,RVS);// LED翻转闪烁
    delayms(200);
#endif
  }
}



