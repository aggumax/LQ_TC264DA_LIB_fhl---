//-------------------------------------------------------------------------------------------------------------------
//  简介:八邻域图像处理  Version ：3.0
//------------------------------------------------------------------------------------------------------------------
#include "image_8.h"
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
/*
函数名称：int user_abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
修改时间：2022年9月8日
备    注：
example：  user_abs( x)；
 */
int user_abs(int value)
{
if(value>=0) return value;
else return -value;
}
/*
函数名称：int16 user_limit(int16 x, int16 y)
功能说明：求x,y中的最小值
参数说明：
函数返回：返回两值中的最小值
修改时间：2022年9月8日
备    注：
example：  user_limit( x,  y)
 */
int16 user_limit(int16 x, int a, int b)
{
    if(x<a) x = (uint16)a;
    if(x>b) x = (uint16)b;
    return x;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief     动态阈值
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 user_image_width  = col;
    uint16 user_image_height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};
    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;
    for (Y = 0; Y <user_image_height; Y++) //Y<user_image_height改为Y =user_image_height；以便进行 行二值化
    {
        //Y=user_image_height;
        for (X = 0; X < user_image_width; X++)
        {
        HistGram[(int)data[Y*user_image_width + X]]++; //统计每个灰度值的个数信息
        }
    }
    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值
    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }
    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      图像二值化，这里用的是大津法二值化。
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
void turn_to_bin(uint8(*mt9v03x_image)[user_image_w],uint8(*bin_image)[user_image_w],uint8 image_thereshold)
{
  uint8 i,j;
  for(i = 0;i<user_image_h;i++)
  {
      for(j = 0;j<user_image_w;j++)
      {
          if(mt9v03x_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
          else bin_image[i][j] = black_pixel;
      }
  }
}
/*
函数名称：uint8 get_start_point(uint8 start_row,uint8(*bin_image)[user_image_w])
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数，但是不能是最后一行
函数返回：无
修改时间：2022年9月8日
备    注：
example：  get_start_point(user_image_h-2,bin_image)
 */
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 l_border[user_image_h];//左线数组
uint8 r_border[user_image_h];//右线数组
uint8 center_line[user_image_h];//中线数组
uint8 get_start_point(uint8 start_row,uint8(*bin_image)[user_image_w])
{
    uint8 i = 0,l_found = 0,r_found = 0;
    //清零
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y
    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y
    //清零操作
    for (i = 0; i < user_image_h - 1; i++)
    {
        l_border[i] = 0;
        r_border[i] = 0;
        center_line[i] = 0;
    }
    //从中间往左边，先找起点
    for (i = user_image_w / 2; i > border_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            //printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_border[user_image_h - 1] = i;//找到的起点给最下面一行
            l_found = 1;
            break;
        }
    }
    for (i = user_image_w / 2; i < border_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
            //printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_border[user_image_h - 1]  = i;//找到的起点给最下面一行
            r_found = 1;
            break;
        }
    }
    if(l_found&&r_found)return 1;//因为边界涂了黑框，所以一般都是能找到起点的
    else {
        //printf("未找到起点\n");
        return 0;
    }
}
/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[user_image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)
功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r            ：最多需要循环的次数
(*image)[user_image_w]      ：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                       特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic              ：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic              ：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x               ：左边起点横坐标
l_start_y               ：左边起点纵坐标
r_start_x               ：右边起点横坐标
r_start_y               ：右边起点纵坐标
hightest                ：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num user_image_h*3  //定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
void search_l_r(uint16 break_flag, uint8(*image)[user_image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

    uint8 i = 0, j = 0;

    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
     static int8 seeds_l[8][2] = { {-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1, 0}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {1,-1},{0,-1},{-1,-1}, {-1,0},{-1,1},{0,1}, {1,1},{1, 0}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

    //更新版，一边找，一边记录左右边界
     uint8 l_border_index = user_image_h - 2;//从倒数第二行开始
     uint8 r_border_index = user_image_h - 2;//从倒数第二行开始

    l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y
        //开启邻域循环
    while (break_flag--)
    {
        //左边
        for (i = 0; i < 8; i++)//传递八领域坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        //提取左边界
        if (l_border_index== points_l[l_data_statics][1])
        {
            l_border[l_border_index] = (uint8)points_l[l_data_statics][0];
            l_border_index--;
        }
        l_data_statics++;//索引加一
        //右边
        for (i = 0; i < 8; i++)//传递八领域坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y
        //提取右边界
        if (r_border_index == points_r[r_data_statics][1])
        {
            r_border[r_border_index] = (uint8)points_r[r_data_statics][0];
            r_border_index--;
        }
        index_l = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用
        }
        //左边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i);//记录生长方向
            }
            if (index_l)
            {
                //更新坐标点
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }
        }
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("三次进入同一个点，退出\n");
            break;
        }
        if (user_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && user_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1]) < 2)
        {
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一
        index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }
        //右边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//索引加一
                dir_r[r_data_statics - 1] = (i);//记录生长方向
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {
                //更新坐标点
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }
            }
        }
    }
    //取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}
//定义膨胀和腐蚀的阈值区间
#define threshold_max   255*5//此参数可根据自己的需求调节
#define threshold_min   255*2//此参数可根据自己的需求调节
void image_filter(uint8(*bin_image)[user_image_w])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;
    for (i = 1; i < user_image_h - 1; i++)
    {
        for (j = 1; j < (user_image_w - 1); j++)
        {
            //统计八个方向的像素值
            num =
                bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
                + bin_image[i][j - 1] + bin_image[i][j + 1]
                + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];
            if (num >= threshold_max && bin_image[i][j] == 0)
            {
                bin_image[i][j] = 255;//白  可以搞成宏定义，方便更改
            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {
                bin_image[i][j] = 0;//黑
            }
        }
    }
}
/*
函数名称：void image_draw_rectan(uint8(*image)[user_image_w])
功能说明：给图像画一个黑框
参数说明：uint8(*image)[user_image_w]    图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8(*image)[user_image_w])
{
    uint8 i = 0;
    for (i = 0; i < user_image_h; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][user_image_w - 1] = 0;
        image[i][user_image_w - 2] = 0;
    }
    for (i = 0; i < user_image_w; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
    }
}
/*
函数名称：void image_process(void)
功能说明：最终处理函数
参数说明：无
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_process(mt9v03x_image);
 */
uint8 bin_image[user_image_h][user_image_w];//二值化图像数组
void image_process(uint8(*mt9v03x_image)[user_image_w])
{
uint16 i;
uint8 hightest = 0;//定义一个最高行，tip：这里的最高指的是y值的最小
uint8 image_thereshold = 0;//图像分割阈值
//获取动态阈值
 image_thereshold = otsuThreshold(mt9v03x_image[0], user_image_w, user_image_h);
//语义分割
turn_to_bin(mt9v03x_image,bin_image,image_thereshold);
/*提取赛道边界*/
image_filter(bin_image);//滤波
image_draw_rectan(bin_image);//预处理
//清零
data_stastics_l = 0;
data_stastics_r = 0;
if (get_start_point(user_image_h-2,bin_image))//找到起点了，再执行八领域，没找到就一直找
{
    //八邻域巡线，直接提取轮廓和边界一次性完成
    search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
    //处理函数放这里，不要放到if外面去了，不要放到if外面去了，不要放到if外面去了，重要的事说三遍

}
//显示图像   改成你自己的就行 等后期足够自信了，显示关掉，显示屏挺占资源的
TFTSPI_Road(0, 2, LCDH, LCDW, (unsigned char *)bin_image[0]);
//    //根据最终循环次数画出边界点
//    for (i = 0; i < data_stastics_l; i++)
//    {
//        //ips154_drawpoint(points_l[i][0]+2, points_l[i][1], uesr_BLUE);//显示起点
//    }
//    for (i = 0; i < data_stastics_r; i++)
//    {
//        //ips154_drawpoint(points_r[i][0]-2, points_r[i][1], uesr_RED);//显示起点
//    }
    for (i = hightest; i < user_image_h-1; i++)
    {
        center_line[i] = (l_border[i] + r_border[i]) >> 1;//求中线
        //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
        //当然也有多组边线的找法，但是个人感觉很繁琐，不建议
        TFTSPI_Draw_Dot(center_line[i], i, uesr_GREEN);//显示起点 显示中线
        TFTSPI_Draw_Dot(l_border[i], i, uesr_BLUE);//显示起点 显示左边线
        TFTSPI_Draw_Dot(r_border[i], i, uesr_RED);//显示起点 显示右边线
    }
}
/*
这里是起点（0.0）***************——>********************x值最大
************************************************************
************************************************************
************************************************************
************************************************************
******************假如这是一副图像*************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
y值最大************************************************(188.120)
*/
