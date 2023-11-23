/***********八邻域图像处理************/

#include "image_8.h"

#include "Platform_Types.h"

int my_ads(int value)//求平均值
{
    if(value >= 0) return value;
    else return -value;
}

sint16 limit_a_b(sint16 x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

sint16 limit(sint16 x, sint16 y)
{
    if(x > y)       return y;
    else if(x < -y) return -y;
    else            return x;
}

/**********获得一副灰度图像**********/
//void Get_Use_Image(void)   这个函数
uint8 original_image[image_h][image_w];
void Get_image(uint8(*Image_Data)[image_w])
{
#define use_num         1
    uint8 i = 0,j=0,row=0,line=0;
    for(i=0; i<image_h; i+=use_num)
    {
        for(j=0; j<image_w; j+=use_num)
        {
            original_image[row][line] = Image_Data[i][j];
            line++;
        }
        line=0;
        row++;
    }
}

/**********动态阈值**********/
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Y; int X;
    uint16 Image_Heigh = row;
    uint16 Image_Width = col;
    uint8* data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;    //像素总数
    uint8 MinValue=0, MaxValue=0;
    uint32 Pixelintegral = 0;
    uint32 PixelBack = 0;
    uint32 PixelintegralBack = 0;
    sint32 PixelintegralFore = 0;
    sint32 PixelFore = 0;
    uint8 Threshold=0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0;// 类间方差

    for(Y = 0; Y < Image_Heigh; Y++)
    {
        for(X=0; X < Image_Width; X++)
        {
            HistGram[(int)data[Y*Image_Width + X]]++;
        }
    }


    for(MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++);       //获取最小灰度的值
    for(MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--);//获取最大灰度的值

    if(MaxValue == MinValue)
    {
        return MaxValue;   // 图像中只有一个颜色
    }
    if(MinValue + 1 == MaxValue)
    {
        return MinValue;  // 图像中只有二个颜色
    }

    for(Y = MinValue; Y <=MaxValue; Y++)
    {
        Amount += HistGram[Y]; //像素总数
    }

    Pixelintegral = 0;
    for(Y = MinValue; Y <= MaxValue; Y++)
    {
        Pixelintegral += HistGram[Y]*Y;//灰度值总数
    }
    SigmaB = -1;
    for(Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack = PixelBack + HistGram[Y];//前景像素点数
        PixelFore = Amount - PixelBack;//背景像素点数
        OmegaBack = (double)PixelBack / Amount; //前景像素百分比
        OmegaFore = (double)PixelFore / Amount; //背景像素百分比
        PixelintegralBack += HistGram[Y]*Y;     //前景灰度值
        PixelintegralFore = Pixelintegral - PixelintegralBack;//背景灰度值
        MicroBack = (double)PixelintegralBack / PixelBack;//前景灰度百分比
        MicroFore = (double)PixelintegralFore / PixelFore;//背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if(Sigma > SigmaB)
        {
            SigmaB = Sigma;
            Threshold = (uint8)Y;
        }
    }

    return Threshold;
}

/**********图像二值化**********/
//这里是大津法
uint8 bin_image[image_h][image_w];//图像数组
uint8 image_thereshold;
void turn_to_bin(void)
{
    uint8 i,j;
    image_thereshold = otsuThreshold(original_image[0], image_w, image_h);
    for(i = 0; i<image_h;i++)
    {
        for(j = 0; j<image_w; j++)
        {
               if(original_image[i][j] > image_thereshold)bin_image[i][j] = white_pixel;
                else bin_image[i][j] = black_pixel;
        }
    }

}

/**********寻找两个边界点作为八领域循环的起始点***********/
uint8 start_point_l[2] = {0};//左边起点的x,y值
uint8 start_point_r[2] = {0};//右边起点的x,y值
uint8 get_startpoint(uint8 start_row)
{
    uint8 i=0, l_found=0, r_found=0;
    //先清零
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

    for(i = image_w /2; i > borde_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if(bin_image[start_row][i] == 255 && bin_image[start_row][i-1] == 0)
        {
            l_found = 1;//找到左边起点
            break;
        }
    }
    for(i = image_w / 2; i < borde_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if(bin_image[start_row][i] == 255 && bin_image[start_row][i+1] == 0)
        {
            r_found = 1;//找到右边起点
            break;
        }
    }
    
    if(l_found && r_found) return 1;
    else return 0;
}

#define USE_num  image_h*3

//存放点点x,y坐标
uint16 points_l[(uint16)USE_num][2] = {{0}};//左线
uint16 points_r[(uint16)USE_num][2] = {{0}};//右线
uint16 dir_r[(uint16)USE_num][2] = {0};//用于储存右边的生长方向
uint16 dir_l[(uint16)USE_num][2] = {0};//用于储存左边的生长方向
uint16 data_stastics_l = 0;
uint16 data_stastics_r = 0;
uint8 hightest = 0;

void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 *hightest)
{
    uint8 i=0, j=0;

    //左边变量
    uint8 search_filds_l[8][2] = {{0}};
    uint8 index_l = 0;
    uint8 temp_l[8][2] = {{0}};
    uint8 center_point_l[2] = {0};
    uint16 l_data_stastics;//统计左边
    //定义八个领域
    static sint8 seeds_l[8][2] = {{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1},};//这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = {{0}};
    uint8 index_r = 0;
    uint8 temp_r[8][2] = {{0}};
    uint8 center_point_r[2] = {0};
    uint16 r_data_stastics;//统计右边
    //定义八个领域
    static sint8 seeds_r[8][2] = {{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1},};//这个是逆时针

    l_data_stastics = *l_stastic;
    r_data_stastics = *r_stastic;
}


