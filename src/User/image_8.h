#ifndef _IMAGE_8_H
#define _IMAGE_8_H

//数据类型声明
typedef   signed          char int8;
typedef   signed short     int int16;
typedef   signed           int int32;
//typedef unsigned          char uint8;
//typedef unsigned short     int uint16;
//typedef unsigned           int uint32;


//宏定义
#define image_h   120//图像高度
#define image_w   188//图像宽度

#define white_pixel   255
#define black_pixel   0

#define borde_max     image_w-2
#define borde_min     1
#define bin_jump_num  1

#endif
