
#ifndef _IMAGE_H_
#define _IMAGE_H_

#include <string.h>
#include "zf_common_headfile.h"
//=================================================定义 image 参数结构�??===============================================
#define image_h    90//图像高度
#define image_w    140//图像宽度

#define show_float                   ips114_show_float
#define show_string                  ips114_show_string

typedef struct {
    uint8_t leftx;   // 白条左边�??
    uint8_t rightx;  // 白条右边�??
    uint8_t width;   // 白条宽度
} Whitestrip;

typedef struct Imu
{
  uint8 broken_road_flag;
  int16 aim_angle;
  float angle_speed;
  float now_angle;
  float angle_error;
}Imu;

typedef struct
{

    uint8 last_flag;     // ???flag?????????
    uint8 trans_start_y; // ?????
    uint8 trans_start_x; // ????x?????
    uint8 trans_end_x;   // ????x???????
    uint8 trans_active;
} Connect_90;
extern Connect_90 connect90;

enum
{
    normal=0,
    l90,
    r90,
    crossroad,
    lcircle,
    rcircle,
    cut,
    stop,
   

};
extern  uint8 fixed_thres;
extern uint8 sobel_thres;
extern int  first_error;

extern uint8 flagdebug;



extern uint8 road_status;
extern Imu imu;

extern Whitestrip whitearea[image_h][10]; 
extern int aindex[image_h];              



typedef struct
{
    uint8 top_jump;  
    uint8 bottom_jump;
    uint8 left_jump; 
    uint8 right_jump;
    uint8 left_jump_circle;
    uint8 right_jump_circle;
    uint8 top_jump_circle;
    uint8 l_c_connect;
    uint8 r_c_connect;
}element_check;
extern element_check  check;

extern int flag;
extern int left_wx;
extern int right_wx;

typedef struct//跳变坐标
{  
    uint8 bottom_jump_x;
    uint8 bottom_jump_y;
    uint8 left_jump_x; 
    uint8 left_jump_y; 
    uint8 right_jump_x;
    uint8 right_jump_y;    
    uint8 top_jump_x;
    uint8 top_jump_y;    
    uint8 left_jump_circle_x;
    uint8 left_jump_circle_y;
    uint8 right_jump_circle_x;
    uint8 right_jump_circle_y;
    uint8 top_jump_circle_x;   
    uint8 top_jump_circle_y;
    uint8  l_c_connect_x;
    uint8  l_c_connect_y;
    uint8  r_c_connect_x;
    uint8  r_c_connect_y;         
}jump;
extern jump  coord;


extern uint8 Image_use_zip[image_h][image_w];//图像数组
extern int16 err;

extern uint8 sobel_Image[image_h][image_w];


extern uint8 center_line[image_h];//�??线数�??

extern float nav_angle;
extern uint8 test_index;

extern double oriimg_error;
//=================================================声明 image 基�?�函�?================================================
void image_process();
void gyro_summation();
void imu_param_init();

#endif


