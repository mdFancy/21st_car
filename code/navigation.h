	#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_


//*********************用户设置区域****************************//
#define MaxSize 500    //flash存储的最大页面

//参数范围 <0 - 47>
#define Nag_End_Page 4      //flash中止页面
#define Nag_Start_Page 46   //flah起始页面

#define Nag_Set_mileage 250 //里程计

#define set_wholemileage 400 //里程计

#define Nag_Prev 200    //前瞻
#define Nag_Yaw angle_Z //陀螺仪读取出来的偏航角
  
#define L_Mileage encoder.l_speed   //左轮编码器
#define R_Mileage encoder.r_speed //右轮编码器
//********************************************************//

typedef struct{
       uint8 Nag_SystemRun_Index;   //惯导执行索引
       uint8 End_f;//中止flag

       uint8  datapage_index;
       uint16 size; //  当前惯导数组索引通用计数
       uint8 Flash_page_index;//当前flash页面索引
       uint16 nowelecount;

       uint16 elecount;//第几段惯导，范围0-9
       
       uint16 endindex[10];//结束的索引号
       uint16 endpage[10];//结束的索引页

       uint16 wholeendindex;//整体惯导 结束索引号
       uint16 wholeendpage;//整体惯导  结束页
       uint16 wholesize;
       uint16 whole_page_index;
       float  whole_Mileage;   //里程计数



       uint8 Flash_read_f;//惯导读取flag

       

       float judge_error;
       float Final_Out; //最终输出
       float Mileage_All;   //里程计数
       float Angle_Run; //flash读取的偏航角


       //暂时未开发部分
       int Prev_mile[Nag_Prev]; //前瞻
}Nag;

extern Nag N;   //整个变量的结构体，方便开发和移植
extern uint8 GPSnum;
extern float nav_data[10][512];


extern int32 judge_data[40];
extern bool speedstart;
extern bool slowstart;


void Nag_Run(); //偏航角复现总函数
void Run_Nag_GPS();//偏航角复现

void read_error();

void Run_Nag_Save();    //偏航角读取函数
void Nag_Record();    //偏航角读取总函数
void  store_error();
//****************************//
void Init_Nag();    //这个是参数初始化与flash的缓冲区初始化，请放到函数开始。
void Nag_System();  //这个是惯性导航最后的包装函数，请放到中断中。
#endif /* _NAVIGATION_H_ */
