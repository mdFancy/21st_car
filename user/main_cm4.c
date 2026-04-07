/*********************************************************************************************************************
* CYT2BL3 Opensourec Library 即（ CYT2BL3 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT2BL3 开源库的一部分
*
* CYT2BL3 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm4
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT2BL3 
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-11-19       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设


bool acflag=0;
uint16 counter_now = 0;
uint16 counter_last = 0;
uint16 counter = 0;
uint8  nav_record_flag =1;  //模式



// **************************** 代码区域 ****************************

void uart_debug()
{
    //printf("%d\n",counter);
    // printf("%d\n",limitvalue);
    //printf("%d\n",counter);
    //printf("%d\n",road_status);
    //printf("%f,%d\n",imu.now_angle,encoder.sum);
    //printf("%d,%d,%d\n",found_left,found_up,found_right);
    // printf("%f,%d,%f,%f,%d,%d,%d\n",imu.now_angle,imu.aim_angle,Coor_now.x_cur,Coor_now.y_cur,cutcount,road_status,(int)oriimg_error);
    //printf("%d,%d,%d, %d,%d,%d,%d\n",ipts0_num,ipts1_num,dir_backnum0,dir_backnum1,enter_flag,(int)oriimg_error,road_status);//,countup 
    //printf("%d,%d,%d\n",road_status,(int)oriimg_error,encoder.sum);
    //printf("%d\n",counter);
    //printf("%d,%d,%d,%d\n",PID.l_speed,PID.r_speed,(int)oriimg_error,road_status);
    

}


int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);      // 时钟配置及系统初始化<务必保留>
    
    debug_init();                       // 调试串口初始化
    // 此处编写用户代码 例如外设初始化代码等
    // wireless_uart_init();
    mt9v03x_init();
    gpio_init(P00_1,GPO,0,GPO_PUSH_PULL); // P00_1 初始化为GPIO功能、输出模式、输出低电平、推挽输出
    ips114_init();
    flash_init();
    KEY_Init();
    PID_KEY_INIT();


    MainMenu_Set();
    mt9v03x_set_exposure_time(exposure);

    speed_pid_init();
    if(PID.bldc_flag)//开启负压
    {
      pwm_init(TCPWM_CH18_P00_0,100,PID.bldc_duty);
    }
    timer_init(TC_TIME2_CH1, TIMER_US);
    imu_param_init();
    imu660ra_init();  
    gyroOffset_init();//开机延迟2S    去零飘
    Init_Nag();
    
    //first_write();
    flash_index_read();//这里读出来 page index
    // for(int i=0;i<N.elecount;i++)
    // {
    //     printf("%d\n",i);
    //     printf("endindex:  %d      endpage:  %d  \n",N.endindex[i],N.endpage[i]);
    // }
    pit_ms_init(PIT_CH0, 5);


   


    if(nav_record_flag)//惯导采集时
    {
        pit_ms_init(PIT_CH1, 1);  //ch1 1ms中断
    }

    
   
    
    
   
    

    if(flagdebug==1)
    {
        seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
        seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
    }
 
    // ?????????????? ?????????????????  counter
    while(1)
    {
        uint8 flagkey = Key_Scan();
        if(nav_record_flag)
        {
            if((flagkey==KeyUp))
            {
                acflag=1;
                
            }

            //惯导打点记录
            // 此处编写需要循环执行的代码
            if(N.Nag_SystemRun_Index==0&&acflag==1)
            {

                //图像惯导循迹
                if(mt9v03x_finish_flag==1)
                {

                    find_strips();
                    int nav_cnt=0;
                    
                    for(int i=89;i>49;i--)
                    {
                        if(aindex[i]==0) nav_cnt++;
                    }
                    if(nav_cnt==40)
                    {
                        N.Nag_SystemRun_Index=1;

                        N.Mileage_All=0;
                        Q_info.q0 = 1;
                        Q_info.q1 = 0;
                        Q_info.q2 = 0;
                        Q_info.q3 = 0;
                        eulerAngle.Dirchange=0;
                        eulerAngle.last_yaw=0;
                        road_status=cut;
                    }
                    //printf("%d,%d\n",nav_cnt,N.Nag_SystemRun_Index);
                    mt9v03x_finish_flag=0;
                }
                
            }
            
            if((flagkey==KeyLeft) && N.Nag_SystemRun_Index == 1) N.End_f=1;//End_f请勿重复赋值
            if(flagkey==KeyDown)
            {
                N.Flash_page_index-=2;
                N.size=0;
                N.nowelecount++;
            }
            if((flagkey==KeyRight)&&N.End_f==0) 
            {
                flash_index_write();
                printf("write index\n");
                for(int i=0;i<N.elecount;i++)
                {
                    printf("%d\n",i);
                    printf("endindex:  %d      endpage:  %d  \n",N.endindex[i],N.endpage[i]);
                }
                gpio_set_level(P00_1, 1); 
            }
             
            uint16 Col=0;

            show_string(0,Col,"stat");show_float(8*sizeof("stat"),Col,road_status,3,1); 
            show_string(100,Col,"Yaw");show_float(100+8*sizeof("Yaw"),Col,angle_Z,3,1);
            Col+=1;

            show_string(0,Col,"SRI"); show_float(8*sizeof("SRI"),Col,N.Nag_SystemRun_Index,3,1);
            show_string(100,Col,"ac");show_float(100+8*sizeof("ac"),Col,acflag,3,1);

            Col+=1;
            
            show_string(0,Col,"End_f");show_float(8*sizeof("End_f"),Col,N.End_f,3,1);
            show_string(100,Col,"stof");show_float(100+8*sizeof("stof"),Col,PID.stop_flag,3,1);


            Col+=1;
            show_string(0,Col,"index");show_float(8*sizeof("index"),Col,N.size,3,1); 
            show_string(100,Col,"angle");show_float(100+8*sizeof("angle"),Col,nav_angle,3,1);


            Col+=1;
            show_string(0,Col,"page");show_float(8*sizeof("page"),Col,N.Flash_page_index ,3,1);  
            show_string(100,Col,"elecount");show_float(100+8*sizeof("elecount"),Col,N.elecount ,3,1);   


            Col+=1;
            show_string(0,Col,"MA");show_float(8*sizeof("MA"),Col,N.Mileage_All ,3,1);
            show_string(100,Col,"nowelecount");show_float(100+8*sizeof("nowelecount"),Col,N.nowelecount ,3,1);           
            Col+=1;
            show_string(0,Col,"AR");show_float(8*sizeof("AR"),Col,N.Angle_Run ,3,4);//检测读取出来的值
            Col+=1;
            show_string(0,Col,"error");show_float(8*sizeof("error"),Col,N.Final_Out ,3,4);//检测读取出来的值
        }
        else
        {
            if(flagdebug!=0)
            {
                if(flagdebug==2)
                {
                    if (flagkey == KeyLeft)
                    {
                        exposure--;
                        mt9v03x_set_exposure_time(exposure);
                    }
                    if (flagkey == KeyRight)
                    {
                        exposure++;
                        mt9v03x_set_exposure_time(exposure);
                    }  
                    if (flagkey == KeyDown)
                    {
                        exposure-=10;
                        mt9v03x_set_exposure_time(exposure);
                    }        
                    if (flagkey == KeyUp)
                    {
                        exposure+=10; 
                        mt9v03x_set_exposure_time(exposure);
                    }
                         
                }
                else
                {
                    if (flagkey == KeyLeft)
                        sobel_thres--;
                    if (flagkey == KeyRight)
                        sobel_thres++;
                    if (flagkey == KeyDown)
                        fixed_thres--;
                    if (flagkey == KeyUp)
                        fixed_thres++;
                }
                
                
            }
            

            //图像惯导循迹
            if(mt9v03x_finish_flag==1)
            {
                // timer_clear(TC_TIME2_CH1);
                // timer_start(TC_TIME2_CH1);
                // counter_now = timer_get(TC_TIME2_CH1);
            
                
                
                
                image_process();

                uart_debug(); 
                mt9v03x_finish_flag=0;



               

            //     counter_last = timer_get(TC_TIME2_CH1);
            //     counter = counter_last - counter_now;
            //     timer_stop(TC_TIME2_CH1);
            //    printf("%d\n",counter);

            
              //printf("%d,%d\n",PID.l_motor_pid_out,PID.r_motor_pid_out);
            }
          //printf("%f,%d,%d\n",tranced_gyro_z,encoder.l_speed,encoder.r_speed);
            
        }


        

    }
}

// **************************** 代码区域 ****************************
