
#include "zf_common_headfile.h"

/*flash 分配情况
0 1 2 菜单
4    惯导索引号  
    N.elecount   惯导数量[0]
    N.startindex 起始索引号[1-10]    
    N.startpage  起始页数[11-20]
    N.endindex   终止索引号[21-30]  
    N.endpage   终止页数[31-40]

    N.wholeindex 41
    N.wholepage 42




5-36  整体记忆，用于判断  长直道加速




46 45   第一段惯导
44 43   第二段惯导
42 41   第三段惯导
40 39   第四段惯导
38 37   第五段惯导





*/
uint8 GPSnum=2;
Nag N;
float nav_data[10][512]={0};

//0-40
int32  judge_data[40]={0};
uint8  judgeindex    =0;


bool speedstart = false;
bool slowstart = false;

extern bool acflag;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯导参数初始化
// 返回参数     void
// 使用示例     放入程序执行开始
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag()
{
    memset(&N, 0, sizeof(N));
    N.Flash_page_index=Nag_Start_Page;
    
    N.whole_page_index = 5;  
    if(speedup_flag)
    {
        N.wholesize=40;
        flash_read_page_to_buffer(0,5 ,FLASH_PAGE_LENGTH);
        for(int i = 0 ;i<N.wholesize;i++)
        {
            judge_data[i]=(flash_union_buffer[i].int32_type / 100.0f);
        }
    }
    

    // for(int i=0;i<500;i++)
    // {
    //     printf("%d\n",flash_union_buffer[i].int32_type/100);
    // }
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     断路偏航角存入
// 参数说明     将读取的YAW存储到flash中存储
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息      页索引  N.Flash_page_index  
//              页内索引 N.size 
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_Save()
{
    N.Mileage_All+=(R_Mileage+L_Mileage)/2.0;//历程计读取，左右编码器，使用浮点数的话误差能保留下来
    if(N.size > MaxSize)//当大于这页有的flash大小的时候，写入一次，防止重复写入
    {
        flash_Nag_Write();
        //printf("write a page\n");
        N.size=0;   //索引重置为0从下一个缓冲区开始读取
        N.Flash_page_index--;   //flash页面索引减小
        zf_assert(N.Flash_page_index > Nag_End_Page);//防止越界报错
    }

    if(N.Mileage_All >= Nag_Set_mileage)    //大于你的设定值的时候
    {
       int32 Save=(int32)(Nag_Yaw*100); //读取的偏航角放大100倍，避免使用Float类型来存储
       flash_union_buffer[N.size++].int32_type = Save;  //将偏航角写入缓冲区
        //printf("index: %d  yaw: %d\n",(N.size-1),Save);

       if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;//重置历程计数字//保存到flash
       else N.Mileage_All += Nag_Set_mileage;//倒车
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     断路偏航角存入
// 参数说明     将读取的YAW存储到flash中存储
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息      页索引  N.Flash_page_index  
//              页内索引 N.size 
//-------------------------------------------------------------------------------------------------------------------

void speed_judge()
{                           
    uint8 zero_cnt=0;
    uint8 pos_cnt=0;
    uint8 neg_cnt=0;
    uint8 absdata=0;
    for(int i =0 ;i<40; i++)
    {
        absdata=abs(judge_data[i]);
        if(absdata<10)zero_cnt++;
        if(absdata>60)
        {
            if(judge_data[i]>0)pos_cnt++;
            if(judge_data[i]<0)neg_cnt++;
        }
    }

    if(pos_cnt>0&&neg_cnt>0)
    {   
        slowstart = true;
    }
    else
    {
        slowstart = false;
    }
    if(zero_cnt>39&&abs(oriimg_error)<10)
    {
        speedstart = true;
    }
    else
    {
        speedstart = false;
    } 
    
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     全局偏航角存入
// 参数说明     将读取的YAW存储到flash中存储
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息      页索引  N.Flash_page_index  
//              页内索引 N.size 
//-------------------------------------------------------------------------------------------------------------------
void  store_error()
{

    N.whole_Mileage+=(R_Mileage+L_Mileage)/2.0;//历程计读取，左右编码器，使用浮点数的话误差能保留下来
    if(N.wholesize > MaxSize)//当大于这页有的flash大小的时候，写入一次，防止重复写入
    {
        if(flash_check(0, N.whole_page_index))flash_erase_page(0, N.whole_page_index);                  
        flash_write_page_from_buffer(0,N.whole_page_index,FLASH_PAGE_LENGTH);
        //printf("write a page\n");
        N.wholesize=0;   //索引重置为0从下一个缓冲区开始读取
        N.whole_page_index++;   //flash页面索引减小
        zf_assert(N.whole_page_index < 37);//防止越界报错
    }
 

    if(N.whole_Mileage >= set_wholemileage)    //大于你的设定值的时候
    {
       int32 Save=(int32)(error*100); //读取的偏航角放大100倍，避免使用Float类型来存储
       flash_union_buffer[N.wholesize++].int32_type = Save;  //将偏航角写入缓冲区

       if(N.whole_Mileage > 0) N.whole_Mileage -= set_wholemileage;//重置历程计数字//保存到flash
       else N.whole_Mileage += set_wholemileage;//倒车
    }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  


}

void read_error()
{

    N.whole_Mileage+=(R_Mileage+L_Mileage)/2.0;//历程计读取，左右编码器，使用浮点数的话误差能保留下来
    if(N.Flash_read_f == 0)//同样是每次翻页只读取一次，防止反复读取
    {

        flash_buffer_clear();
        flash_read_page_to_buffer(0,N.whole_page_index,FLASH_PAGE_LENGTH);
        N.Flash_read_f=1;

    }

    


    
    
    if(N.whole_Mileage >= set_wholemileage)
    {
        N.wholesize++;
        judgeindex++;
        if(judgeindex==40)judgeindex=0;

        N.judge_error = (flash_union_buffer[N.wholesize].int32_type / 100.0f);
        judge_data[judgeindex] = (int)N.judge_error;
        


        if(N.wholesize > MaxSize)    //当大于设定的flsh大小的时候
        {
            N.whole_page_index++;   //页面减少
            N.wholesize=0;
            N.Flash_read_f=0;
            zf_assert(N.whole_page_index > Nag_End_Page);
        }
        //printf("curindex: %d    targetindex:  %d    curpage: %d     targetpage:  %d  \n",N.size,N.endindex[N.nowelecount],N.Flash_page_index,N.endpage[N.nowelecount]);
        if(N.whole_Mileage > 0) N.whole_Mileage -= set_wholemileage;//重置历程计数字//保存到flash
        else N.whole_Mileage += set_wholemileage; 
        //倒车
    }
    
    

    //N.Final_Out=PID_KEY.nav_kp*N.Final_Out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取偏航角的线程函数
// 参数说明     读取偏航角的线程函数，通过切换N.End_f来切换线程
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Record()
{
        switch(N.End_f)
        {
            case 0:
            {
                Run_Nag_Save();  //默认执行函数
                break;
            }
            case 1:
            {
                flash_Nag_Write();  //写入最后一页，保证falsh存储满
                
                N.End_f=0;
                acflag=0;
                
                N.Nag_SystemRun_Index=0;
               
                road_status=normal;
                // Q_info.q0 = 1;
                // Q_info.q1 = 0;
                // Q_info.q2 = 0;
                // Q_info.q3 = 0;
                // eulerAngle.Dirchange=0;
                // eulerAngle.last_yaw=0;

                //gpio_set_level(P00_1, 0);   //蜂鸣器确认执行
                break;
            }
        }
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用于生成偏差计算
// 参数说明     N.Final_Out为最终生成的偏差大小
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Run()
{

    N.Angle_Run = (nav_data[N.datapage_index][N.size]);//nav_data[0][N.size]
    
    
    N.Mileage_All+=(R_Mileage+L_Mileage)/2.0;//历程计读取，左右编码器，使用浮点数的话误差能保留下来
    if(N.Mileage_All >= Nag_Set_mileage)
    {
        N.size++;
        if(N.size > MaxSize)    //当大于设定的flsh大小的时候
        {
            N.datapage_index++;   //页面增加
            N.size=0;
        }
        //printf("curindex: %d    targetindex:  %d    curpage: %d     targetpage:  %d  \n",N.size,N.endindex[N.nowelecount],N.Flash_page_index,N.endpage[N.nowelecount]);

        if(!nav_record_flag && test_index)
        {
            if(N.size==N.endindex[test_index-1]  &&  N.datapage_index  ==  (46-N.endpage[test_index-1])  ) // 0  46    1   45
            {
                //N.endindex[N.nowelecount]
                //printf("in the end\n");
                N.Nag_SystemRun_Index=0;
                PID.stop_flag=1;
                gpio_set_level(P00_1, 1); 
                road_status=normal;
                return;

            }

        }
        else
        {

            if(N.size==N.endindex[N.nowelecount] && N.datapage_index==  (46-N.endpage[N.nowelecount])  )
            {
                
                //printf("in the end\n");
                N.Nag_SystemRun_Index=0;
                N.nowelecount++;
                if(N.nowelecount==GPSnum)N.nowelecount=0;
                road_status=normal;
                encoder.flag = 1; 
                encoder.sum=0;
                return;

            }
            
        }
        

        if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;//重置历程计数字//保存到flash
        else N.Mileage_All += Nag_Set_mileage; 
          //倒车
    }



    N.Final_Out=N.Angle_Run-Nag_Yaw;//负数向右转，正数向左转
    //N.Final_Out=PID_KEY.nav_kp*N.Final_Out;
    
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯性导航执行函数
// 参数说明     index           索引
// 参数说明     type            类型值
// 返回参数     void
// 使用示例     放入中断中
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

void Nag_System()
{
    //卫保护
    if(!N.Nag_SystemRun_Index||road_status!=cut  )  return;

    switch(N.Nag_SystemRun_Index)
    {
        case 1 : Nag_Record();    //1推车 记录
            break;
        case 2: Nag_Run(); //2是复现
            break;

    }
}




