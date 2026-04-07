#include "zf_common_headfile.h"

/*flash 分配情况
0 1 2 3菜单
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
36 35   第六段惯导

*/
uint8 save_index[6] = {46,44,42,40,38,36};

void flash_Nag_Write()
{
   
    if(flash_check(0, N.Flash_page_index))flash_erase_page(0, N.Flash_page_index);                  
     
    flash_write_page_from_buffer(0,N.Flash_page_index,FLASH_PAGE_LENGTH);
    if(N.End_f == 1)
    { 

        if(N.size ==0)
        {
            N.endindex[N.nowelecount]=500;
            N.endpage[N.nowelecount]=N.Flash_page_index+1;
            N.nowelecount++;
        }
        else
        {
            N.endindex[N.nowelecount]=N.size-1;
            N.endpage[N.nowelecount]=N.Flash_page_index;
            N.nowelecount++;
        }
        flash_write_page_from_buffer(0,N.Flash_page_index,FLASH_PAGE_LENGTH);
        N.Flash_page_index=save_index[N.nowelecount];
        N.size=0;
        printf("write final page\n");
        
    }


    //flash_buffer_clear();
   

}


void flash_index_write()
{
    if(flash_check(0, 4))flash_erase_page(0, 4);
    flash_buffer_clear();
    if(N.nowelecount>N.elecount) N.elecount = N.nowelecount;
        
    flash_union_buffer[0].int32_type = N.elecount;

     for(int i=21;i<31;i++)
    {
        flash_union_buffer[i].int32_type = N.endindex[i-21];
    }
     for(int i=31;i<41;i++)
    {
        flash_union_buffer[i].int32_type = N.endpage[i-31];
    }

    if(memory_f)
    {
        flash_union_buffer[41].int32_type = N.wholeendindex;
        flash_union_buffer[42].int32_type = N.wholeendpage ;
    }


    flash_write_page_from_buffer(0,4,FLASH_PAGE_LENGTH);
    system_delay_ms(20); 

    

}
void flash_index_read()
{
    flash_buffer_clear();
    system_delay_ms(20); 
    flash_read_page_to_buffer(0,4,FLASH_PAGE_LENGTH);
    system_delay_ms(20); 
    N.elecount = (int16)flash_union_buffer[0].int32_type ;
   

    for(int i=21;i<31;i++)
    {
        N.endindex[i-21]  = (int16)flash_union_buffer[i].int32_type ;
    }
    for(int i=31;i<41;i++)
    {
        N.endpage[i-31] = (int16)flash_union_buffer[i].int32_type   ;
    }
    if(speedup_flag)
    {
        N.wholeendindex = (int16)flash_union_buffer[41].int32_type;
        N.wholeendpage  = (int16)flash_union_buffer[42].int32_type;
    }
    for(int i=46;i>36;i--)
    {
        flash_buffer_clear();
        flash_read_page_to_buffer(0,i,FLASH_PAGE_LENGTH);
        system_delay_ms(20); 
        for(int j = 0; j <=MaxSize ;j++)
        {
            nav_data[46-i][j] = flash_union_buffer[j].int32_type/ 100.0f;
          
            //printf("%f\n", nav_data[46-i][j]);
        }
    }
}





