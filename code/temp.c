#include "zf_common_headfile.h"
uint8 road_status = normal;

int ipts0[LINE_LENTH][2] = {0}; /**< 原图左边线*/
int ipts1[LINE_LENTH][2] = {0}; /**< 原图右边线*/

int dir_f0[LINE_LENTH]; /**< 原图左边线每个点的方向*/
int dir_f1[LINE_LENTH]; /**< 原图右边线每个点的方向*/

int dir_backnum0, dir_backnum1;  /**<统计左右边线的向下点的个数*/
int dir_rightnum0, dir_leftnum1; /**<统计左边线的向右点，和右边线向左的个数*/

int ipts0_num = LINE_LENTH;
int ipts1_num = LINE_LENTH;
/**********************************************************圆环****************************************************************************************/
#define C_STANDARD 60  // 光的标准值
#define C_LENGTH 32    // 扫线的个数
#define C_GET 15       // 扫到多少个点就算圆环
#define C_WITCH_MIN 12 // 判断为圆环的最小宽度
#define C_WITCH_MAX 50 // 判断为圆环的最大宽度
#define c_x_step 1     // 圆圈x轴步长
#define c_y_step 2     // 圆圈y轴步长

// 寻找黑到白跳变点宏定义
#define jump_left(img, x, y) (pixel(img, x, y) < C_STANDARD && pixel(img, x + 1, y) > C_STANDARD && pixel(img, x + 2, y) > C_STANDARD)
#define jump_right(img, x, y) (pixel(img, x, y) < C_STANDARD && pixel(img, x - 1, y) > C_STANDARD && pixel(img, x - 2, y) > C_STANDARD)

#define white(img, x, y) (pixel(img, x, y) > C_STANDARD && pixel(img, x, y - 1) > C_STANDARD)
#define jump2(img, x, y) (pixel(img, x, y) < C_STANDARD && pixel(img, x, y - 1) > C_STANDARD && pixel(img, x, y - 2) > C_STANDARD)
#define jump3(img, x, y) (pixel(img, x, y) > C_STANDARD && pixel(img, x, y - 1) < C_STANDARD && pixel(img, x, y - 2) < C_STANDARD)
int search_right_flag = 0;
int search_left_flag = 0;            /**< 圆圈左边线搜索标志*/
int c_left_line[C_LENGTH][2] = {0};  /**< 圆圈左边线*/
int c_right_line[C_LENGTH][2] = {0}; /**< 圆圈右边线*/

int c_l_num = 0; /**< 圆圈左边线点数*/
int c_r_num = 0; /**< 圆圈右边线点数*/
int current_y = 110;

int c_find_num = 0;
void c_init()
{
    current_y = 110;
    c_l_num = 0;
    c_r_num = 0;
}
void search_left(image_t *img)
{
    search_left_flag = 0;
    int i;
    int start_x = 94 - 70;
    int end_x = 94 + 50;
    for (i = start_x; i < end_x; i += c_x_step)
    {
        if (jump_left(img, i, current_y))
        {
            search_left_flag = 1;
            c_left_line[c_l_num][0] = i + 1;
            c_left_line[c_l_num++][1] = current_y;
            break;
        }
    }
    if (!search_left_flag && c_l_num < C_LENGTH)
    {
        c_left_line[c_l_num][0] = 0;
        c_left_line[c_l_num++][1] = current_y;
    }
}
void search_right(image_t *img)
{
    search_right_flag = 0;
    int i;
    int start_x = 94 + 70;
    int end_x = 94 - 50;
    for (i = start_x; i > end_x; i -= c_x_step)
    {
        if (jump_right(img, i, current_y))
        {
            search_right_flag = 1;
            c_right_line[c_r_num][0] = i - 1;
            c_right_line[c_r_num++][1] = current_y;
            break;
        }
    }
    if (!search_right_flag && c_l_num < C_LENGTH)
    {
        c_right_line[c_r_num][0] = 0;
        c_right_line[c_r_num++][1] = current_y;
    }
}

int l_circle_flag = 0;
int r_circle_flag = 0;
int get_circle_flag = 0;
void s_l_line()
{
    l_circle_flag = 0;
    int cen_x = ipts0[1][0] - 20;
    int cen_y = ipts0[1][1];
    for (int j = cen_y; j > 40; j--)
    {
        if (jump2(&img_raw, cen_x, j) || jump3(&img_raw, cen_x, j))
        {
            l_circle_flag++;
        }
        if (jump2(&img_raw, cen_x - 1, j) || jump3(&img_raw, cen_x - 1, j))
        {
            l_circle_flag++;
        }
        if (jump2(&img_raw, cen_x - 2, j) || jump3(&img_raw, cen_x - 2, j))
        {
            l_circle_flag++;
        }
    }
}
void s_r_line()
{
    r_circle_flag = 0;
    int cen_x = ipts1[1][0] + 20;
    int cen_y = ipts1[1][1];
    for (int j = cen_y; j > 40; j--)
    {
        if (jump2(&img_raw, cen_x, j) || jump3(&img_raw, cen_x, j))
        {
            r_circle_flag++;
        }
        if (jump2(&img_raw, cen_x + 1, j) || jump3(&img_raw, cen_x + 1, j))
        {
            r_circle_flag++;
        }
        if (jump2(&img_raw, cen_x + 2, j) || jump3(&img_raw, cen_x + 2, j))
        {
            r_circle_flag++;
        }
    }
}

void circle_l_r_get()
{
    int decrease = 0;
    if (c_find_num != 1)
    {

        for (int i = 0; i < c_l_num - 2; i++)
        {
            int current_dis = c_right_line[i][0] - c_left_line[i][0];
            int next_dis = c_right_line[i + 2][0] - c_left_line[i + 2][0];
            if (current_dis > 0 && next_dis > 0 && current_dis > next_dis && next_dis > C_WITCH_MIN && next_dis < C_WITCH_MAX + 20 && c_right_line[i][0] && c_left_line[i][0])
            {
                decrease++;
            }
            //  printf("%d, %d, %d ,%d, %d ,%d\n", current_dis, next_dis, decrease,(c_right_line[i][0]+c_left_line[i][0])/2,c_left_line[i][0],c_right_line[i][0]);
        }
        if (decrease >= C_GET)
        {
            c_find_num = 1;
        }
        else
        {
            c_find_num = 2;
        }

        if (c_find_num == 1)
        {
            s_l_line();
            s_r_line();
            if (l_circle_flag > r_circle_flag)
            {
                get_circle_flag = 1;
            }
            else
            {
                get_circle_flag = 2;
            }
        }
        // printf("%d,%d,%d,%d,%d\n", c_find_num, l_circle_flag, r_circle_flag, get_circle_flag,decrease);
    }
}

//  printf("%d\n",get_circle_flag);
void get_line()
{
    if (search_left_flag && search_right_flag)
    {
        current_y -= c_y_step;
    }
    else
    {
        current_y -= c_y_step;
    }
}
void circle_line(image_t *img)
{
    c_init();
    while (current_y > 110 - C_LENGTH * c_y_step)
    {
        search_left(img);
        search_right(img);
        get_line();
    }
}
int incre[C_GET][2] = {0};
void check_circle2(image_t *img)
{

    circle_l_r_get();
    int increase = 0;
    for (int i = 0; i < c_l_num - 2; i++)
    {
        int current_dis = c_right_line[i][0] - c_left_line[i][0];
        int next_dis = c_right_line[i + 2][0] - c_left_line[i + 2][0];
        if (current_dis > 0 && next_dis > 0 && current_dis < next_dis && next_dis > C_WITCH_MIN && next_dis < C_WITCH_MAX && c_right_line[i][0] && c_left_line[i][0])
        {
            incre[increase][0] = c_left_line[i][0];
            incre[increase][1] = c_right_line[i][0];
            increase++;
        }

        //    printf("%d, %d, %d ,%d, %d ,%d\n", current_dis, next_dis, increase,(incre[increase-1][1]+incre[increase-1][0])/2,incre[increase-1][0],incre[increase-1][1]);
        if (increase >= C_GET)
        {
            // int left_sum = 0, right_sum = 0, num_lr = 0;
            // for (int i = increase / 2; i < increase; i++)
            // {
            //     left_sum += incre[i][0];
            //     right_sum += incre[i][1];
            //     num_lr++;
            // }
            // int center = (left_sum + right_sum) / (num_lr * 2);
            if (get_circle_flag == 1)
            {
                road_status = lcircle; // 左环
            }
            else if (get_circle_flag == 2)
            {
                road_status = rcircle; // 右环
            }
            break;
        }
    }
}

int l_c_flag = 0;
int r_c_flag = 0;
int ensum_flag = 1;
int circle_flag = 0;
void lcircle_process(image_t *img)
{
    encoder.flag = 1;
    if (encoder.sum < 4000 && ensum_flag)
    {
        // int left_sum = 0, valid_points = 0,far_p;
        // for (int i = c_l_num / 2; i < c_l_num; i++)
        // { // 从左线中点开始
        //     if (c_left_line[i][0] > 0)
        //     { // 跳过无效点
        //         far_p = c_left_line[i][0];
        //     }
        // }
        // if(!c_left_line[c_l_num-1][0])
        // {
        //     oriimg_error = c_left_line[c_l_num-1][0] - 90;
        // }
        // else
        // {
        //     oriimg_error = c_left_line[c_l_num -2][0] - 90;
        // }
        // float weighted_sum = 0;
        // int total_weight = 0;

        // for (int i = 19; i <= 23; i++)
        // {
        //     if (c_left_line[i][0] > 0) // 确保点有效
        //     {
        //         int weight = i - 18; // 权重，索引越大权重越高
        //         weighted_sum += c_left_line[i][0] * weight;
        //         total_weight += weight;
        //     }
        // }

        // if (total_weight > 0)
        // {
        //     oriimg_error = (weighted_sum / total_weight) - 90; // 计算加权误差
        // }
        // else
        // {
        //     oriimg_error = 0; // 如果没有有效点，误差设为0
        // }
        oriimg_error = (c_left_line[C_LENGTH / 2 - 1][0] - 90) - 15; // 计算误差
        // printf("%f\n", oriimg_error);
    }
    else
    {
        oriimg_error = (ipts0[60][0] + ipts1[60][0]) / 2 - 90;
        if (encoder.sum > 12000 || !ensum_flag)
        {
            int increase = 0;
            for (int i = 0; i < c_l_num - 1; i++)
            {
                int current_dis = c_right_line[i][0] - c_left_line[i][0];
                int next_dis = c_right_line[i + 1][0] - c_left_line[i + 1][0];
                if (current_dis < next_dis && next_dis > C_WITCH_MIN && next_dis < C_WITCH_MAX)
                {
                    increase++;
                }
                if (increase >= C_GET)
                {
                    circle_flag = 1;
                    break;
                }
            }
            if (circle_flag && ensum_flag) // 如果出环
            {
                encoder.sum = 0; // 重置积分
                ensum_flag = 0;
            }
            else if (!ensum_flag)
            {
                oriimg_error = c_right_line[C_LENGTH / 2][0] - 90;
            }
            if (circle_flag && encoder.sum > 6000) // 如果出环
            {
                road_status = normal; // 进入正常状态
                encoder.flag = 0;     // 关闭编码器
                circle_flag = 0;      // 重置圆环标志
                ensum_flag = 1;       // 重置积分标志
                l_c_flag = 0;         // 重置左圆环标志
                r_c_flag = 0;
                get_circle_flag = 0; // 重置圆环标志
                c_find_num = 0;      // 重置圆环标志
                                     // printf("%d\n", 0); // 重置右圆环标志
                // gpio_set_level(P23_7, 0);        // 重置右圆环标志
            }
        }
    }
    //  printf("%f,%d,%d\n", oriimg_error, encoder.sum, 0);
}

void rcircle_process(image_t *img)
{
    encoder.flag = 1;
    if (encoder.sum < 4000 && ensum_flag)
    {
        // int right_sum = 0, valid_points = 0;
        // for (int i = c_r_num / 2; i < c_r_num; i++)
        // { // 从右线中点开始
        //     if (c_right_line[i][0] > 0)
        //     { // 跳过无效点
        //         int weight = i - (c_r_num / 2) + 1; // 权重，距离越远权重越大
        //         right_sum += c_right_line[i][0] * weight;
        //         valid_points += weight;
        //     }
        // }
        // if (valid_points > 0)
        // {
        //     oriimg_error = (right_sum / valid_points) - 90; // 计算误差
        // }
        // if(!c_right_line[c_r_num-1][0])
        // {
        //     oriimg_error = c_right_line[c_r_num-1][0] - 90;
        // }
        // else
        // {
        //     oriimg_error = c_right_line[c_r_num -2][0] - 90;
        // }
        // float weighted_sum = 0;
        // int total_weight = 0;

        // for (int i = 19; i <= 23; i++)
        // {
        //     if (c_right_line[i][0] > 0) // 确保点有效
        //     {
        //         int weight = i - 18; // 权重，索引越大权重越高
        //         weighted_sum += c_right_line[i][0] * weight;
        //         total_weight += weight;
        //     }
        // }

        // if (total_weight > 0)
        // {
        //     oriimg_error = (weighted_sum / total_weight) - 90; // 计算加权误差
        // }
        // else
        // {
        //     oriimg_error = 0; // 如果没有有效点，误差设为0
        // }
        oriimg_error = (c_right_line[C_LENGTH / 2 - 1][0] - 90) + 15; // 计算误差
        //  printf("%f,%d\n", oriimg_error,1);
    }
    else
    {
        oriimg_error = (ipts0[60][0] + ipts1[60][0]) / 2 - 90;
        if (encoder.sum > 12000 || !ensum_flag)
        {
            int increase = 0;
            for (int i = 0; i < c_r_num - 1; i++)
            {
                int current_dis = c_right_line[i][0] - c_left_line[i][0];
                int next_dis = c_right_line[i + 1][0] - c_left_line[i + 1][0];

                if (current_dis < next_dis && next_dis > C_WITCH_MIN && next_dis < C_WITCH_MAX)
                {
                    increase++;
                }
                if (increase >= C_GET)
                {
                    circle_flag = 1;
                    break;
                }
            }
            if (circle_flag && ensum_flag) // 如果出环
            {
                encoder.sum = 0; // 重置积分
                ensum_flag = 0;
            }
            if (!ensum_flag)
            {
                oriimg_error = c_left_line[C_LENGTH / 2][0] - 90;
            }
            if (circle_flag && encoder.sum > 6000) // 如果出环
            {
                road_status = normal; // 进入正常状态
                encoder.flag = 0;     // 关闭编码器
                circle_flag = 0;      // 重置圆环标志
                ensum_flag = 1;       // 重置积分标志
                l_c_flag = 0;         // 重置左圆环标志
                r_c_flag = 0;         // 重置右圆环标志
                get_circle_flag = 0;  // 重置圆环标志
                c_find_num = 0;       // 重置圆环标志
                // gpio_set_level(P23_7, 0); // 关闭圆环指示灯
                //           printf("%d\n", 1); // 重置右圆环标志
            }
        }
        //   printf("%f,%d,%d\n", oriimg_error, encoder.sum, 1);
    }
}


/*************************************************************************************/

int cutcount = 0;
int temp = 0;

void status_judge()
{
    if (road_status == normal)
    {

        int minxl = 188;
        int minxl_index = 0;

        int maxxr = 0;
        int maxxr_index = 0;

        for (int i = 0; i < ipts0_num; i++)
        {
            if (ipts0[i][0] < minxl)
            {
                minxl = ipts0[i][0];
                minxl_index = i;
            }
        }
        for (int i = 0; i < ipts1_num; i++)
        {
            if (ipts1[i][0] > maxxr)
            {
                maxxr = ipts1[i][0];
                maxxr_index = i;
            }
        }
        int dis_xl = abs(minxl_index - ipts0_num / 2);
        int dis_xr = abs(maxxr_index - ipts1_num / 2);
        int dis = maxxr - minxl;
        if ((dis_xl < 10 || dis_xr < 10) && dis > 30 && dis < 80 && dir_rightnum0 > 10 && dir_leftnum1 > 10)
        {
            road_status = stop;
        }

        // if(flagdebug==1)
        // {

        //     printf("%d,%d,%d,%d,%d\n",dis_xl,dis_xr,dis,dir_rightnum0,dir_leftnum1);
        // }

        // find_strip(&img_raw);
        // analyze_circle();

        if (((typecount + 1) < limitvalue) && dir_backnum0 > 30 && dir_backnum1 > 30)
        {
            nav_start_init();
            road_status = cut;
        }
    }
}

#define Y_MAX 120 // 图像底部Y坐标（假设图像高度120）
#define EXP_K 0.1 // 指数衰减系数（需实测调整）

int enter_flag = 0;
int endcount = 0; // 停车 计数器
void status_control()
{

    switch (road_status)
    {
    case normal:
    {
	if (dir_rightnum0 > 40 && dir_leftnum1 < 5) // 右直角
        {
            oriimg_error = ipts1[ipts1_num - 2][0] - 85;
            enter_flag = 1;
            break;
        }
        if (dir_leftnum1 > 40 && dir_rightnum0 < 5) // 左直角
        {
            oriimg_error = ipts0[ipts0_num - 2][0] - 95;
            enter_flag = 2;
            break;
        }
	
	if(ipts0_num-ipts1_num>40)
	{
	    oriimg_error= ipts0[70][0] - 90;
	}
	else if(ipts1_num-ipts0_num>40)
	{
	    oriimg_error= ipts1[70][0] - 90;
	}
	else
	{
	    oriimg_error = (ipts0[70][0] + ipts1[70][0]) / 2 - 90;
	}
        
        

        //check_circle2(&img_raw);

        break;
    }
    case stop:
    {
        encoder.flag = 1;

        if (endcount == 1)
        {
            if (encoder.sum > 4000)
            {

                encoder.flag = 0;

                road_status = normal;
                PID.base_speed = 0;
                oriimg_error = 0;
            }
        }
        if (endcount == 0)
        {
            if (encoder.sum > 4000)
            {
                endcount = 1;
                encoder.flag = 0;

                road_status = normal;
            }
        }

        break;
    }
    case cut:
    {

        nav_process();

        break;
    }
    case lcircle:
    {
        lcircle_process(&img_raw);
        break;
    }
    case rcircle:
    {
        rcircle_process(&img_raw);
        break;
    }
    }
}
void print_line()
{
    for(int i= 0;i<ipts0_num;i++)
    {
        ips114_draw_point((ipts0[i][0]),(ipts0[i][1]),RGB565_RED);
    }
    for(int i=0;i<ipts1_num;i++)
    {
        ips114_draw_point((ipts1[i][0]),(ipts1[i][1]),RGB565_GREEN);
    }

    // for (int i = 0; i < 20; i++)
    // {
    //     for (int j = 0; j < aindex[i]; j++)
    //     {
    //         ips114_draw_point((whitearea[i][j].leftx * MT9V03X_W / 2 / 188), ((80 - i * 2) * MT9V03X_H / 2 / 120), RGB565_RED);
    //         ips114_draw_point((whitearea[i][j].rightx * MT9V03X_W / 2 / 188), ((80 - i * 2) * MT9V03X_H / 2 / 120), RGB565_GREEN);
    //     }
    // }
    // for(int i =0 ;i< 188;i++)
    // {
    //     ips114_draw_point((i*MT9V03X_W/2/188),(40*MT9V03X_H/2/120),RGB565_YELLOW);
    //     ips114_draw_point((i*MT9V03X_W/2/188),(80*MT9V03X_H/2/120),RGB565_YELLOW);
    // }
    // for(int i= 0;i<ipts0_num;i++)
    // {
    //     ips114_draw_point((bipts0[i][0]*MT9V03X_W/2/188),(bipts0[i][1]*MT9V03X_H/2/120),RGB565_RED);
    // }
    // for(int i=0;i<ipts1_num;i++)
    // {
    //     ips114_draw_point((bipts1[i][0]*MT9V03X_W/2/188),(bipts1[i][1]*MT9V03X_H/2/120),RGB565_GREEN);
    // }
}
uint8 flagdebug = 0;
void image_process()
{
    //circle_line(&img_raw);
    dir_backnum0 = 0, dir_backnum1 = 0, dir_rightnum0 = 0, dir_leftnum1 = 0;
    if (road_status != cut)
    {
        if (find_start_point(&img_raw))
        {
            find_oriline();

            
        }
    }
    //status_judge();
    status_control();

    if (flagdebug == 1)
    {
        print_line();
    }
}


void findline_lefthand_adaptive_grow(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num, int dir_[], int *num0, int *num1)
{
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0, back_num = 0, right_num = 0;
    while (step < *num && half < x && x < img->width - half - 1 && 0 < y && y < img->height - half && turn < 4)
    {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++)
        {
            for (int dx = -half; dx <= half; dx++)
            {
                local_thres += pixel(img, x + dx, y + dy);
            }
        }
        local_thres /= block_size * block_size;
        local_thres += clip_value;

        int front_value = pixel(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontleft_value = pixel(img, x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);
        if (front_value < local_thres)
        {
            dir = (dir + 1) % 4; // 此处为转向，生长方向还没有固定
            turn++;
        }
        else if (frontleft_value < local_thres)
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;

            // ips114_draw_point((x*MT9V03X_W/2/188),(y*MT9V03X_H/2/120),RGB565_GREEN);

            dir_[step] = dir;
            if (dir == 2)
            {
                back_num++; // dir==2判断向下生长
            }
            if (dir == 1)
            {
                right_num++; // dir==1判断向右生长
            }
            step++;
            turn = 0;
        }
        else
        {
            dir_[step] = dir;
            if (dir == 2)
            {
                back_num++; // dir==2判断向下生长
            }
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = x;
            pts[step][1] = y;

            // ips114_draw_point((x*MT9V03X_W/2/188),(y*MT9V03X_H/2/120),RGB565_GREEN);

            step++;
            turn = 0;
        }
    }
    *num = step;
    *num0 = back_num;
    *num1 = right_num;
}

void findline_righthand_adaptive_grow(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num, int dir_[], int *num0, int *num1)
{
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0, back_num = 0, left_num = 0;
    while (step < *num && 0 < x && x < img->width - half - 1 && 0 < y && y < img->height - half && turn < 4)
    {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++)
        {
            for (int dx = -half; dx <= half; dx++)
            {
                local_thres += pixel(img, x + dx, y + dy);
            }
        }
        local_thres /= block_size * block_size;
        local_thres += clip_value;

        int front_value = pixel(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontright_value = pixel(img, x + dir_frontright[dir][0], y + dir_frontright[dir][1]);
        if (front_value < local_thres)
        {
            dir = (dir + 3) % 4;
            turn++; // 此处为转向，生长方向还没有固定
        }
        else if (frontright_value < local_thres)
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;

            // ips114_draw_point((x*MT9V03X_W/2/188),(y*MT9V03X_H/2/120),RGB565_RED);
            dir_[step] = dir;
            if (dir == 2)
            {
                back_num++; // dir==2判断向下生长
            }
            if (dir == 3)
            {
                left_num++; // dir==3判断向左生长
            }
            step++;
            turn = 0;
        }
        else
        {
            dir_[step] = dir;
            if (dir == 2)
            {
                back_num++; // dir==2判断向下生长
            }
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;

            // ips114_draw_point((x*MT9V03X_W/2/188),(y*MT9V03X_H/2/120),RGB565_RED);
            step++;
            turn = 0;
        }
    }
    *num = step;
    *num0 = back_num;
    *num1 = left_num;
}

uint8 startxl = 0;
uint8 startxr = 0;

uint8 startyl = 110;
uint8 startyr = 110;

bool flag_l = 0;
bool flag_r = 0;

uint8 hl = 0;
uint8 hr = 187;

uint8 standard = 60;
uint8 thres = 50;

int32 clip_value = -4;
int32 adaptive_block = 7;

bool find_start_point(image_t *img)
{
    int hy = 110; // 初始扫描行

    while (hy >= 90)
    {
        flag_l = 0, flag_r = 0;
        startxl = 0, startxr = 0;

        // 左边缘扫描（从左边界向右）
        for (int i = hl; i < 180; i++)
        {
            if (pixel(img, i, hy) < thres && pixel(img, i + 1, hy) > thres && pixel(img, i + 2, hy) > thres)
            {
                startxl = i + 1;
                hl = clip(startxl - 20, 5, 182);
                flag_l = 1;
                break;
            }
        }

        // 右边缘扫描（从右边界向左）
        for (int i = hr; i > 5; i--)
        {
            if (pixel(img, i, hy) < thres && pixel(img, i - 1, hy) > thres && pixel(img, i - 2, hy) > thres)
            {
                startxr = i - 1;
                hr = clip(startxr + 20, 5, 182);
                flag_r = 1;
                break;
            }
        }
        int dis = startxr - startxl;
        // 有效性检查
        if (flag_l && flag_r && (dis > 0) && (dis < 20))
        {
            startyl = hy;
            startyr = hy;
            return true;
        }

        // 未找到则上移扫描行
        hy--;
    }
    if (!flag_l && flag_r)
    {
        hl = 0;
        hr = 187;
        oriimg_error = startxr - 94;
    }
    else if (!flag_r && flag_l)
    {
        hl = 0;
        hr = 187;
        oriimg_error = 94 - startxl;
    }
    else
    {
        hl = 0;
        hr = 187;
    }

    return false;
}
void find_oriline()
{
    // int local_thres = adaptive_threshold_area(&img_raw,adaptive_block,clip_value,startxl,startyl);
    // while(pixel(&img_raw,startxl,startyl)<local_thres)
    // {
    //     startxl++;
    //     local_thres = adaptive_threshold_area(&img_raw,adaptive_block,clip_value,startxl,startyl);
    // }
    ipts0_num = LINE_LENTH;
    findline_lefthand_adaptive_grow(&img_raw, adaptive_block, clip_value, startxl, startyl, ipts0, &ipts0_num, dir_f0, &dir_backnum0, &dir_rightnum0); // 只有此处使用了左手巡线新版

    // local_thres = adaptive_threshold_area(&img_raw,adaptive_block,clip_value,startxr,startyr);
    // while(pixel(&img_raw,startxr,startyr)<local_thres)
    // {
    //     startxr--;
    //     local_thres = adaptive_threshold_area(&img_raw,adaptive_block,clip_value,startxr,startyr);
    // }
    ipts1_num = LINE_LENTH;
    findline_righthand_adaptive_grow(&img_raw, adaptive_block, clip_value, startxr, startyr, ipts1, &ipts1_num, dir_f1, &dir_backnum1, &dir_leftnum1); // 只有此处使用了右手巡线新版
}

#define  ELE_LENTH   (40)
#define  STARTRAW  (80)

#define ADDCOL (10)

int disnum = 0;
int circle_distance[ELE_LENTH / 2] = {0};
Whitestrip whitearea[ELE_LENTH / 2][10] = {0}; // 数组大小减半
int aindex[ELE_LENTH / 2] = {0};               // 索引数组大小减半
void find_strip(image_t *img)
{
    disnum = 0;
    for (int y = STARTRAW; y > STARTRAW - ELE_LENTH; y -= 2)
    {
        int y_index = (STARTRAW - y) / 2; // 换算为数组索引(0~59)
        int strip_cnt = 0;                // 当前行白条计数器

        bool in_white = false;
        int start_x = 0;

        // 横向扫描当前行
        for (int x = 0; x < img->width; x++)
        {
            uint8 val = pixel(img, x, y);

            // 黑→白跳变：开始新白条
            if (!in_white && val >= standard)
            {
                start_x = x;
                in_white = true;
            }
            // 白→黑跳变：结束当前白条
            else if (in_white && val < standard && x - start_x > 5)
            {
                if (strip_cnt < 10)
                {
                    whitearea[y_index][strip_cnt].leftx = start_x;
                    whitearea[y_index][strip_cnt].rightx = x - 1;
                    whitearea[y_index][strip_cnt].width = (x - 1) - start_x + 1;
                    strip_cnt++;
                }
                in_white = false;
            }
        }
        // 处理行末未闭合的白条
        if (in_white && strip_cnt < 10 && 119 - start_x > 5)
        {
            whitearea[y_index][strip_cnt].leftx = start_x;
            whitearea[y_index][strip_cnt].rightx = img->width - 1;
            whitearea[y_index][strip_cnt].width = (img->width - 1) - start_x + 1;
            strip_cnt++;
        }
        aindex[y_index] = strip_cnt;

        if (strip_cnt == 2)
        {
            if (y_index >= 0 && y_index < ELE_LENTH / 2)
            { // 确保索引有效
                int first_right = whitearea[y_index][0].rightx;
                int second_left = whitearea[y_index][1].leftx;

                // 仅当第二个白条在第一个右侧时记录
                if (second_left > first_right)
                {
                    circle_distance[disnum++] = second_left - first_right;
                }
            }
        }
    }
}

int decreasing_flag = 0; // 递减标记
int lcircle_cnt = 0;     // 连续符合次数
#define TREND_THRESH 5   // 连续趋势阈值

void analyze_circle()
{
    int valid_count = 0;  // 有效数据点数
    int trend_start = -1; // 趋势起始索引

    // 正向遍历（假设从近到远存储）
    for (int i = 0; i < ELE_LENTH / 2; i++)
    {

        // 第一阶段：检测递减趋势
        if (!decreasing_flag)
        {
            if (trend_start == -1)
            {
                trend_start = i;
                valid_count = 1;
            }
            else
            {
                // 检查是否连续递减
                if (circle_distance[i] < circle_distance[i - 1])
                {
                    if (++valid_count >= TREND_THRESH)
                    {
                        decreasing_flag = 1;
                        lcircle_cnt = 0;
                    }
                }
                else
                {
                    trend_start = -1; // 趋势中断重置
                    valid_count = 0;
                }
            }
        }
        // 第二阶段：检测递增趋势
        else
        {
            if (circle_distance[i] > circle_distance[i - 1])
            {
                if (++lcircle_cnt >= TREND_THRESH)
                {
                    road_status = lcircle;
                    decreasing_flag = 0; // 重置状态机
                    break;
                }
            }
            else
            {
                lcircle_cnt = max(0, lcircle_cnt - 1); // 弹性容错
            }
        }
    }
    if (flagdebug == 1)
        printf("%d,%d,%d\n", decreasing_flag, valid_count, lcircle_cnt);
}

int left90[10] = {0}; // 存储左直角弯检测点
int leftnum = 0;
int right90[10] = {0}; // 存储右直角弯检测点
int rightnum = 0;
int upline[10] = {0};
int countup = 0;
void find_eleline(image_t *img)
{

    leftnum = 0;
    rightnum = 0;
    countup = 0;
    memset(left90, 0, sizeof(left90));
    memset(right90, 0, sizeof(right90));
    memset(upline, 0, sizeof(upline));

    //=================================================左线================================================

    for (int i = clip(50, 0, 187); i > clip(30, 0, 187); i -= 2)
    {
        for (int j = 110; j > 40; j--)
        {
            if (pixel(img, i, j) < standard && pixel(img, i, j - 1) > standard && pixel(img, i, j - 2) > standard && pixel(img, i, j - 3) > standard)
            {
                if (flagdebug == 1)
                    ips114_draw_point(((i)*MT9V03X_W / 2 / 188), ((j)*MT9V03X_H / 2 / 120), RGB565_RED);
                left90[leftnum++] = i;
                break;
            }
        }
    }

    //=================================================右线================================================

    for (int i = clip(130, 0, 187); i < clip(150, 0, 187); i += 2)
    {
        for (int j = 110; j > 40; j--)
        {
            if (pixel(img, i, j) < standard && pixel(img, i, j - 1) > standard && pixel(img, i, j - 2) > standard && pixel(img, i, j - 3) > standard)
            {
                if (flagdebug == 1)
                    ips114_draw_point(((i)*MT9V03X_W / 2 / 188), ((j)*MT9V03X_H / 2 / 120), RGB565_RED);
                right90[rightnum++] = i;
                break;
            }
        }
    }
}



#define DASH_SEARCH_RANGE 40    // 横向搜索范围
#define DASH_JUMP_HEIGHT 5     // 向上跳跃行数
#define HEIGHT_DIFF_TOLERANCE 10 // 最高点横向容差
#define MIN_STRIP_WIDTH 3       // 最小白条宽度
#define MAX_SEARCH_ROWS 5       // 最大向上搜索行数
#define EDGE_EXTEND 10          // 边缘扩展像素
// 虚线特征结构体
typedef struct {
    uint8 x;
    uint8 y;
    uint8 width;
} DashPoint;
DashPoint candidate_points[10]; // 候选点缓冲区
uint8 dash_cnt = 0;            // 候选点计数
void find_dash_point_left() 
{
    uint8 found = 0;
    uint8 original_y = heightest_posleft[1];
    
    // 多层搜索（从近到远）
    for(int jump = 1; jump <= MAX_SEARCH_ROWS && !found; jump++) 
    {
        // 计算当前搜索行（向上跳跃）
        uint8 search_y = original_y - 15-DASH_JUMP_HEIGHT * jump;
        if(search_y < 30) break;  // 超出有效范围
        
        // // 动态计算搜索区域（根据历史路径扩展）
        // int search_x_start = 70 - EDGE_EXTEND*jump;
        // int search_x_end = 120 + EDGE_EXTEND*jump;
        
        // // 约束图像边界
        // search_x_start = clip(search_x_start, 0, 180);
        // search_x_end = clip(search_x_end, 0, 180);
        
        dash_cnt = 0;

        
        // 横向扫描（带自适应阈值）
        bool in_strip = false;
        uint8 strip_start = 0;
        for(int x=10; x<=180; x++) 
        {
            // 动态阈值计算（考虑周围5x5区域）
            //int local_thres = adaptive_threshold_area(&img_raw, 5, -5, x, search_y);
            
            if(pixel(&img_raw, x, search_y) > 60) // 阈值偏移增强
            { 
                if(!in_strip) 
                {
                    strip_start = x;
                    in_strip = true;
                }
            } 
            else 
            {
                if(in_strip) 
                {
                    uint8 width = x - strip_start;
                    // 宽度验证 + 路径连续性检查
                    if(width > MIN_STRIP_WIDTH) 
                    {
                        candidate_points[dash_cnt++] = (DashPoint){
                            strip_start, search_y, width};
                        if(dash_cnt >= 8) break;
                    }
                    in_strip = false;
                }
            }
        }

        // 处理行末未闭合白条
        if(in_strip && (180 - strip_start) > MIN_STRIP_WIDTH) 
        {
            candidate_points[dash_cnt++] = (DashPoint){
                strip_start, search_y, 180 - strip_start};
        }

        // 候选点验证
        if(dash_cnt > 0) 
        {
            // 根据宽度和位置综合评分
            uint8 best_index = 0;
            uint8  min_dis = 100;
            for (uint8 i=0; i<dash_cnt; i++) 
            {
                uint8 dis = abs(candidate_points[i].x  - heightest_posleft[0]);
                if(dis<min_dis) 
                {
                    min_dis= dis;
                    best_index=i;
                }

            }
            // 更新路径参数
            currentx = candidate_points[best_index].x;
            currenty = candidate_points[best_index].y;

            linear0[linear0_num][0] = currentx;
            linear0[linear0_num][1] = currenty;
            ips114_draw_point((currentx),(currenty),RGB565_PINK);
            heightest_posleft[0] = currentx;
            heightest_posleft[1] = currenty;
            heightest_posleft[2] = linear0_num;
            heightest_leftcnt = 0;
            linear0_num++;
            found = 1;
            break;
            
        }
    }
}

void find_dash_point_right() 
{
    uint8 found = 0;
    uint8 original_y = heightest_posright[1];

    // 多层搜索（从近到远）
    for(int jump = 1; jump <= MAX_SEARCH_ROWS && !found; jump++) 
    {
        // 计算当前搜索行（向上跳跃）
        uint8 search_y = original_y - 15-DASH_JUMP_HEIGHT * jump;
        if(search_y < 30) break;

        // 动态计算搜索区域（镜像处理）
        // int center_x = heightest_posright[0]; // 右线中心点
        // int search_x_start = center_x - DASH_SEARCH_RANGE - EDGE_EXTEND*jump;
        // int search_x_end = center_x + DASH_SEARCH_RANGE + EDGE_EXTEND*jump;
        
        // 约束图像边界（右侧特殊处理）
        // search_x_start = clip(search_x_start, 0, 180);
        // search_x_end = clip(search_x_end, 0, 180);

        dash_cnt = 0;

        uint8 strip_end = 0;  // 改为记录右边界

        // 横向扫描（从右向左扫描）
        bool in_strip = false;
        for(int x = 180; x >= 10; x--)  // 反向遍历
        {
            // 动态阈值计算
            // int local_thres = adaptive_threshold_area(&img_raw, 5, -5, x, search_y);
            
            if(pixel(&img_raw, x, search_y) > 60) 
            {
                if(!in_strip) {
                    strip_end = x;  // 记录右边界
                    in_strip = true;
                }
            } 
            else 
            {
                if(in_strip) 
                {
                    uint8 width = strip_end - x;
                    if(width > MIN_STRIP_WIDTH) 
                    {
                        candidate_points[dash_cnt++] = (DashPoint){
                            strip_end, search_y, width};
                        if(dash_cnt >= 8) break;
                        
                    }
                    in_strip = false;
                }
            }
        }

        // 处理行首未闭合情况
        if(in_strip && ((strip_end - 10) > MIN_STRIP_WIDTH) )
        {
            candidate_points[dash_cnt++] = (DashPoint){
                strip_end, search_y, strip_end - 10};
        }

        // 候选点验证
        if(dash_cnt > 0) 
        {
            // 选择最接近右线路径的点
            uint8 best_index = 0;
            uint8 min_dis = 100;
            for(uint8 i=0; i<dash_cnt; i++)
            {
                uint8 dis = abs(candidate_points[i].x - heightest_posright[0]);
                if(dis < min_dis) 
                {
                    min_dis = dis;
                    best_index = i;
                }
            }

            // 更新右线路径
            currentx = candidate_points[best_index].x;
            currenty = candidate_points[best_index].y;

            linear1[linear1_num][0] = currentx;
            linear1[linear1_num][1] = currenty;

            // 更新右线最高点
            heightest_posright[0] = currentx;
            heightest_posright[1] = currenty;
            heightest_posright[2] = linear1_num;
            
            heightest_rightcnt = 0;
            linear1_num++;
            found = 1;
            break;
        }
    }
}


//_______________________________________________________贝塞尔直角_______________________________________________________________________________//



/**
 * @brief 生成二次贝塞尔曲线点集
 * @param x1,y1 起点坐标
 * @param x2,y2 终点坐标
 * @param x3,y3 控制点坐标
 * @param Bessel_pt 存储曲线的二维数组（输出）
 * @param num 数组的最大容量（行数）
 * @return 实际生成的点数量（若数组容量不足返回-1）
 */
int generate_bezier(
    int x1, int y1,    // 起点
    int x2, int y2,    // 终点
    int x3, int y3,    // 控制点
    int Bessel_pt[][2], // 输出数组
    int num             // 数组行数
) {
    if (num < 2) return -1; // 至少需要存储起点和终点
    
    const float t_step = 1.0f / (num - 1); // 参数t步长
    int actual_count = 0; // 实际生成点数
    
    for (int i = 0; i < num; i++) {
        float t = i * t_step;
        float u = 1 - t;
        
        // 贝塞尔公式计算坐标
        float x = u*u*x1 + 2*u*t*x3 + t*t*x2;
        float y = u*u*y1 + 2*u*t*y3 + t*t*y2;
        
        // 存入数组（四舍五入为整数坐标）
        Bessel_pt[i][0] = (int)(x + 0.5f); 
        Bessel_pt[i][1] = (int)(y + 0.5f);
        actual_count++;
        
        // 提前到达终点时跳出
        if (Bessel_pt[i][0] == x2 && Bessel_pt[i][1] == y2) break;
    }
    
    return actual_count;
}


int Bezier_left[45][2]={0};
int Bezier_right[45][2]={0};
int Blnum=0;
int Brnum=0;


int left90[10] = {0}; // 存储左直角弯检测点
int leftnum = 0;
int right90[10] = {0}; // 存储右直角弯检测点
int rightnum = 0;
void find_eleline(image_t *img)
{

    leftnum = 0;
    rightnum = 0;
    memset(left90, 0, sizeof(left90));
    memset(right90, 0, sizeof(right90));
    memset(Bezier_left, 0, sizeof(Bezier_left));
    memset(Bezier_right, 0, sizeof(Bezier_right));
    //=================================================左线================================================

    for (int i = clip(50, 0, 187); i > clip(30, 0, 187); i -= 2)
    {
        for (int j = 110; j > 40; j--)
        {
            if (pixel(img, i, j) < thres && pixel(img, i, j - 1) > thres && pixel(img, i, j - 2) > thres && pixel(img, i, j - 3) > thres)
            {
                if (flagdebug == 1)
                    ips114_draw_point(((i)), ((j)), RGB565_RED);
                left90[leftnum++] = j;
                break;
            }
        }
    }

    //=================================================右线================================================

    for (int i = clip(130, 0, 187); i < clip(150, 0, 187); i += 2)
    {
        for (int j = 110; j > 40; j--)
        {
            if (pixel(img, i, j) < thres && pixel(img, i, j - 1) > thres && pixel(img, i, j - 2) > thres && pixel(img, i, j - 3) > thres)
            {
                if (flagdebug == 1)
                    ips114_draw_point(((i)), ((j)), RGB565_GREEN);
                right90[rightnum++] = j;
                break;
            }
        }
    }
}


void process_left_bezier() {
    if (leftnum == 0) return ;

    // 计算x坐标均值
    float sum_x = 0;
    for (int i = 0; i < leftnum; i++) sum_x += left90[i];
    float mean_x = sum_x / leftnum;

    // 找到离均值最近的x
    int Bessel_yl = left90[0];
    float min_diff = fabs(left90[0] - mean_x);
    for (int i = 1; i < leftnum; i++) {
        float diff = fabs(left90[i] - mean_x);
        if (diff < min_diff) {
            Bessel_yl = left90[i];
            min_diff = diff;
        }
    }

    Blnum = generate_bezier(
                    0      ,Bessel_yl,
                    startxl,110,
                    startxl,Bessel_yl,
                    Bezier_left,30);


 
}

void process_right_bezier() {
    if (rightnum == 0) return ;

    // 计算右边缘点x坐标均值
    float sum_x = 0;
    for (int i = 0; i < rightnum; i++) 
        sum_x += right90[i];
    float mean_x = sum_x / rightnum;

    // 找到离均值最近的x坐标
    int Bessel_yr = right90[0];
    float min_diff = fabs(right90[0] - mean_x);
    for (int i = 1; i < rightnum; i++) {
        float diff = fabs(right90[i] - mean_x);
        if (diff < min_diff) {
            Bessel_yr = right90[i];
            min_diff = diff;
        }
    }


    // 生成右贝塞尔曲线（起点在右侧边界，控制点对称）
    Brnum = generate_bezier(
        MT9V03X_W - 1, Bessel_yr, // 起点：右侧边界 (x=图像宽度-1)
        startxr, 110,              // 终点：右边缘底部
        startxr, Bessel_yr,        // 控制点：终点x + 起点y
        Bezier_right, 
        30
    );

}



    int disup = 0, disleft=0,disright=0;

    disup=0,disleft=0,disright=0;
    ips114_draw_point((up_dash[0]),(up_dash[1]),RGB565_GREEN);
    ips114_draw_point((left_dash[0]),(left_dash[1]),RGB565_CYAN);
    ips114_draw_point((right_dash[0]),(right_dash[1]),RGB565_CYAN);

bool found_up , found_left, found_right;
void find_dashpoint()
{
    memset(up_dash, 0, sizeof(up_dash));
    memset(left_dash, 0, sizeof(left_dash));
    memset(right_dash, 0, sizeof(right_dash));
    found_up = false, found_left = false, found_right = false;

    // 1. 向上搜索虚线点
    found_up = find_dash_point_up(
        &img_raw, 
        currenty - 10,        // 从当前行的上一行开始
        currentx,             // 目标x坐标
        30,                   //向上行数
        60,                   //遍历宽度
        30,                   //允许宽度
        5,                    //最小宽度
        thres
    );
    // 2. 向左垂直搜索
    found_left = search_vertical_left(
        &img_raw,
        currentx - 20,        // 从当前列左侧开始
        currenty,            // 目标y坐标
        60,                  //上下距离
        20,                 //向左距离
        20,                 //x差距最大
        5,                  
        thres
    );
    // 3. 向右垂直搜索
    found_right = search_vertical_right(
        &img_raw,
        currentx + 20,        // 从当前列右侧开始
        currenty,       // 目标y坐标
        60,             //上下距离
        20,             //向右距离
        20,             //x差距最大
        5,
        thres
    );
}



    if (stopflag && currenty > 60) 
        {
            find_dashpoint();
            int grow_dir = currentx - up_posleft[0];

            if(grow_dir>0 )
            {
                if(found_up)
                {
                    currentx = up_dash[0];
                    currenty = up_dash[1];
                    findline_lefthand_adaptive_grow(&img_raw, linear0,0);

                }
                

            }
            else
            {
                if(found_left)
                {
                    currentx = left_dash[0];
                    currenty = left_dash[1];
                    findline_lefthand_adaptive_grow(&img_raw, linear0,0);
                }
                else if(found_up)
                {
                    currentx = up_dash[0];
                    currenty = up_dash[1];
                    findline_lefthand_adaptive_grow(&img_raw, linear0,0);


                }
            }
            
            

           
        }

        if (stopflag && currenty > 60) 
        {
            find_dashpoint();
            int grow_dir = currentx - up_posright[0];
            if(grow_dir<0 )
            {
                if(found_up)
                {
                    currentx = up_dash[0];
                    currenty = up_dash[1];
                    findline_lefthand_adaptive_grow(&img_raw, linear0,0);

                }
                

            }
            else
            {
                if(found_left)
                {
                    currentx = left_dash[0];
                    currenty = left_dash[1];
                    findline_lefthand_adaptive_grow(&img_raw, linear0,0);
                }
                else if(found_up)
                {
                    currentx = up_dash[0];
                    currenty = up_dash[1];
                    findline_lefthand_adaptive_grow(&img_raw, linear0,0);


                }
            }
           
        }


        

        if(leftdir && rightdir)//左向右边拐 右向右拐  向右
            {

                cout<<"enter all right"<<endl;
                waitKey(0);
                currentx = linear0[linear0_num-1][0];
                currenty = linear0[linear0_num-1][1];
                memset(right_dash, 0, sizeof(right_dash));
                

                //一直向右
                while(currentx>30 &&currentx<150 && currenty>30 &&linear0_num<LINE_LENTH)
                {
                    found_right = false;
                    found_right = search_vertical_right(
                                                        &img_raw,
                                                        currentx + 20,        // 从当前列左侧开始
                                                        currenty,            // 目标y坐标
                                                        60,                  //上下距离
                                                        20,                 //向左距离
                                                        20,                 //x差距最大
                                                        3,                  
                                                        thres
                                                    );

                    // cout<<"enter222  "<<found_left<<endl;
                    // cout<<right_dash[0][0]<< "    "<<right_dash[0][1]<<"  "<<right_dash[1][0]<< "    "<<right_dash[1][1]<<endl;
                    // cout<<(int)currentx<<"   "<<(int)currenty<<endl;
                    // waitKey(0);

                    if(found_right)
                    {
                        currentx =  right_dash[0][0];
                        currenty =  right_dash[0][1];
                        right_posleft[0] = currentx;
                        right_posleft[1] = currenty;


                        circle(color_img, Point(currentx,currenty), 1, Scalar(0, 216, 255), -1);
                        imshow("Road Detection Result", color_img);
                        waitKey(0);
                        findline_lefthand_adaptive_grow(&img_raw, linear0,2);
                    }
                    else  break; 
                }
                    
                
                
            }




                        else if (!rightdir && !leftdir) // 右向左拐，左向左拐 → 向左延伸
            {
                cout<<"enter all left you"<<endl;
                waitKey(0);

                currentx = linear1[linear1_num - 1][0];
                currenty = linear1[linear1_num - 1][1];
                memset(left_dash, 0, sizeof(left_dash));

                // 持续向左搜索
                while (currentx > 30 && currentx < 150 && currenty > 30 && linear1_num < LINE_LENTH)
                {
                    found_left = search_vertical_left(
                        &img_raw,
                        currentx - 20,   // 向左扩展搜索范围
                        currenty,        // 目标y坐标
                        60,              // 垂直搜索高度
                        20,              // 水平搜索宽度
                        20,              // 最大水平偏差
                        3,               // 最小有效高度
                        thres
                    );

                    cout<<"enter222  "<<found_left<<endl;
                    cout<<left_dash[1][0]<< "    "<<left_dash[1][1]<<endl;
                    cout<<(int)currentx<<"   "<<(int)currenty<<endl;
                    waitKey(0);


                    if (found_left)
                    {
                        currentx = left_dash[1][0];   // 更新为左虚线上边界
                        currenty = left_dash[1][1];
                        left_posright[0] = currentx;
                        left_posright[1] = currenty;

                        findline_righthand_adaptive_grow(&img_raw, linear1, 1); // 向左生长
                    }
                    else
                        break;
                }
            }

//________________________________________________________爬线_______________________________________________________________________________//
uint8 startxl = 0;
uint8 startxr = 0;
uint8 startyl = 110;
uint8 startyr = 110;
uint8 hl = 0;
uint8 hr = 187;
bool flag_l = 0;
bool flag_r = 0;
bool find_start_point(image_t *img)
{
    int hy = 100; // 初始扫描行

    while (hy >= 80)
    {
        flag_l = 0, flag_r = 0;
        startxl = 0, startxr = 0;

        // 左边缘扫描（从左边界向右）
        for (int i = hl; i < 180; i++)
        {
            if (pixel(img, i, hy) < thres && pixel(img, i + 1, hy) > thres )//&& pixel(img, i + 2, hy) > thres
            {
                startxl = i + 1;
                hl = clip(startxl - 10, 5, 182);
                flag_l = 1;
                break;
            }
        }

        // 右边缘扫描（从右边界向左）
        for (int i = hr; i > 5; i--)
        {
            if (pixel(img, i, hy) < thres && pixel(img, i - 1, hy) > thres )//&& pixel(img, i - 2, hy) > thres
            {
                startxr = i - 1;
                hr = clip(startxr + 10, 5, 182);
                flag_r = 1;
                break;
            }
        }
        int dis = startxr - startxl;
        // 有效性检查
        if (flag_l && flag_r && (dis > 0) && (dis < 20))
        {
            startyl = hy;
            startyr = hy;
            return true;
        }

        // 未找到则上移扫描行
        hy--;
    }
    if (!flag_l && flag_r)
    {
        hl = 0;
        hr = 187;
        oriimg_error = startxr - 94;
    }
    else if (!flag_r && flag_l)
    {
        hl = 0;
        hr = 187;
        oriimg_error = 94 - startxl;
    }
    else
    {
        hl = 0;
        hr = 187;
    }

    return false;
}




const int dir_front[4][2] = {{0, -1},
                             {1, 0},
                             {0, 1},
                             {-1, 0}};
const int dir_frontleft[4][2] = {{-1, -1},
                                 {1, -1},
                                 {1, 1},
                                 {-1, 1}};
const int dir_frontright[4][2] = {{1, -1},
                                  {1, 1},
                                  {-1, 1},
                                  {-1, -1}};

uint8 up_posleft[3]={0};  // 0  x坐标  1  y坐标  2 索引号
uint8 up_posright[3]={0};  // 0  x坐标  1  y坐标  2 索引号

uint8 up_leftcnt=0;
uint8 up_rightcnt=0;

uint8 left_posleft[3]={0};  // 0  x坐标  1  y坐标  2 索引号
uint8 left_posright[3]={0};  // 0  x坐标  1  y坐标  2 索引号

uint8 left_leftcnt=0;
uint8 left_rightcnt=0;

uint8 right_posleft[3]={0};  // 0  x坐标  1  y坐标  2 索引号
uint8 right_posright[3]={0};  // 0  x坐标  1  y坐标  2 索引号

uint8 right_leftcnt=0;
uint8 right_rightcnt=0;

uint8 linear0[LINE_LENTH][2]={0};//左边线数组
uint8 linear1[LINE_LENTH][2]={0};//右边线数组

uint8 linear0_num=0;//左有效点数
uint8 linear1_num=0;//右有效点数

uint8 currentx = 0;
uint8 currenty = 0;
int clip_value = -4;
int adaptive_block = 7;
bool stopflagleft = 0;
bool stopflagright =0;
/*
        search_flag   0 表示向上搜  1 向左搜   2  向右搜
    
    */
void findline_lefthand_adaptive_grow(image_t *img, uint8 pts[][2],int search_flag)
{
    /*
        search_flag   0 表示向上搜  1 向左搜   2  向右搜
    
    */
    stopflagleft=0;
    up_leftcnt=0;
    left_leftcnt=0;
    right_leftcnt=0;
    int  turn = 0;
    int dir = 0;
    if(search_flag == 1) dir = 3;
    if(search_flag == 2) dir = 1;
    while (linear0_num < LINE_LENTH && 30 < currentx && currentx < 150 && 10 < currenty && turn < 4 &&!stopflagleft)
    {
        
        int local_thres = adaptive_threshold_area(img,adaptive_block,clip_value,currentx,currenty);

        int front_value = pixel(img, currentx + dir_front[dir][0], currenty + dir_front[dir][1]);
        int frontleft_value = pixel(img, currentx + dir_frontleft[dir][0], currenty + dir_frontleft[dir][1]);
        if (front_value < local_thres)
        {
            dir = (dir + 1) % 4; // 此处为转向，生长方向还没有固定
            turn++;
        }
        else if (frontleft_value < local_thres)
        {
            currentx += dir_front[dir][0];
            currenty += dir_front[dir][1];
            pts[linear0_num][0] = currentx;
            pts[linear0_num][1] = currenty;
            linear0_num++;
            turn = 0;


            switch(search_flag)
            {
                case 0:
                {
                    // 更新最高点
                    if (up_posleft[1] > currenty)
                    {
                        up_posleft[0] = currentx;
                        up_posleft[1] = currenty;
                        up_posleft[2] = linear0_num;

                        up_leftcnt = 0;
                    }
                    else if (currenty >= up_posleft[1])
                    {
                        up_leftcnt++;
                    }
                    if (up_leftcnt >= MAX_Jduge_CNT)
                    {
                        stopflagleft=1;
                          
                                         
                    }
                    
                    break;

                }

                case 1:
                {
                    if (left_posleft[0] > currentx)
                    {
                        left_posleft[0] = currentx;
                        left_posleft[1] = currenty;
                        left_posleft[2] = linear0_num;

                        left_leftcnt = 0;
                    }
                    else if (currentx >= left_posleft[0])
                    {
                        left_leftcnt++;
                    }
                    if (left_leftcnt >= MAX_Jduge_CNT)
                    {
                        stopflagleft=1;                    
                    }

                    break;
                }

                case 2:
                {
                    if (right_posleft[0] < currentx)
                    {
                        right_posleft[0] = currentx;
                        right_posleft[1] = currenty;
                        right_posleft[2] = linear0_num;

                        right_leftcnt = 0;
                    }
                    else if (currentx <= right_posleft[0])
                    {
                        right_leftcnt++;
                    }
                    if (right_leftcnt >= MAX_Jduge_CNT)
                    {
                        stopflagleft=1;                    
                    }

                    break;



                }

            }
          
        }
        else
        {

            currentx += dir_frontleft[dir][0];
            currenty += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[linear0_num][0] = currentx;
            pts[linear0_num][1] = currenty;

            

            linear0_num++;
            turn = 0;



            switch(search_flag)
            {
                case 0:
                {
                    // 更新最高点
                    if (up_posleft[1] > currenty)
                    {
                        up_posleft[0] = currentx;
                        up_posleft[1] = currenty;
                        up_posleft[2] = linear0_num;

                        up_leftcnt = 0;
                    }
                    else if (currenty >= up_posleft[1])
                    {
                        up_leftcnt++;
                    }
                    if (up_leftcnt >= MAX_Jduge_CNT)
                    {
                        stopflagleft=1;    
                                        
                    }
                    
                    break;

                }

                case 1:
                {
                    if (left_posleft[0] > currentx)
                    {
                        left_posleft[0] = currentx;
                        left_posleft[1] = currenty;
                        left_posleft[2] = linear0_num;

                        left_leftcnt = 0;
                    }
                    else if (currentx >= left_posleft[0])
                    {
                        left_leftcnt++;
                    }
                    if (left_leftcnt >= MAX_Jduge_CNT)
                    {
                        stopflagleft=1;                    
                    }

                    break;
                }

                case 2:
                {
                    if (right_posleft[0] < currentx)
                    {
                        right_posleft[0] = currentx;
                        right_posleft[1] = currenty;
                        right_posleft[2] = linear0_num;

                        right_leftcnt = 0;
                    }
                    else if (currentx <= right_posleft[0])
                    {
                        right_leftcnt++;
                    }
                    if (right_leftcnt >= MAX_Jduge_CNT)
                    {
                        stopflagleft=1;                    
                    }

                    break;

                }

            }


        }
    }

}
void findline_righthand_adaptive_grow(image_t *img, uint8 pts[][2],int search_flag)
{
    /*
        search_flag   0 表示向上搜  1 向左搜   2  向右搜
    
    */
    stopflagright=0;
    up_rightcnt=0;
    left_rightcnt=0;
    right_rightcnt=0;
    int dir = 0, turn = 0;
    if(search_flag == 1) dir = 3;
    if(search_flag == 2) dir = 1;
    while (linear1_num < LINE_LENTH && 30 < currentx && currentx < 150 && 10 < currenty  && turn < 4 &&!stopflagright)
    {
        int local_thres = adaptive_threshold_area(img,adaptive_block,clip_value,currentx,currenty);
       

        int front_value = pixel(img, currentx + dir_front[dir][0], currenty + dir_front[dir][1]);
        int frontright_value = pixel(img, currentx + dir_frontright[dir][0], currenty + dir_frontright[dir][1]);
        if (front_value < local_thres)
        {
            dir = (dir + 3) % 4;
            turn++; // 此处为转向，生长方向还没有固定
        }
        else if (frontright_value < local_thres)
        {
            currentx += dir_front[dir][0];
            currenty += dir_front[dir][1];
            pts[linear1_num][0] = currentx;
            pts[linear1_num][1] = currenty;

           
           
            linear1_num++;
            turn = 0;

            switch(search_flag)
            {
                case 0:
                {
                    // 更新最高点
                    if (up_posright[1] > currenty)
                    {
                        up_posright[0] = currentx;
                        up_posright[1] = currenty;
                        up_posright[2] = linear0_num;

                        up_rightcnt = 0;
                    }
                    else if (currenty >= up_posright[1])
                    {
                        up_rightcnt++;
                    }
                    if (up_rightcnt >= MAX_Jduge_CNT)
                    {
                        stopflagright=1;    
                          
                    }
                    
                    break;

                }

                case 1:
                {
                    if (left_posright[0] > currentx)
                    {
                        left_posright[0] = currentx;
                        left_posright[1] = currenty;
                        left_posright[2] = linear0_num;

                        left_rightcnt = 0;
                    }
                    else if (currentx >= left_posright[0])
                    {
                        left_rightcnt++;
                    }
                    if (left_rightcnt >= MAX_Jduge_CNT)
                    {
                        
                        stopflagright=1;                    
                    }


                    break;
                }

                case 2:
                {
                    if (right_posright[0] < currentx)
                    {
                        right_posright[0] = currentx;
                        right_posright[1] = currenty;
                        right_posright[2] = linear0_num;

                        right_rightcnt = 0;
                    }
                    else if (currentx <= right_posright[0])
                    {
                        right_rightcnt++;
                    }
                    if (right_rightcnt >= MAX_Jduge_CNT)
                    {
                        stopflagright=1;                    
                    }



                }

            }
            
        }
        else
        {

            currentx += dir_frontright[dir][0];
            currenty += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[linear1_num][0] = currentx;
            pts[linear1_num][1] = currenty;

            
            linear1_num++;
            turn = 0;


            switch(search_flag)
            {
                case 0:
                {
                    // 更新最高点
                    if (up_posright[1] > currenty)
                    {
                        up_posright[0] = currentx;
                        up_posright[1] = currenty;
                        up_posright[2] = linear0_num;

                        up_rightcnt = 0;
                    }
                    else if (currenty >= up_posright[1])
                    {
                        up_rightcnt++;
                    }
                    if (up_rightcnt >= MAX_Jduge_CNT)
                    {
                        
                        stopflagright=1;                    
                    }
                    
                    break;

                }

                case 1:
                {
                    if (left_posright[0] > currentx)
                    {
                        left_posright[0] = currentx;
                        left_posright[1] = currenty;
                        left_posright[2] = linear0_num;

                        left_rightcnt = 0;
                    }
                    else if (currentx >= left_posright[0])
                    {
                        left_rightcnt++;
                    }
                    if (left_rightcnt >= MAX_Jduge_CNT)
                    {
                        stopflagright=1;                    
                    }


                    break;
                }

                case 2:
                {
                    if (right_posright[0] < currentx)
                    {
                        right_posright[0] = currentx;
                        right_posright[1] = currenty;
                        right_posright[2] = linear0_num;

                        right_rightcnt = 0;
                    }
                    else if (currentx <= right_posright[0])
                    {
                        right_rightcnt++;
                    }
                    if (right_rightcnt >= MAX_Jduge_CNT)
                    {
                        stopflagright=1;                    
                    }



                }

            }

        }

    }

}



int up_dash[2][2] = {0};
int left_dash[2][2] = {0};
int right_dash[2][2] = {0};


struct StripInfoHorizontal find_horizontal_strip(image_t *img, int cur_y, int aim_x, int left, int right, int min_width, int threshold) {
    struct StripInfoHorizontal result = {-1, -1, -1};
    bool in_strip = false;
    int strip_start = 0;
    int min_dist = 188;

    for (int x = left; x <= right; x++) 
    {
        if (pixel(img, x, cur_y) > threshold) 
        {
            if (!in_strip) 
            {
                strip_start = x;
                in_strip = true;
            }
        } else {
            if (in_strip) {
                int strip_end = x - 1;
                int current_width = strip_end - strip_start + 1;
                if (current_width >= min_width) {
                    int center_x = (strip_start + strip_end) / 2;
                    int dist = abs(center_x - aim_x);
                    if (dist <= min_dist) {
                        min_dist = dist;
                        result.start = strip_start;
                        result.end = strip_end;
                        result.center = center_x;
                    }
                }
                in_strip = false;
            }
        }
    }

    if (in_strip) {
        int strip_end = right;
        int current_width = strip_end - strip_start + 1;
        if (current_width >= min_width) {
            int center_x = (strip_start + strip_end) / 2;
            int dist = abs(center_x - aim_x);
            if (dist <= min_dist) {
                result.start = strip_start;
                result.end = strip_end;
                result.center = center_x;
            }
        }
    }
    return result;
}

bool find_dash_point_up(image_t *img, int start_y, int aim_x, int height, int width, int max_dis,
                        int min_width, int threshold) {
    for (int i = 0; i < height; i++) {
        int cur_y = start_y - i;
        if (cur_y < 0) break;

        int left = aim_x - width / 2;
        int right = aim_x + width / 2;
        left = clip(left, 0, img->width - 1);
        right = clip(right, 0, img->width - 1);

        struct StripInfoHorizontal strip = find_horizontal_strip(img, cur_y, aim_x, left, right, min_width, threshold);
        if (strip.start != -1) {
            // Check next two rows
            bool valid = true;
            int last_center = strip.center;
            for (int j = 1; j <= 2; j++) {
                int next_cur_y = cur_y - j;
                if (next_cur_y < 0) {
                    valid = false;
                    break;
                }
                struct StripInfoHorizontal next_strip = find_horizontal_strip(img, next_cur_y, aim_x, left, right, min_width, threshold);
                if (next_strip.start == -1 || abs(next_strip.center - last_center) > max_dis) {
                    valid = false;
                    break;
                }
                last_center = next_strip.center;
            }

            if (valid) {
                up_dash[0][0] = strip.start;
                up_dash[0][1] = cur_y;
                up_dash[1][0] = strip.end;
                up_dash[1][1] = cur_y;
                return true;
            }
        }
    }
    return false;
}



struct StripInfoVertical find_vertical_strip(image_t *img, int cur_x, int top, int bottom, int min_height, int threshold) {
    struct StripInfoVertical result = {-1, -1, -1, -1};
    bool in_strip = false;
    int strip_start = 0;

    for (int y = bottom; y >= top; y--) 
    {
        if (pixel(img, cur_x, y) > threshold) {
            if (!in_strip) {
                strip_start = y;
                in_strip = true;
            }
        } else {
            if (in_strip) {
                int strip_end = y ;
                int strip_height = strip_start - strip_end ;
                if (strip_height >= min_height) {
                    int center_y = (strip_start + strip_end) / 2;
                    result.start_y = strip_start;
                    result.end_y = strip_end;
                    result.center_y = center_y;
                    result.center_x = cur_x;
                    return result;
                }
                in_strip = false;
            }
        }
    }

    if (in_strip) {
        int strip_end = top;
        int strip_height = strip_start - strip_end + 1;
        if (strip_height >= min_height) {
            int center_y = (strip_start + strip_end) / 2;
            result.start_y =  strip_start;
            result.end_y = strip_end;
            result.center_y = center_y;
            result.center_x = cur_x;
        }
    }
    return result;
}

bool search_vertical_left(image_t *img, int start_x, int aim_y, int height, int width, int max_dis,
                          int min_height, int threshold) {
    for (int col = 0; col < width; col++) {  // Step changed to 1
        int cur_x = start_x - col;
        if (cur_x < 0) break;

        int top = aim_y - height / 2;
        int bottom = aim_y + height / 2;
        top = clip(top, 0, img->height - 1);
        bottom = clip(bottom, 0, img->height - 1);

        struct StripInfoVertical strip = find_vertical_strip(img, cur_x, top, bottom, min_height, threshold);
        if (strip.start_y != -1) {
            // Check next two columns
            bool valid = true;
            int last_x = strip.center_x;
            int last_y = strip.center_y;
            for (int j = 1; j <= 2; j++) {
                int next_cur_x = cur_x - j;
                if (next_cur_x < 0) {
                    valid = false;
                    break;
                }
                struct StripInfoVertical next_strip = find_vertical_strip(img, next_cur_x, top, bottom, min_height, threshold);
                if (next_strip.start_y == -1 || 
                    abs(next_strip.center_x - last_x) > 20 || 
                    abs(next_strip.center_y - last_y) > 20) {
                    valid = false;
                    break;
                }
                last_x = next_strip.center_x;
                last_y = next_strip.center_y;
            }

            if (valid) {
                left_dash[0][0] = cur_x;
                left_dash[0][1] =  strip.start_y;  
                left_dash[1][0] = cur_x;
                left_dash[1][1] = strip.end_y; 
                return true;
            }
        }
    }
    return false;
}

bool search_vertical_right(image_t *img, int start_x, int aim_y, int height, int width, int max_dis,
                           int min_height, int threshold) {
    for (int col = 0; col < width; col++) {  // Step changed to 1
        int cur_x = start_x + col;
        if (cur_x >= img->width) break;

        int top = aim_y - height / 2;
        int bottom = aim_y + height / 2;
        top = clip(top, 0, img->height - 1);
        bottom = clip(bottom, 0, img->height - 1);

        struct StripInfoVertical strip = find_vertical_strip(img, cur_x, top, bottom, min_height, threshold);
        if (strip.start_y != -1) {
            // Check next two columns
            bool valid = true;
            int last_x = strip.center_x;
            int last_y = strip.center_y;
            for (int j = 1; j <= 2; j++) {
                int next_cur_x = cur_x + j;
                if (next_cur_x >= img->width) {
                    valid = false;
                    break;
                }
                struct StripInfoVertical next_strip = find_vertical_strip(img, next_cur_x, top, bottom, min_height, threshold);
                if (next_strip.start_y == -1 || 
                    abs(next_strip.center_x - last_x) > 20 || 
                    abs(next_strip.center_y - last_y) > 20) {
                    valid = false;
                    break;
                }
                last_x = next_strip.center_x;
                last_y = next_strip.center_y;
            }

            if (valid) {
                right_dash[0][0] = cur_x;
                right_dash[0][1] = strip.end_y; 
                right_dash[1][0] = cur_x;
                right_dash[1][1] = strip.start_y;   
                return true;
            }
        }
    }
    return false;
}


bool found_up , found_left, found_right;
void judge_lose_dir(int mode)//0 为左   1 为右
{
    found_up =false;
    found_left =false;
    found_right=false;
//这里也要补全
    if(!mode)//左
    {
        if(currenty>60 )
        {
            found_up = find_dash_point_up(&img_raw, currenty-15, currentx+3, 30, 40, 30, 5, thres);
            found_left  = search_vertical_left (&img_raw, currentx-20, currenty,80,30, 20, 5, thres);
            found_right = search_vertical_right(&img_raw, currentx+20, currenty, 80, 30, 20, 5, thres);
        }
        else if(currenty>30)
        {
            found_up = find_dash_point_up(&img_raw, currenty-10, currentx+3, 20, 40, 20, 4, thres);
            found_left  = search_vertical_left (&img_raw, currentx-20, currenty,70,30, 20, 4, thres);
            found_right = search_vertical_right(&img_raw, currentx+20, currenty, 70, 30, 20, 4, thres);
        }
        else 
        {
            found_up = find_dash_point_up(&img_raw, currenty-5, currentx+3, 10, 40, 10, 3, thres);
            found_left  = search_vertical_left (&img_raw, currentx-20, currenty,60,30, 20, 4, thres);
            found_right = search_vertical_right(&img_raw, currentx+20, currenty, 60, 30, 20, 4  , thres);
        }

    }
    else // 右
    {
        if (currenty > 60)
        {
            found_up = find_dash_point_up(&img_raw, currenty - 15, currentx - 3, 30, 40, 30, 5, thres);
            found_left = search_vertical_left(&img_raw, currentx - 20, currenty, 80, 30, 20, 5, thres);
            found_right = search_vertical_right(&img_raw, currentx + 20, currenty, 80, 30, 20, 5, thres);
        }
        else if (currenty > 30)
        {
            found_up = find_dash_point_up(&img_raw, currenty - 10, currentx - 3, 20, 40, 20, 4, thres);
            found_left = search_vertical_left(&img_raw, currentx - 20, currenty, 70, 30, 20, 5, thres);
            found_right = search_vertical_right(&img_raw, currentx + 20, currenty, 70, 30, 20, 5, thres);
        }
        else
        {
            found_up = find_dash_point_up(&img_raw, currenty - 5, currentx - 3, 10, 40, 10, 3, thres);
            found_left = search_vertical_left(&img_raw, currentx - 20, currenty, 60, 30, 20, 5, thres);
            found_right = search_vertical_right(&img_raw, currentx + 20, currenty, 60, 30, 20, 5, thres);
        }
    }

}
void find_straight_line()
{
    
    if (find_start_point(&img_raw))
    {

        fl=0;
        fr=0;

        int bflag=0;
        //________________________________________________________左线_______________________________________________________________________________//
        currentx = startxl;
        currenty = startyl;
        linear0_num = 0;
        up_posleft[0]=startxl;
        up_posleft[1]=startyl;
        up_posleft[2]=0;
/*
        search_flag   0 表示向上搜  1 向左搜   2  向右搜
    
    */
        findline_lefthand_adaptive_grow(&img_raw, linear0,0);


        // 左边缘补线主循环
        while(linear0_num < LINE_LENTH && currentx>30 && currentx<150 && currenty>10&&stopflagleft)
        {
             /*
            这里进行补线
            左     上    右  
            0      0     0         断路 停止搜线  road_status = cut
            0      0     1         之后一直向右走 ，碰到断的就search_vertical_right    search_flag = 2  直到 没搜到 found_right 为false
            0      1     0         向上走 但下一次  还需要再到这里判断方向 find_dash_point_up   search_vertical_right  search_vertical_left
            0      1     1         一直向右 search_vertical_right     search_flag = 2
            1      0     0         一直向左 search_vertical_left
            1      0     1         一直直走 find_dash_point_up
            1      1     0          一直向左 search_vertical_left
            1      1     1          一直向前  find_dash_point_up
            */


            judge_lose_dir(0); // 左补线模式

            if(found_left && !found_up && found_right) // 1 0 1
            { 
                if(right_dash[0][1]<left_dash[0][1])
                {
                    found_right = true;
                    found_left = false;
                    found_up = false;

                }
                else
                {
                    found_right =  false;
                    found_left = true;
                    found_up = false;

                }
            }
            else if(!found_left && found_up && found_right) // 0 1 1 向右生长
            { 
                 while(currentx>30 &&currentx<150 && currenty>30 &&linear0_num<LINE_LENTH&&stopflagleft)
                {

                    found_right = false;
                    found_right = search_vertical_right(
                                                        &img_raw,
                                                        currentx + 20,        // 从当前列左侧开始
                                                        currenty,            // 目标y坐标
                                                        80,                  //上下距离
                                                        20,                 //向左距离
                                                        40,                 //x差距最大
                                                        3,                  
                                                        thres
                                                    );

                    if(found_right)
                    {
                        currentx =  right_dash[0][0];
                        currenty =  right_dash[0][1];
                        right_posleft[0] = currentx;
                        right_posleft[1] = currenty;


                        
                        findline_lefthand_adaptive_grow(&img_raw, linear0,2);
                    }
                    else  break;
                }
                break;
            }
            else if(found_left && found_up && !found_right) // 1 1 0 向左生长
            { 
                while(currentx>30 &&currentx<150 && currenty>30 &&linear0_num<LINE_LENTH&&stopflagleft)
                {
                    

                    found_left = false;
                    found_left = search_vertical_left(
                                                        &img_raw,
                                                        currentx - 20,        // 从当前列左侧开始
                                                        currenty,            // 目标y坐标
                                                        80,                  //上下距离
                                                        20,                 //向左距离
                                                        40,                 //x差距最大
                                                        3,                  
                                                        thres
                                                    );

                    if(found_left)
                    {
                        currentx =  left_dash[0][0];
                        currenty =  left_dash[0][1];
                        left_posleft[0] = currentx;
                        left_posleft[1] = currenty;


                        
                        findline_lefthand_adaptive_grow(&img_raw, linear0,1);
                    }
                    else  break;
                }
                break;

            }

            


            // 状态机决策
            if(!found_left && !found_up && !found_right) // 0 0 0 断路
            { 
                //road_status = cut;
                break;
            }

            else if(!found_left && !found_up && found_right) // 0 0 1  一直向右生长
            { 

                while(currentx>30 &&currentx<150 && currenty>10 &&linear0_num<LINE_LENTH&&stopflagleft)
                {
                    found_right = false;
                    found_right = search_vertical_right(
                                                        &img_raw,
                                                        currentx + 20,        // 从当前列左侧开始
                                                        currenty,            // 目标y坐标
                                                        80,                  //上下距离
                                                        20,                 //向左距离
                                                        20,                 //x差距最大
                                                        3,                  
                                                        thres
                                                    );

                    if(found_right)
                    {
                        currentx =  right_dash[0][0];
                        currenty =  right_dash[0][1];
                        right_posleft[0] = currentx;
                        right_posleft[1] = currenty;


                        
                        findline_lefthand_adaptive_grow(&img_raw, linear0,2);
                    }
                    else  break;
                }
                break;

            }



            else if(!found_left && found_up && !found_right) // 0 1 0 向上生长
            { 

                currentx = up_dash[0][0];
                currenty = up_dash[0][1];
                findline_lefthand_adaptive_grow(&img_raw, linear0, 0);

            }

            else if(found_left && !found_up && !found_right) // 1 0 0 向左生长
            { 
                while(currentx>30 &&currentx<150 && currenty>30 &&linear0_num<LINE_LENTH&&stopflagleft)
                {
                    found_left = false;
                    found_left = search_vertical_left(
                                                        &img_raw,
                                                        currentx - 20,        // 从当前列左侧开始
                                                        currenty,            // 目标y坐标
                                                        80,                  //上下距离
                                                        20,                 //向左距离
                                                        40,                 //x差距最大
                                                        3,                  
                                                        thres
                                                    );

                    if(found_left)
                    {
                        currentx =  left_dash[0][0];
                        currenty =  left_dash[0][1];
                        left_posleft[0] = currentx;
                        left_posleft[1] = currenty;


                        
                        findline_lefthand_adaptive_grow(&img_raw, linear0,1);

                    }
                    else  break;
                }
                break;


            }
            else if(found_left && found_up && found_right)// 1 1 1  向上生长
            { 
                fl=1;
                while(currentx>30 &&currentx<150 && currenty>30 &&linear0_num<LINE_LENTH&&stopflagleft)
                {


                    found_up = false;
                    found_up = find_dash_point_up(
                        &img_raw, 
                        currenty - 10,        // 从当前行的上一行开始
                        currentx,             // 目标x坐标
                        30,                   //向上行数
                        60,                   //遍历宽度
                        30,                   //允许宽度
                        5,                    //最小宽度
                        thres
                    );

                    if(found_up)
                    {
                        currentx =  up_dash[0][0];
                        currenty =  up_dash[0][1];
                        up_posleft[0] = currentx;
                        up_posleft[1] = currenty;


                        
                        findline_lefthand_adaptive_grow(&img_raw, linear0,0);
                    }
                    else  break;
                }
                break;
            }
            
        }



        //________________________________________________________右线_______________________________________________________________________________//
        bflag=0;
        currentx = startxr;
        currenty = startyr;
        linear1_num = 0;
        up_posright[0]=startxr;
        up_posright[1]=startyr;
        up_posright[2]=0;

        findline_righthand_adaptive_grow(&img_raw, linear1,0);

        while(linear1_num < LINE_LENTH && currentx>30 && currentx<150 && currenty>10&&stopflagright)
        {
             /*
            这里进行补线
            左     上    右  
            0      0     0         断路 停止搜线  road_status = cut
            0      0     1         之后一直向右走 ，碰到断的就search_vertical_right    search_flag = 2  直到 没搜到 found_right 为false
            0      1     0         向上走 但下一次  还需要再到这里判断方向 find_dash_point_up   search_vertical_right  search_vertical_left
            0      1     1         一直向右 search_vertical_right     search_flag = 2
            1      0     0         一直向左 search_vertical_left
            1      0     1         一直直走 find_dash_point_up
            1      1     0          一直向左 search_vertical_left
            1      1     1          一直向前  find_dash_point_up
            */
            judge_lose_dir(1); // 左补线模式



            if(found_left && !found_up && found_right) // 1 0 1
            { 
                if(right_dash[1][1]<left_dash[1][1])
                {
                    found_right = true;
                    found_left = false;
                    found_up = false;

                }
                else
                {
                    found_right =  false;
                    found_left = true;
                    found_up = false;

                }

            }
            else if(!found_left && found_up && found_right) // 0 1 1 向右生长
            { 
                 while(currentx>30 &&currentx<150 && currenty>10 &&linear1_num<LINE_LENTH&&stopflagright)
                {
                    found_right = false;
                    found_right = search_vertical_right(
                                                        &img_raw,
                                                        currentx + 20,        // 从当前列左侧开始
                                                        currenty,            // 目标y坐标
                                                        80,                  //上下距离
                                                        20,                 //向左距离
                                                        40,                 //x差距最大
                                                        3,                  
                                                        thres
                                                    );

                    if(found_right)
                    {
                        currentx =  right_dash[1][0];
                        currenty =  right_dash[1][1];
                        right_posright[0] = currentx;
                        right_posright[1] = currenty;


                        
                        findline_righthand_adaptive_grow(&img_raw, linear1,2);
                    }
                    else  break;
                }
                break;
            }
            else if(found_left && found_up && !found_right) // 1 1 0 向左生长
            { 
                while(currentx>30 &&currentx<150 && currenty>10 &&linear1_num<LINE_LENTH&&stopflagright)
                {
                    
                    found_left = false;
                    found_left = search_vertical_left(
                                                        &img_raw,
                                                        currentx - 20,        // 从当前列左侧开始
                                                        currenty,            // 目标y坐标
                                                        80,                  //上下距离
                                                        20,                 //向左距离
                                                        40,                 //x差距最大
                                                        3,                  
                                                        thres
                                                    );

                    if(found_left)
                    {
                        currentx =  left_dash[1][0];
                        currenty =  left_dash[1][1];
                        left_posright[0] = currentx;
                        left_posright[1] = currenty;


                        
                        findline_righthand_adaptive_grow(&img_raw, linear1,1);
                    }
                    else  break;
                }
                break;

            }










            // 状态机决策
            if(!found_left && !found_up && !found_right) // 0 0 0 断路
            { 
                //road_status = cut;
                break;
            }

            else if(!found_left && !found_up && found_right) // 0 0 1  一直向右生长
            { 

                while(currentx>30 &&currentx<150 && currenty>10 &&linear1_num<LINE_LENTH&&stopflagright)
                {
                    found_right = false;
                    found_right = search_vertical_right(
                                                        &img_raw,
                                                        currentx + 20,        // 从当前列左侧开始
                                                        currenty,            // 目标y坐标
                                                        80,                  //上下距离
                                                        20,                 //向左距离
                                                        40,                 //x差距最大
                                                        3,                  
                                                        thres
                                                    );

                    if(found_right)
                    {
                        currentx =  right_dash[1][0];
                        currenty =  right_dash[1][1];
                        right_posright[0] = currentx;
                        right_posright[1] = currenty;


                        
                        findline_righthand_adaptive_grow(&img_raw, linear1,2);
                    }
                    
                    else  break;
                }
                break;

            }

            else if(!found_left && found_up && !found_right) // 0 1 0 向上生长
            { 
                currentx = up_dash[1][0];
                currenty = up_dash[1][1];
                findline_righthand_adaptive_grow(&img_raw, linear1, 0);

            }


            else if(found_left && !found_up && !found_right) // 1 0 0 向左生长
            { 
                while(currentx>30 &&currentx<150 && currenty>10 &&linear1_num<LINE_LENTH&&stopflagright)
                {

                    found_left = false;
                    found_left = search_vertical_left(
                                                        &img_raw,
                                                        currentx - 20,        // 从当前列左侧开始
                                                        currenty,            // 目标y坐标
                                                        80,                  //上下距离
                                                        20,                 //向左距离
                                                        40,                 //x差距最大
                                                        3,                  
                                                        thres
                                                    );

                    if(found_left)
                    {
                        currentx =  left_dash[1][0];
                        currenty =  left_dash[1][1];
                        left_posright[0] = currentx;
                        left_posright[1] = currenty;


                        
                        findline_righthand_adaptive_grow(&img_raw, linear1,1);

                    }
                    else  break;
                }
                break;


            }
            else if(found_left && found_up && found_right)// 1 1 1  向上生长
            { 
                fr=1;
                while(currentx>30 &&currentx<150 && currenty>10 &&linear1_num<LINE_LENTH&&stopflagright)
                {

                    found_up = false;
                    found_up = find_dash_point_up(
                        &img_raw, 
                        currenty - 10,        // 从当前行的上一行开始
                        currentx,             // 目标x坐标
                        30,                   //向上行数
                        60,                   //遍历宽度
                        30,                   //允许宽度
                        5,                    //最小宽度
                        thres
                    );

                    if(found_up)
                    {
                        currentx =  up_dash[1][0];
                        currenty =  up_dash[1][1];
                        up_posright[0] = currentx;
                        up_posright[1] = currenty;


                        
                        findline_righthand_adaptive_grow(&img_raw, linear1,0);
                    }
                    else  break;
                }
                break;
            }
            
        }

    }
}




void print_line()
{
    for(int i= 0;i<linear0_num;i++)
    {
        ips114_draw_point((linear0[i][0]),(linear0[i][1]),RGB565_RED);
    }
    for(int i=0;i<linear1_num;i++)
    {
        ips114_draw_point((linear1[i][0]),(linear1[i][1]),RGB565_GREEN);
    }

    // ips114_draw_point((left_dash[0][0]),(left_dash[0][1]),RGB565_CYAN);

    ips114_draw_point((left_dash[1][0]),(left_dash[1][1]),RGB565_RED);
    
    //ips114_draw_point((right_dash[0][0]),(right_dash[0][1]),RGB565_PURPLE);

    ips114_draw_point((right_dash[1][0]),(right_dash[1][1]),RGB565_GREEN);

    printf("%d,%d\n",(int)error_left,(int)error_right);
    // for(int i=0;i<Blnum;i++)
    // {
    //     ips114_draw_point((Bezier_left[i][0]),(Bezier_left[i][1]),RGB565_RED);

    // }
    // for(int i=0;i<Brnum;i++)
    // {
    //     ips114_draw_point((Bezier_right[i][0]),(Bezier_right[i][1]),RGB565_GREEN);
    // }

}

double error_left;
double error_right;


void status_control()
{
    error_left = 0;
    for(int i= 0;i<linear0_num-5;i++)
    {
        error_left+= (linear0[i][0] - 90);
    }
    error_left /= (linear0_num-5);


    error_right=0;
    for(int i=0;i<linear1_num-5;i++)
    {
        error_right+= (linear1[i][0] - 98);
    }
    error_right /=(linear1_num-5);

    

    if(fr)
    {
        oriimg_error = error_right;
        return;
    } 
    if(fl)  
    {
        oriimg_error = error_left;
        return;
    }
    if(linear0_num>linear1_num)
    {
        oriimg_error = error_left;
    }
    else oriimg_error = error_right;
}


//_______________________________________________________补线_______________________________________________________________________________//

/** 
* @brief 最小二乘法
* @param uint8 begin				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界首地址
*  @see CTest		Slope_Calculate(start, end, border);//斜率
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
	float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
	int16 i = 0;
	float result = 0;
	static float resultlast;

	for (i = begin; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		xysum += i * (border[i]); 
		x2sum += i * i;

	}
	if ((end - begin)*x2sum - xsum * xsum) //判断除数是否为零
	{
		result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
		resultlast = result;
	}
	else
	{
		result = resultlast;
	}
	return result;
}

/** 
* @brief 计算斜率截距
* @param uint8 start				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界
* @param float *slope_rate			输入斜率地址
* @param float *intercept			输入截距地址
*  @see CTest		calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
	uint16 i, num = 0;
	uint16 xsum = 0, ysum = 0;
	float y_average, x_average;

	num = 0;
	xsum = 0;
	ysum = 0;
	y_average = 0;
	x_average = 0;
	for (i = start; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		num++;
	}

	//计算各个平均数
	if (num)
	{
		x_average = (float)(xsum / num);
		y_average = (float)(ysum / num);

	}

	/*计算斜率*/
	*slope_rate = Slope_Calculate(start, end, border);//斜率
	*intercept = y_average - (*slope_rate)*x_average;//截距
}

void ips_show_round(uint8 x,uint8 y)
{
  ips200_draw_point(x+3,y,RGB565_RED);
  ips200_draw_point(x+3,y+3,RGB565_RED);
  ips200_draw_point(x,y+3,RGB565_RED);
  ips200_draw_point(x-3,y+3,RGB565_RED);
  ips200_draw_point(x-3,y,RGB565_RED);
  ips200_draw_point(x-3,y-3,RGB565_RED);
  ips200_draw_point(x,y-3,RGB565_RED);
  ips200_draw_point(x+3,y-3,RGB565_RED);
}

float k_get(uint8 y1,uint8 y2,uint8 *border)
{
  float k;
  k=1.0*(y1-y2)/(border[y1]-border[y2]);
  return k;

}

//连接temp数组上的两点
//x1x2分别为两点的行数
//y1y2分别为两点所对应的列数
void connect_point(int temp[], int x1, int x2, int y1, int y2)//x为行数，y为列数
{
  int dx = x2 - x1;
  int dy = y2 - y1;
  int ux;
  if (dx > 0)
    ux = 1;
  else
    ux = -1;//x的增量方向，取或-1
  int uy;
  if (dy > 0)
    uy = 1;
  else
    uy = -1;//y的增量方向，取或-1
  int x = x1, y = y1, eps;//eps为累加误差
  eps = 0; dx = my_abs(dx); dy = my_abs(dy);
  if (dx > dy)
  {
    for (x = x1; x != x2; x += ux)
    {
      temp[x] = y;
      eps += dy;
      if ((eps << 1) >= dx)
      {
        y += uy; eps -= dx;
      }
      
    }
  }
  else
  {
    for (y = y1; y != y2; y += uy)
    {
      //一些操作
      //if (src[y,x] == type)
      //num++;
      temp[x] = y;
      eps += dx;
      if ((eps << 1) >= dy)
      {
        x += ux; eps -= dy;
      }
      
    }
  }
}

//依据两点的斜率从下往上延申
//x1x2分别为两点的行数
//y1y2分别为两点所对应的列数
void yanshen_to_up(int temp[], int x1, int x2, int y1, int y2)//x为行数，y为列数
{
  int dx = x2 - x1;
  int dy = y2 - y1;
  int ux;
  if (dx > 0)
    ux = 1;
  else
    ux = -1;//x的增量方向，取或-1
  int uy;
  if (dy > 0)
    uy = 1;
  else
    uy = -1;//y的增量方向，取或-1
  int x = x1, y = y1, eps;//eps为累加误差
  eps = 0; dx = my_abs(dx); dy = my_abs(dy);
  
  if (ux == 1)
  {
    ux = -1;
    int temper = 0;
    temper = x1;
    x1 = x2;
    x2 = temper;
    temper = y1;
    y1 = y2;
    y2 = temper;
  }
  
  for (x = x1; x >= 0; x += ux)
  {
    //一些操作
    //if (src[y,x] == type)
    //num++;
    if (y < 1) y = 1;
    if (y > 186) y = 186;
    temp[x] = y;
    eps += dy;
    if ((eps << 1) >= dx)
    {
      y += uy; eps -= dx;
    }
  }
}

//依据两点的斜率从上往下延申
//x1x2分别为两点的行数
//y1y2分别为两点所对应的列数
void yanshen_to_down(int temp[], int x1, int x2, int y1, int y2)//x为行数，y为列数
{
  int dx = x2 - x1;
  int dy = y2 - y1;
  int ux;
  if (dx > 0)
    ux = 1;
  else
    ux = -1;//x的增量方向，取或-1
  int uy;
  if (dy > 0)
    uy = 1;
  else
    uy = -1;//y的增量方向，取或-1
  int x = x1, y = y1, eps;//eps为累加误差
  eps = 0; dx = my_abs(dx); dy = my_abs(dy);
  
  if (ux == -1)
  {
    ux = 1;
    int temper = 0;
    temper = x1;
    x1 = x2;
    x2 = temper;
    temper = y1;
    y1 = y2;
    y2 = temper;
  }
  
  for (x = x1; x <120; x += ux)
  {
    //一些操作
    //if (src[y,x] == type)
    //num++;
    if (y < 1) y = 1;
    if (y > 186) y = 186;
    temp[x] = y;
    eps += dy;
    if ((eps << 1) >= dx)
    {
      y += uy; eps -= dx;
    }
  }
}



// 扫描左侧10列 --------------------------------------------------
bool find_left_line() 
{
    for (int x = 0; x < 10; x++) 
    { // 遍历左侧10列
        int start = -1;
        for (int y = 0; y < image_h; y++) 
        { // 纵向扫描
            if (Image_use_zip[y][x] >= thres) 
            {
                if (start == -1) start = y;
            } 
            else if (start != -1)
            {
                if (y - start >= 3) 
                {        // 高度>3
                    left_center_x = x;
                    left_center_y = (start + y-1) / 2;
                    return true;
                }
                start = -1;
            }
        }
        // 处理列末白条未闭合
        if (start != -1 && (image_h - start) >= 3) 
        {
            left_center_x = x;
            left_center_y = (start + image_h-1) / 2;
            return true;
        }
    }
    return false;
}

// 扫描右侧10列 --------------------------------------------------
bool find_right_line() 
{
    for (int i = 0; i < 10; i++) 
    { // 从右往左扫10列
        int x = image_w - 1 - i;
        int start = -1;
        for (int y = 0; y < image_h; y++) 
        { // 纵向扫描
            if (Image_use_zip[y][x] >= thres) 
            {
                if (start == -1) start = y;
            } 
            else if (start != -1) 
            {
                if (y - start >= 3)
                {        // 高度>3
                    right_center_x = x;
                    right_center_y = (start + y-1) / 2;
                    return true;
                }
                start = -1;
            }
        }
        // 处理列末白条未闭合
        if (start != -1 && (image_h - start) >= 3) 
        {
            right_center_x = x;
            right_center_y = (start + image_h-1) / 2;
            return true;
        }
    }
    return false;
}

// 全局变量存储白条中心坐标（假设已声明）
int up_center_x, up_center_y;
int left_center_x, left_center_y;
int right_center_x, right_center_y;

// 扫描顶部10行 --------------------------------------------------
bool find_up_line() 
{
    for (int y = 0; y < 10; y++) 
    { // 遍历顶部10行
        int start = -1;
        for (int x = 0; x < image_w; x++) 
        { // 横向扫描
            if (Image_use_zip[y][x] >= thres) 
            { // 发现白色
                if (start == -1) start = x;      // 记录起始位置
            } 
            else if (start != -1) 
            {            // 白条结束
                if (x - start >= 3) 
                {            // 宽度>3
                    up_center_x = (start + x-1) / 2;
                    up_center_y = y;
                    return true;
                }
                start = -1; // 重置白条起始
            }
        }
        // 处理行末白条未闭合的情况
        if (start != -1 && (image_w - start) >= 3) 
        {
            up_center_x = (start + image_w-1) / 2;
            up_center_y = y;
            return true;
        }
    }
    return false;
}
bool found_down=false,found_up = false,found_left = false,found_right = false;



unsigned char in_flag = 0;
unsigned char zuoyou_xunxian(uint8(*bin_image)[image_w]) // 从中间往两边搜索中线
{
    int i, j;
    static int left_border = 0, right_border = image_w-1, mid = image_w / 2;
    
    static int last_left_border = 0, last_right_border = image_w-1;
    

    // 第一次巡线，近大远小，近处按旧扫描方式
    for (i = image_h - 1; i >= image_h - 5; i--) // 从最底下往上扫描
    {
            // 往右扫描
            for (j = 0; j < image_w - 2; j++)
            {
                if (bin_image[i][j] == 0 && bin_image[i][j + 1] == 255&& bin_image[i][j + 2] == 255 )
                {
                    left_border = j;
                    break; // 跳出，找到本行边界就没必要循环下去了
                }
            }

            // 没找到右边界的处理
            if (j == image_w - 3)
            {
                left_border = last_left_border;
            }

            // 往左边扫描
            for (j = image_w-1; j > 1; j--)
            {
                if (bin_image[i][j] == 0 && bin_image[i][j - 1] == 255 && bin_image[i][j - 2] == 255) // 黑白认为到达左边界
                {
                    right_border = j;
                    break; // 跳出，找到本行边界就没必要循环下去了
                }
            }

            // 没找到右边界的处理
            if (j == 2)
            {
                right_border = last_right_border;
            }

            mid = (left_border + right_border) / 2; // 中线坐标
            

            l_border[i] = (unsigned char)left_border; // 左边线线数组
            r_border[i] = (unsigned char)right_border; // 右边线线数组
            center_line[i] = (unsigned char)mid; // 修正中心线计算公式
    }


    for(i = image_h - 6;i>0;i--)
    {
        for (j = center_line[i+1]; j > 0; j--)//从上一行的中线往左找
        {
		if (bin_image[i][j] == 0xFF && bin_image[i][j - 1] == 0x00 )
		{
                    left_border = j;
                    break; // 跳出，找到本行边界就没必要循环下去了
		}
        }
            // 没找到左边界的处理，继承上一行的左边界
            if (j == 1)
            {
                left_border = last_left_border;
            }


            // 往右边扫描，从上一行的中线往右边找
        for (j = center_line[i+1]; j < image_w-2; j++)
        {
            if (bin_image[i][j] == 0xFF && bin_image[i][j + 1] == 0x00 ) // 黑黑白认为到达左边界
            {
                right_border = j;
                break; // 跳出，找到本行边界就没必要循环下去了
            }
        }

            // 没找到右边界的处理，继承上一行的右边界
            if (j == image_w-3)
            {
                right_border = last_right_border;
            }
            
        mid = (left_border + right_border) / 2; // 中线坐标
            

        l_border[i] = (unsigned char)left_border; // 左边线线数组
        r_border[i] = (unsigned char)right_border; // 右边线线数组
        center_line[i] = (unsigned char)mid; // 修正中心线计算公式
        
        last_left_border= left_border;
        last_right_border= right_border;

    }

    return in_flag;
}



// 选取最左侧白条中线
void getline_left() 
{
    static int last_center = 70; // 静态变量保存历史值
    
    for (int y = image_h - 1; y >= 0; y--) { // 从 image_h 到 0
        // 无白条时的处理
        if (aindex[y] == 0) 
        {
            center_line[y] = (y == image_h - 1) ? 70 : last_center;
        }
        // 有白条时取第一个（最左）白条中点
        else 
        {
            Whitestrip *strip = &whitearea[y][0];
            uint8_t mid = (strip->leftx + strip->rightx) / 2;
            center_line[y] = mid;
            last_center = mid; // 更新历史值
        }
    }
}

// 选取最右侧白条中线
void getline_right() 
{
    static int last_center = 70; // 静态变量保存历史值
    
    for (int y = image_h - 1; y >= 0; y--) { // 从 image_h 到 0
        // 无白条时的处理
        if (aindex[y] == 0) 
        {
            center_line[y] = (y == image_h - 1) ? 70 : last_center;
        }
        // 有白条时取最后一个（最右）白条中点
        else 
        {
            int last_idx = aindex[y] - 1;
            Whitestrip *strip = &whitearea[y][last_idx];
            uint8_t mid = (strip->leftx + strip->rightx) / 2;
            center_line[y] = mid;
            last_center = mid; // 更新历史值
        }
    }
}










int left_cnt = 0;          // 左边界点数量
int right_cnt = 0;         // 右边界点数量
uint8 found_left_cnt =0;
uint8 found_right_cnt=0;
int flag=0 ;
int left_wx;
int right_wx;
uint8 trackmap[image_h][image_w];
void getline_normal() 
{

    int last_center = 70; // 静态变量保存上次中线位置，首行初始化为70
    bool has_jumped = false; // 标记是否已经跳过行
    
    // 初始化左右边界线
    memset(left_line, 0, sizeof(left_line));
    memset(right_line, 0, sizeof(right_line));
    memset(trackmap, 0, sizeof(trackmap));
    left_cnt = 0;
    right_cnt = 0;
    
    for (int y = image_h-1; y >= 0; y--) 
    { // 从 image_h 到 0
        int wid_thres =0;
        if(y>45) wid_thres=60;
        else  wid_thres=30;
        // 情况1：无白条，继承上一次值
        if (aindex_row[y] == 0) 
        {
            if (y == image_h - 1) 
            {
               return;
            } 
            else 
            { 
                center_line[y] = last_center;
            }
        }
        else
        {
            // 情况2：1个白条，取其中点
            if (aindex_row[y] == 1) 
            {
                Whitestrip_row *strip = &whitearea_row[y][0];
                uint8_t mid = (strip->leftx + strip->rightx) / 2;
                uint8_t width = strip->width;
                if(width < wid_thres) 
                {
                    // 窄白条处理
                    if(abs(mid-last_center)<30)
                    {
                        center_line[y] = mid;
                        last_center = mid; // 更新历史值
                        trackmap[y][strip->leftx]=1;
                        trackmap[y][strip->rightx]=1;
                    }
                    else
                    {
                        center_line[y] = last_center;
                    }
                }
                else 
                {
                    // 宽白条处理：跳行并横向搜索
                    center_line[y] = last_center;
                    if(!has_jumped)
                    {
                        getline_lf(y,last_center);
                        // 向上跳10行
                        int jump_end = max(0, y - 10);
                        for (int j = y - 1; j >= jump_end; j--) 
                        {
                            center_line[j] = last_center; // 继承当前中线值
                        }
                        has_jumped = true;
                        y = jump_end-1; // 更新当前行到跳行后的位置

                    }
                    

                }
            }
            // 情况3：2个白条，选距离更近的
            else if (aindex_row[y] == 2) 
            {
                Whitestrip_row *strip1 = &whitearea_row[y][0];
                Whitestrip_row *strip2 = &whitearea_row[y][1];
                
                // 计算两个白条中点
                uint8_t mid1 = (strip1->leftx + strip1->rightx) / 2;
                uint8_t mid2 = (strip2->leftx + strip2->rightx) / 2;
                
                // 选择距离上次中线更近的
                int diff1 = abs(mid1 - last_center);
                int diff2 = abs(mid2 - last_center);
                if(diff1 <= diff2 )
                {
                    center_line[y] = mid1;
                    last_center = center_line[y];
                    trackmap[y][strip1->leftx]=1;
                    trackmap[y][strip1->rightx]=1;

                } 
                else
                {
                    center_line[y] = mid2;
                    last_center = center_line[y];
                    trackmap[y][strip2->leftx]=1;
                    trackmap[y][strip2->rightx]=1;
                }
                
                
            }
            // 情况4：超过2个白条，继承历史值
            else 
            {
                center_line[y] = last_center;
            }
            
        }
        

    }
    

}

void getline_lf(int sy, int sx)
{
    found_left_cnt =0;
    found_right_cnt=0;
    // 初始化左右搜索的起始位置
    int last_left_y = sy;
    int last_right_y = sy;


    // 向左搜索
    for (int x = clip(sx-10, 0, image_w); x >= 0; x--) 
    {
        if (aindex_col[x] == 0) 
        {
            // 情况1：当前列没有白条，继承上一次
            left_line[x] = last_left_y;
        }
        else if (aindex_col[x] == 1) 
        {
            // 情况2：当前列有一个白条
            uint8_t strip_mid = (whitearea_col[x][0].topy + whitearea_col[x][0].bottomy) / 2;
            
            // 判断与上一列的距离
            if (abs(strip_mid - last_left_y) < 20) 
            {

                // 更新上一次并记录结果
                last_left_y = strip_mid;
                left_line[x] = strip_mid;
                if(  trackmap[whitearea_col[x][0].topy ][x]  ||  trackmap[whitearea_col[x][0].bottomy ][x] )
                {
                    found_left_cnt++;

                }
                left_cnt++;                  
                

            }
            else 
            {
                // 距离过大，继承上一次
                left_line[x] = last_left_y;
            }
        }
        else 
        {
            // 情况3：当前列有多个白条
            int best_idx = -1;
            int min_diff = 100;
            
            // 找到最匹配的垂直白条
            for (int i = 0; i < aindex_col[x]; i++) 
            {
                int diff = abs(whitearea_col[x][i].topy - last_left_y);
                if (diff < min_diff) {
                    min_diff = diff;
                    best_idx = i;
                }
            }
            
            // 计算垂直白条的中点
            uint8_t strip_mid = (whitearea_col[x][best_idx].topy + whitearea_col[x][best_idx].bottomy) / 2;
            
            // 判断与上一列的距离
            if (min_diff < 20) 
            {

                // 更新上一次并记录结果
                last_left_y = strip_mid;
                left_line[x] = strip_mid;
                if(  trackmap[ whitearea_col[x][best_idx].topy ][x]  ||  trackmap[ whitearea_col[x][best_idx].bottomy ][x] )
                {
                    found_left_cnt++;

                }
                left_cnt++;

            }
            else 
            {
                // 距离过大，继承上一次
                left_line[x] = last_left_y;
            }
        }

   

    }




    // 向右搜索
    for (int x = clip(sx+10, 0, image_w); x < image_w; x++) 
    {
        if (aindex_col[x] == 0) 
        {
            // 情况1：当前列没有白条，继承上一次
            right_line[x] = last_right_y;
        }
        else if (aindex_col[x] == 1) 
        {
            // 情况2：当前列有一个白条
            uint8_t strip_mid = (whitearea_col[x][0].topy + whitearea_col[x][0].bottomy) / 2;
            
            // 判断与上一列的距离
            if (abs(strip_mid - last_right_y) < 20) 
            {
                // 更新上一次并记录结果
                last_right_y = strip_mid;
                right_line[x] = strip_mid;
                if( trackmap[whitearea_col[x][0].topy ][x]  ||  trackmap[whitearea_col[x][0].bottomy ][x])
                {
                    found_right_cnt++;
                }
                right_cnt++;

            }
            else 
            {
                // 距离过大，继承上一次
                right_line[x] = last_right_y;
            }
        }
        else 
        {
            // 情况3：当前列有多个白条
            int best_idx = -1;
            int min_diff = 100;
            
            // 找到最匹配的垂直白条
            for (int i = 0; i < aindex_col[x]; i++) 
            {
                int diff = abs(whitearea_col[x][i].topy - last_right_y);
                if (diff < min_diff) {
                    min_diff = diff;
                    best_idx = i;
                }
            }
            
            // 计算垂直白条的中点
            uint8_t strip_mid = (whitearea_col[x][best_idx].topy + whitearea_col[x][best_idx].bottomy) / 2;

            // 判断与上一列的距离
            if (min_diff < 20) 
            {
                // 更新上一次并记录结果
                last_right_y = strip_mid;
                right_line[x] = strip_mid;
                if(  trackmap[ whitearea_col[x][best_idx].topy ][x]  ||  trackmap[ whitearea_col[x][best_idx].bottomy ][x])
                {
                    found_right_cnt++;
                }
                right_cnt++;
                
            }
            else 
            {
                // 距离过大，继承上一次
                right_line[x] = last_right_y;
            }
        }

    }
    if(found_right_cnt>10) right_cnt=0;
    if(found_left_cnt>10)   left_cnt=0;


}
void status_control()
{

    if(left_cnt>40 && right_cnt>40)  //十字
    {
        connect_angle_points(70,89,center_line[5],5);

        unsigned char i;

        sum_mid=0;
        errorcount=0;
        average_mid=0;

        //加权
        for(i=89;i>=5;i--)
        {
            sum_mid+=control_line[i]*mid_weight[i];
            errorcount+=mid_weight[i]; 
        }
        average_mid=(int16)(sum_mid/errorcount);
        oriimg_error=average_mid-(image_w/2);
        oriimg_error=moving_average_filter(history_err,ERR_SIZE,oriimg_error);//滑动平均滤波 
        road_status = crossroad;
        return;
    }
    if(left_cnt>40 && right_cnt<20)  //左直角
    {
        connect_angle_points(70,89,5,left_line[5]);
        for(int i = left_line[5] ; i>0;i--)
        {
            control_line[i]=0;
        }
        unsigned char i;

        sum_mid=0;
        errorcount=0;
        average_mid=0;

        //加权
        for(i=89;i>=5;i--)
        {
            sum_mid+=control_line[i]*mid_weight[i];
            errorcount+=mid_weight[i]; 
        }
        average_mid=(int16)(sum_mid/errorcount);
        oriimg_error=average_mid-(image_w/2);

        oriimg_error=moving_average_filter(history_err,ERR_SIZE,oriimg_error);//滑动平均滤波 
        road_status = l90;
        return;
    }
    if(left_cnt<20 && right_cnt>40)  //右直角
    {
        connect_angle_points(70,89,135,right_line[135]);
        for(int i = right_line[135] ; i>0;i--)
        {
            control_line[i]=image_w-1;
        }
        unsigned char i;

        sum_mid=0;
        errorcount=0;
        average_mid=0;

        //加权
        for(i=89;i>=5;i--)
        {
            sum_mid+=control_line[i]*mid_weight[i];
            errorcount+=mid_weight[i]; 
        }
        average_mid=(int16)(sum_mid/errorcount);
        oriimg_error=average_mid-(image_w/2);

        oriimg_error=moving_average_filter(history_err,ERR_SIZE,oriimg_error);//滑动平均滤波 

        road_status = r90;
        return;
    }


    for(int i = 0; i<image_h;i++)
    {

        control_line[i] = center_line[i];
    }
    unsigned char i;

    sum_mid=0;
    errorcount=0;
    average_mid=0;

    //加权
    for(i=89;i>=5;i--)
    {
        sum_mid+=control_line[i]*mid_weight[i];
        errorcount+=mid_weight[i]; 
    }
    average_mid=(int16)(sum_mid/errorcount);
    oriimg_error=average_mid-(image_w/2);
    oriimg_error=moving_average_filter(history_err,ERR_SIZE,oriimg_error);//滑动平均滤波 
    road_status = normal;
    
}



bool sobelCheckPoint(uint8_t* imageIn, int x, int y, uint8_t th) 
{
    // 边界检查（确保3x3卷积核不越界）
    if (x < 1 || x >= image_w - 1 || y < 1 || y >= image_h - 1) 
        return false;

    short gx = -(short)imageIn[(y-1)*image_w + (x-1)] + (short)imageIn[(y-1)*image_w + (x+1)]
             - (short)imageIn[ y   *image_w + (x-1)]    + (short)imageIn[ y   *image_w + (x+1)]
             - (short)imageIn[(y+1)*image_w + (x-1)]    + (short)imageIn[(y+1)*image_w + (x+1)];

    short gy = -(short)imageIn[(y-1)*image_w + (x-1)] + (short)imageIn[(y+1)*image_w + (x-1)]
             - (short)imageIn[(y-1)*image_w +  x   ]    + (short)imageIn[(y+1)*image_w +  x   ]
             - (short)imageIn[(y-1)*image_w + (x+1)]    + (short)imageIn[(y+1)*image_w + (x+1)];

    return (abs(gx) + abs(gy)) > th;  // 与原始函数相同的梯度合并方式
}


void sobel_filter(uint8_t threshold) 
{
    // 遍历每一行
    for (int y = 0; y < image_h; y++) 
    {
        int strip_count = aindex[y];  // 获取当前行的白条数量
        int valid_strip_count = 0;    // 记录有效白条数量
        
        // 遍历当前行的每个白条
        for (int s = 0; s < strip_count; s++) 
        {
            Whitestrip* strip = &whitearea[y][s];
            bool left_valid = false;
            bool right_valid = false;
            
            // 检查左边界10x10区域
            for (int dy = -5; dy <= 5 && !left_valid; dy++) 
            {
                for (int dx = -5; dx <= 5 && !left_valid; dx++) 
                {
                    int check_x = strip->leftx + dx;
                    int check_y = y + dy;
                    
                    // 边界检查
                    if (check_x >= 0 && check_x < image_w && check_y >= 0 && check_y < image_h) 
                    {
                        // 使用sobelCheckPoint检查该点
                        if (sobelCheckPoint(&mt9v03x_image[0][0], check_x, check_y, threshold)) 
                        {
                            left_valid = true;
                            break;
                        }
                    }
                }
            }
            
            // 检查右边界10x10区域
            for (int dy = -5; dy <= 5 && !right_valid; dy++) 
            {
                for (int dx = -5; dx <= 5 && !right_valid; dx++) 
                {
                    int check_x = strip->rightx + dx;
                    int check_y = y + dy;
                    
                    // 边界检查
                    if (check_x >= 0 && check_x < image_w && check_y >= 0 && check_y < image_h) 
                    {
                        // 使用sobelCheckPoint检查该点
                        if (sobelCheckPoint(&mt9v03x_image[0][0], check_x, check_y, threshold)) 
                        {
                            right_valid = true;
                            break;
                        }
                    }
                }
            }
            
            // 如果两边都有效，保留该白条
            if (left_valid || right_valid) 
            {
                if (valid_strip_count < s) 
                {
                    // 将有效白条前移
                    whitearea[y][valid_strip_count] = whitearea[y][s];
                }
                valid_strip_count++;
            }
        }
        
        // 更新当前行的白条数量
        aindex[y] = valid_strip_count;
        
        
    }
} 


void row_col_check() // 通过行列元素跳变次数来判断元素，自行编写元素判断与处理
{
    uint8 i, j;
    jump_init();
    // 顶部元素行搜索 
    for (i = 1; i < 20; i += 3)
    {

        if(aindex[i]>1)
        {
            coord.top_jump_y = i; // 记录当前扫描行
            check.top_jump=4;
            if (flagdebug == 4)
            {
                ips114_draw_point(j, i, RGB565_RED);
            }
            break;
        }
        check.top_jump = 0; // 每行跳变计数器重置
        for (j = 0; j < image_w -1; j++)
        {
            if ((Image_use_zip[i][j] < thres && Image_use_zip[i][j + 1] >= thres && Image_use_zip[i][j + 2] >= thres) || (Image_use_zip[i][j] >= thres && Image_use_zip[i][j + 1] >= thres && Image_use_zip[i][j + 2] < thres))
            {
                check.top_jump++;
                coord.top_jump_x = j; // 记录最后跳变位置
            }
            if (flagdebug == 4)
            {
                ips114_draw_point(j, i, RGB565_RED);
            }
        }
        coord.top_jump_y = i; // 记录当前扫描行
        if (check.top_jump >= 2)
            break; // 跳变异常终止扫描
    }

    // 底部元素行搜索 
    for (i = image_h - 12; i > image_h - (12 + 10); i -= 2)
    {
        check.bottom_jump = 0; // 每行跳变计数器重置
        for (j = 0; j < image_w - 1; j++)
        {
            if ((Image_use_zip[i][j] < thres && Image_use_zip[i][j + 1] >= thres && Image_use_zip[i][j + 2] >= thres) || (Image_use_zip[i][j] >= thres && Image_use_zip[i][j + 1] >= thres && Image_use_zip[i][j + 2] < thres))
            {
                check.bottom_jump++;
                coord.bottom_jump_x = j; // 记录最后跳变位置
            }
            if (flagdebug == 4)
            {
                ips114_draw_point(j, i, RGB565_RED);
            }
        }
        coord.bottom_jump_y = i; // 记录当前扫描行
        if (check.bottom_jump > 2)
            break; // 跳变异常终止扫描
    }

    // 左部元素列搜索 (2-5列)
    for (j = 10; j < 26; j += 2)
    {
        check.left_jump = 0; // 每列跳变计数器重置
        for (i = image_h - 1; i > 1; i--)
        {
            if ((Image_use_zip[i][j] < thres && Image_use_zip[i - 1][j] >= thres && Image_use_zip[i - 2][j] >= thres) || (Image_use_zip[i][j] >= thres && Image_use_zip[i - 1][j] >= thres && Image_use_zip[i - 2][j] < thres))
            {
                check.left_jump++;
                coord.left_jump_y = i; // 记录最后跳变位置
            }
            if (flagdebug == 4)
            {
                ips114_draw_point(j, i, RGB565_RED);
            }
        }
        coord.left_jump_x = j; // 记录当前扫描列
        if (check.left_jump >= 2)
            break; // 跳变异常终止扫描
    }

    // 右部元素列搜索 (倒数5列)
    for (j = image_w - 15; j > image_w - 26; j -= 2)
    {
        check.right_jump = 0; // 每列跳变计数器重置
        for (i = image_h - 1; i > 1; i--)
        {
            if ((Image_use_zip[i][j] < thres && Image_use_zip[i - 1][j] >= thres && Image_use_zip[i - 2][j] >= thres) || (Image_use_zip[i][j] >= thres && Image_use_zip[i - 1][j] >= thres && Image_use_zip[i - 2][j] < thres))
            {
                check.right_jump++;
                coord.right_jump_y = i; // 记录最后跳变位置
            }
            if (flagdebug == 4)
            {
                ips114_draw_point(j, i, RGB565_RED);
            }
        }
        coord.right_jump_x = j; // 记录当前扫描列
        if (check.right_jump >= 2)
            break; // 跳变异常终止扫描
    }

    /********************************************专用于圆环的元素列***************************************/
    // 顶部元素行搜索 (1-5行)
    for (i = 20; i < 51; i += 6)
    {
        check.top_jump_circle = 0; // 每行跳变计数器重置
        for (j = 0; j < image_w - 1; j++)
        {
            if ((Image_use_zip[i][j] < thres && Image_use_zip[i][j + 1] >= thres && Image_use_zip[i][j + 2] >= thres) || (Image_use_zip[i][j] >= thres && Image_use_zip[i][j + 1] >= thres && Image_use_zip[i][j + 2] < thres))
            {
                check.top_jump_circle++;
                coord.top_jump_circle_x = j; // 记录最后跳变位置
            }
            if (flagdebug == 4)
            {
                ips114_draw_point(j, i, RGB565_YELLOW);
            }
        }
        coord.top_jump_circle_y = i; // 记录当前扫描行
        if (check.top_jump_circle > 2)
            break; // 跳变异常终止扫描
    }
    //左
    for (j =14; j < 55; j += 8)
    {
        check.left_jump_circle = 0; // 每列跳变计数器重置
        for (i = image_h - 1; i > 1; i--)
        {
            if ((Image_use_zip[i][j] < thres && Image_use_zip[i - 1][j] >= thres && Image_use_zip[i - 2][j] >= thres) || (Image_use_zip[i][j] >= thres && Image_use_zip[i - 1][j] >= thres && Image_use_zip[i - 2][j] < thres))
            {
                check.left_jump_circle++;
                coord.left_jump_circle_y = i; // 记录最后跳变位置
            }
            if (flagdebug == 4)
            {
                ips114_draw_point(j, i, RGB565_YELLOW);
            }
        }
        coord.left_jump_circle_x = j; // 记录当前扫描列
        if (check.left_jump_circle > 2)
            break; // 跳变异常终止扫描
    }

    // 右部元素列搜索 
    for (j = image_w - 14; j > image_w - 55; j -= 8)
    {
        check.right_jump_circle = 0; // 每列跳变计数器重置
        for (i = image_h - 1; i > 1; i--)
        {
            if ((Image_use_zip[i][j] < thres && Image_use_zip[i - 1][j] >= thres && Image_use_zip[i - 2][j] >= thres) || (Image_use_zip[i][j] >= thres && Image_use_zip[i - 1][j] >= thres && Image_use_zip[i - 2][j] < thres))
            {
                check.right_jump_circle++;
                coord.right_jump_circle_y = i; // 记录最后跳变位置
            }
            if (flagdebug == 4)
            {
                ips114_draw_point(j, i, RGB565_YELLOW);
            }
        }
        coord.right_jump_circle_x = j; // 记录当前扫描列
        if (check.right_jump_circle > 2)
            break; // 跳变异常终止扫描
    }
}

int adaptive_threshold_area(image_t *img, int block_size, int cli_value, uint8 x, uint8 y)
{
    int half = block_size / 2;
    int thres_value = 0;
    for (int dy = -half; dy <= half; dy++)
    {
        for (int dx = -half; dx <= half; dx++)
        {
            thres_value += pixel(img, x + dx, y + dy);
        }
    }
    thres_value /= block_size * block_size;
    thres_value += cli_value;

    return thres_value;
}

typedef struct image
{
    uint8 *data;
    uint8 width;
    uint8 height;
}image_t;

#define pixel(imag,x,y)  ((imag)->data[(clip(y,0,MT9V03X_H-1))*(imag)->width+(clip(x,0,MT9V03X_W))]) //鑾峰緱鍥惧儚鍦▁,y澶勭殑鍍忕礌锟??
#define  LINE_LENTH  (90)

case crossroad:
    {
        find_strips();
        if (aindex[1] == 1 && whitearea[1][0].leftx > 25 && whitearea[1][0].rightx < image_w - 25)
        {
            connect_angle_points(70, 89, (whitearea[1][0].leftx + whitearea[1][0].rightx) / 2, 1);
        }

        if (beepcount > 20)
        {
            gpio_toggle_level(P00_1); // 切换蜂鸣器状�?
            beepcount = 0;
        }

        unsigned char i;
        err = 0;
        sum_mid = 0;
        errorcount = 0;
        average_mid = 0;

        // 加权
        for (i = 89; i > 10; i--)
        {
            sum_mid += center_line[i] * mid_weight_c[i];
            errorcount += mid_weight_c[i];
        }
        average_mid = (int16)(sum_mid / errorcount);
        err = average_mid - (image_w / 2);

        err = moving_average_filter(history_err, ERR_SIZE, err); // 滑动平均滤波
        if (encoder.sum > 7000)
        {
            gpio_set_level(P00_1, 0);
            road_status = normal;
            encoder.sum = 0;
            encoder.flag = 0;
        }
        break;
    }
    
int top_judge_flag = 0; // 顶部元素判断标志
void top_judge()
{
    top_judge_flag = 0; // 初�?�化标志
    if (aindex[coord.top_jump_y] > 1)
    {
        for (int i = 0; i < aindex[coord.top_jump_y]; i++)
        {
            if (abs((whitearea[coord.top_jump_y][i].leftx + whitearea[coord.top_jump_y][i].rightx) / 2 - coord.bottom_jump_x) < 30)
            {
                top_judge_flag = 1;
                break;
            }
        }
    }
}

int crossroad_flag = 0;
int crossroad_flag_lr = 0;
void crossroad_judge()
{
    int judge_count = 0;
    int mid_x = (coord.top_jump_x + coord.bottom_jump_x) / 2;
    int y_diff = abs(coord.top_jump_y - coord.bottom_jump_y) / 2;
    for (int i = coord.top_jump_y; i <= coord.bottom_jump_y; i++)
    {
        for (int j = aindex[i]; j > 0; j--)
        {
            if (abs((whitearea[i][j - 1].leftx + whitearea[i][j - 1].rightx) / 2 - mid_x) < 15)
            {
                judge_count++;
                break;
            }
        }
    }
    if (judge_count > y_diff)
    {
        crossroad_flag = 1;
    }
    // printf("%d,%d,%d,%d,%d,%d,%d\n",judge_count,y_diff,mid_x,coord.top_jump_x,coord.bottom_jump_x,coord.top_jump_y,coord.bottom_jump_y);
}

// void crossroad_judge_lr() //矩形看白像素�?
// {
//     int sum_count = 0;
//     int judge_count = 0;
//     int sy=min(coord.left_jump_y,coord.right_jump_y);
//     int ey=max(coord.left_jump_y,coord.right_jump_y);
//     if(abs(sy-ey)<6)
//     {
//         ey =ey+6;
//     }
//     for(int i=coord.left_jump_x;i<=coord.right_jump_x;i++)
//     {
//         for(int j=sy;j<=ey;j++)
//         {
//             sum_count++;
//              if(mt9v03x_image[j][i]>thres)
//               {
//                   judge_count++;
//              }

//          // ips114_draw_point(i,j,RGB565_RED);
//         }
//     }
//      // ips114_show_int(20,135,sum_count,5);
//        // ips114_show_int(50,135,judge_count,5);
//     if(judge_count>sum_count/4)
//     {
//         crossroad_flag_lr = 1;
//     }

// }
void crossroad_judge_lr2() // 拟合�?
{
    crossroad_flag_lr = 0;
    connect_sum = 0;
    connect_judge = 0;
    // connect_crossroad(coord.left_jump_x, coord.left_jump_y, coord.right_jump_x, coord.right_jump_y);
    // connect_crossroad(coord.left_jump_x, coord.left_jump_y-1, coord.right_jump_x, coord.right_jump_y-1);
    connect_crossroad(coord.left_jump_x, coord.left_jump_y, coord.right_jump_x, coord.right_jump_y);
    connect_crossroad(coord.left_jump_x, coord.left_jump_y + 1, coord.right_jump_x, coord.right_jump_y + 1);
    connect_crossroad(coord.left_jump_x, coord.left_jump_y + 2, coord.right_jump_x, coord.right_jump_y + 2);
    connect_crossroad(coord.left_jump_x, coord.left_jump_y + 3, coord.right_jump_x, coord.right_jump_y + 3);
    connect_crossroad(coord.left_jump_x, coord.left_jump_y + 4, coord.right_jump_x, coord.right_jump_y + 4);
    if (flagdebug == 5)
    {
        ips114_show_int(20, 135, connect_sum, 5);
        ips114_show_int(50, 135, connect_judge, 5);
    }
    if (connect_judge > connect_sum / 3)
    {
        crossroad_flag_lr = 1;
    }
}
// void crossroad_judge_lr3()//统�?�一列上的白点数
// {
//         int sum_count = 0;
//     int judge_count = 0;
//     int sy=min(coord.left_jump_y,coord.right_jump_y);
//     int ey=max(coord.left_jump_y,coord.right_jump_y);
//     if(abs(sy-ey)<6)
//     {
//         ey =ey+6;
//     }
//     for(int i=coord.left_jump_x;i<=coord.right_jump_x;i++)
//     {
//         sum_count++;
//         for(int j=sy;j<=ey;j++)
//         {
//              if(mt9v03x_image[j][i]>thres)
//               {
//                   judge_count++;
//              }

//            if(flagdebug == 5)
//            ips114_draw_point(i,j,RGB565_RED);
//         }
//     }
//         if(flagdebug == 5)
//         {
//             ips114_show_int(20,135,sum_count,5);
//             ips114_show_int(50,135,judge_count,5);
//         }
//     if(judge_count>sum_count/4)
//     {
//         crossroad_flag_lr = 1;
//     }
// }

float distance(int x0, int y0, int x1, int y1)
{
    return sqrt(abs(x0 * x0 - x1 * x1) + abs(y0 * y0 - y1 * y1));
}

    //  top_judge(); // 顶部元素判断
    //  if (check.top_jump == 2  && check.left_jump == 2 && check.right_jump == 2 && check.bottom_jump == 2)
    //  {
    //     // gpio_set_level(P00_1,0);
    //      crossroad_judge();
    //       crossroad_judge_lr2();
    //      if (crossroad_flag == 1&& crossroad_flag_lr==1)
    //      {

    //         encoder.flag = 1;
    //         encoder.sum = 0;
    //         road_status = crossroad;
    //         crossroad_flag = 0;
    //         crossroad_flag_lr = 0;
    //         top_judge_flag = 0;
    //         gpio_set_level(P00_1,0);
    //         return;
    //     }
    // }