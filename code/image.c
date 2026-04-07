
#include "zf_common_headfile.h"
extern PID_Parma_KEY PID_KEY;
uint8 sight_range = 0;   // 视�?�范�??
uint8 range_step = 0;    // 视�?�范围�?�长
uint8 sight_range_c = 0; // 视�?�范�??
uint8 range_step_c = 0;  // 视�?�范围�?�长
uint8 flagdebug = 0;
float error = 0;
double oriimg_error = 0;
Imu imu;
uint8 road_status = normal;
uint8 thres = 0;
uint8 center_line[image_h]; // �???线数�???
uint8 sobel_Image[image_h][image_w];
#define ERR_SIZE 5
int16 history_err[ERR_SIZE]; // 历史�???�???

uint8 sobel_thres = 100;
uint8 fixed_thres = 0;

uint8 zimage[image_h][image_w] = {0};
int first_error = 0;

uint8 test_index = 0;
uint8 navstop_flag=0;
float nav_angle = 0;
bool nav_stf = true;
Connect_90 connect90 = {0, 0, 0, 0, 0};
//__________________________________________________________________工具函数_________________________________________________________//
int min(int a, int b)
{
    return ((a < b) ? a : b);
}
int max(int a, int b)
{
    return (a > b) ? a : b;
}

int my_abs(int value)
{
    if (value >= 0)
        return value;
    else
        return -value;
}

uint8 cal(uint8 a, uint8 b) // 计算�??比和
{
    return my_abs(a - b) * 100 / (a + b);
}

int clip(int x, int low, int up) // 限幅
{
    if (x > up)
        x = up;
    if (x < low)
        x = low;
    return x;
}

// 连接函数
void connect_angle_points(uint8 x0, uint8 y0, uint8 x1, uint8 y1) // 连线函数，x0，y0-底部跳变坐标   x1，y1-直�?��?�跳变坐�???
{
    int16 dx = my_abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int16 dy = -my_abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int16 error = dx + dy;

    // 计算最大可能的�???代�?�数（线长的2倍保证�?�盖�???
    int16 max_iter = 2 * (dx > -dy ? dx : -dy);

    // 使用for�???�???替代while
    for (int16 i = 0; i <= max_iter; i++)
    {
        // 将线上的有效点存入中线数�???
        if (y0 >= 0 && y0 < image_h && x0 >= 0 && x0 < image_w)
        {
            center_line[y0] = x0;
        }

        // 到达终点则提前退�???
        if (x0 == x1 && y0 == y1)
            break;

        int16 e2 = 2 * error;
        if (e2 >= dy)
        {
            error += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            error += dx;
            y0 += sy;
        }
    }
}

int connect_sum = 0;                                           // 连线点数统�??
int connect_judge = 0;                                         // 检测白�???
void connect_crossroad(uint8 x0, uint8 y0, uint8 x1, uint8 y1) // 连线函数，x0，y0-底部跳变坐标   x1，y1-直�?��?�跳变坐�???
{
    int16 dx = my_abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int16 dy = -my_abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int16 error = dx + dy;

    // 计算最大可能的�???代�?�数（线长的2倍保证�?�盖�???
    int16 max_iter = 2 * (dx > -dy ? dx : -dy);

    // 使用for�???�???替代while
    for (int16 i = 0; i <= max_iter; i++)
    {
        // 将线上的有效点存入中线数�???
        if (y0 >= 0 && y0 < image_h && x0 >= 0 && x0 < image_w)
        {

            connect_sum++;
            if (mt9v03x_image[y0][x0] > thres)
            {
                connect_judge++;
            }

            if (flagdebug == 5)
            {
                ips114_draw_point(x0, y0, RGB565_GREEN); // 绘制�???
                // ips114_draw_point(x0, y0+1, RGB565_RED);
                // ips114_draw_point(x0, y0+2, RGB565_RED);
                // ips114_draw_point(x0, y0+3, RGB565_RED);
            }
        }

        // 到达终点则提前退�???
        if (x0 == x1 && y0 == y1)
            break;

        int16 e2 = 2 * error;
        if (e2 >= dy)
        {
            error += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            error += dx;
            y0 += sy;
        }
    }
}

extern uint8 beepcount;

int16 history_err[ERR_SIZE] = {0, 0, 0, 0, 0}; // 历史�???�???
// 滑动平均滤波
/**
 * @brief 滑动平均滤波函数，�?�输入的数据进�?�滑动平均滤�??�???�理�???
 *
 * 该函数使用一�???固定大小的缓冲区来存储最近的输入数据，每次输入一�???新值时�???
 * 会更新缓冲区�???的数�???并重新�?�算平均值�?
 *
 * @param buffer 指向存储数据的缓冲区的指针�?
 * @param size 缓冲区的大小�???
 * @param new_value 新输入的数据值�?
 * @return int16 经过滑动平均滤波后的结果�???
 */
int16 moving_average_filter(int16 *buffer, uint8 size, int16 new_value)
{
    // 静态变量，用于记录当前要更新的缓冲区索�???
    static uint8 index = 0;
    // 静态变量，用于存储当前缓冲区中所有数�???的总和
    static int16 sum = 0;
    // 静态变量，用于存储当前缓冲区中数据的平均�?
    static int16 avg = 0;
    // 静态变量，用于标�?�缓冲区�???否已经初始化
    static uint8 is_initialized = 0;

    // 初�?�化sum和avg
    if (!is_initialized)
    {
        // 遍历缓冲区，�???加所有数�???到sum
        for (uint8 i = 0; i < size; i++)
        {
            sum += buffer[i];
        }
        // 计算缓冲区中数据的初始平均�?
        avg = sum / size;
        // 标�?�缓冲区已经初�?�化
        is_initialized = 1;
    }

    // 更新求和和索�???
    // 从总和�???减去当前索引位置的旧�???
    sum -= buffer[index];
    // 将新值存储到当前索引位置
    buffer[index] = new_value;
    // 将新值累加到总和�???
    sum += buffer[index];
    // 更新索引，使用取模运算确保索引在缓冲区范围内�???�???
    index = (index + 1) % size;

    // 计算新的平均�???
    avg = (int16)(sum / size);

    return avg;
}

//________________________________________________________陀螺仪_______________________________________________________________________________//

void gyro_summation()
{
    imu.angle_speed = tranced_gyro_z;
    imu.now_angle += imu.angle_speed;
    imu.angle_error = imu.now_angle - imu.aim_angle;
}

void imu_param_init()
{
    imu.aim_angle = 0;
    imu.angle_error = 0;
    imu.angle_speed = 0;
    imu.broken_road_flag = 0;
    imu.now_angle = 0;
}

//________________________________________________________图像预�?�理_______________________________________________________________________________//
uint8 stop_otsu_flag = 0;                                 // �???�???
uint8 image_thereshold;                                   // 图像分割阈�?
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row) // 限定了�?�图像中心底部进行统计，节约运算�???
{
#define GrayScale 256
    uint16 Image_Width = col;
    uint16 Image_Height = row;
    int X;
    uint16 Y;
    uint8 *data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack = 0, OmegaFore = 0, MicroBack = 0, MicroFore = 0, SigmaB = 0, Sigma = 0; // 类间方差;
    uint8 MinValue = 0, MaxValue = 0;
    uint8 Threshold = 0;

    for (Y = 60; Y < Image_Height; Y++) // Y<Image_Height改为Y =Image_Height；以便进�??? 行二值化,�???统�?�中间部�???
    {
        // Y=Image_Height;
        for (X = 46; X < Image_Width - 46; X++)
        {
            HistGram[(int)data[Y * Image_Width + X]]++; // 统�?�每�???灰度值的�???数信�???
        }
    }

    for (MinValue = 0; MinValue < 255 && HistGram[MinValue] == 0; MinValue++)
        ; // 获取最小灰度的�???
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--)
        ; // 获取最大灰度的�???

    if (MaxValue == MinValue)
    {
        return MaxValue; // 图像�???�???有一�???颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue; // 图像�???�???有二�???颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y]; //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y; // 灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack = PixelBack + HistGram[Y];                                               // 前景像素点数
        PixelFore = Amount - PixelBack;                                                    // 背景像素点数
        OmegaBack = (double)PixelBack / Amount;                                            // 前景像素百分�???
        OmegaFore = (double)PixelFore / Amount;                                            // 背景像素百分�???
        PixelIntegralBack += HistGram[Y] * Y;                                              // 前景灰度�???
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;                             // 背景灰度�???
        MicroBack = (double)PixelIntegralBack / PixelBack;                                 // 前景灰度百分�???
        MicroFore = (double)PixelIntegralFore / PixelFore;                                 // 背景灰度百分�???
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore); // g
        if (Sigma > SigmaB)                                                                // 遍历最大的类间方差g
        {
            SigmaB = Sigma;
            Threshold = (uint8)Y;
        }
    }
    if (Threshold < 45)
    {
        Threshold = 60;
    } // 阈值下限，�???行调�???
    return Threshold;
}
void sobelThreshold_new(uint8 *imageIn, uint8 *imageOut, uint8 Threshold)
{
    /** 卷积核大�??? */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = image_w - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = image_h - KERNEL_SIZE / 2;
    short i, j;
    short temp[2];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* 计算不同方向�???度幅�???  */
            temp[0] = -(short)imageIn[(i - 1) * image_w + j - 1] + (short)imageIn[(i - 1) * image_w + j + 1] - (short)imageIn[i * image_w + j - 1] + (short)imageIn[i * image_w + j + 1] - (short)imageIn[(i + 1) * image_w + j - 1] + (short)imageIn[(i + 1) * image_w + j + 1];

            temp[1] = -(short)imageIn[(i - 1) * image_w + j - 1] + (short)imageIn[(i + 1) * image_w + j - 1] - (short)imageIn[(i - 1) * image_w + j] + (short)imageIn[(i + 1) * image_w + j] - (short)imageIn[(i - 1) * image_w + j + 1] + (short)imageIn[(i + 1) * image_w + j + 1];

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);

            // /* 找出�???度幅值最大�?  */
            // if (temp[0] < temp[1])
            //     temp[0] = temp[1];

            if ((temp[0] + temp[1]) > Threshold)
                imageOut[i * image_w + j] = 255;
            else
                imageOut[i * image_w + j] = 0;
        }
    }
}

//________________________________________________________寻线_______________________________________________________________________________//

#define MIN_STRIP_WIDTH 4                // 白条最小�?�度
Whitestrip whitearea[image_h][10] = {0}; // 每�?�最多存�???10�???白条
int aindex[image_h] = {0};               // 每�?�白条数量统�???
bool sobel_filter(uint8_t y, uint8_t sx, uint8_t ex, uint8 strip_cnt)
{
    bool fl = false;
    bool fr = false;
    uint8 lb = 0;
    uint8 rb = 0;
    uint8 cur_y = y;
    // 边界计算优化（消除冗余判�??�??
    int star_col = (sx <= 6) ? 6 : sx;
    int end_col = (ex >= (MT9V03X_W - 6)) ? (MT9V03X_W - 6) : ex;
    star_col -= 5;
    end_col += 5;
    if (cur_y == image_h)
        cur_y--;
    if (cur_y == 0)
        cur_y++;
    for (int i = star_col; i < end_col; i++)
    {

        // 计算Sobel算子的x和y方向�???�???
        short gx = -(short)mt9v03x_image[cur_y - 1][i - 1] + (short)mt9v03x_image[cur_y - 1][i + 1] - (short)mt9v03x_image[cur_y][i - 1] + (short)mt9v03x_image[cur_y][i + 1] - (short)mt9v03x_image[cur_y + 1][i - 1] + (short)mt9v03x_image[cur_y + 1][i + 1];

        short gy = -(short)mt9v03x_image[cur_y - 1][i - 1] + (short)mt9v03x_image[cur_y + 1][i - 1] - (short)mt9v03x_image[cur_y - 1][i] + (short)mt9v03x_image[cur_y + 1][i] - (short)mt9v03x_image[cur_y - 1][i + 1] + (short)mt9v03x_image[cur_y + 1][i + 1];

        // 计算�???度幅值（简化版，使用绝对值之和代替平方根�???
        gx = abs(gx);
        gy = abs(gy);

        if (gx + gy > sobel_thres)
        {
            fl = true;
            lb = i;
            break;
        }
    }
    for (int i = end_col; i > star_col; i--)
    {

        // 计算Sobel算子的x和y方向�???�???
        short gx = -(short)mt9v03x_image[cur_y - 1][i - 1] + (short)mt9v03x_image[cur_y - 1][i + 1] - (short)mt9v03x_image[cur_y][i - 1] + (short)mt9v03x_image[cur_y][i + 1] - (short)mt9v03x_image[cur_y + 1][i - 1] + (short)mt9v03x_image[cur_y + 1][i + 1];

        short gy = -(short)mt9v03x_image[cur_y - 1][i - 1] + (short)mt9v03x_image[cur_y + 1][i - 1] - (short)mt9v03x_image[cur_y - 1][i] + (short)mt9v03x_image[cur_y + 1][i] - (short)mt9v03x_image[cur_y - 1][i + 1] + (short)mt9v03x_image[cur_y + 1][i + 1];

        // 计算�???度幅值（简化版，使用绝对值之和代替平方根�???
        gx = abs(gx);
        gy = abs(gy);

        if (gx + gy > sobel_thres)
        {
            fr = true;
            rb = i;
            break;
        }
    }
    if (fl && fr)
    {
        whitearea[y][strip_cnt].leftx = lb;
        whitearea[y][strip_cnt].rightx = rb;

        whitearea[y][strip_cnt].width = rb - lb + 1;
    }
}

static bool sobel_jump_check(uint8_t row, uint8_t col)
{
    // 边界保护
    if (row < 1 || row >= image_h - 1 || col < 1 || col >= image_w - 1)
        return false;

    short gx = -(short)mt9v03x_image[row - 1][col - 1] + (short)mt9v03x_image[row - 1][col + 1] - (short)mt9v03x_image[row][col - 1] + (short)mt9v03x_image[row][col + 1] - (short)mt9v03x_image[row + 1][col - 1] + (short)mt9v03x_image[row + 1][col + 1];

    short gy = -(short)mt9v03x_image[row - 1][col - 1] + (short)mt9v03x_image[row + 1][col - 1] - (short)mt9v03x_image[row - 1][col] + (short)mt9v03x_image[row + 1][col] - (short)mt9v03x_image[row - 1][col + 1] + (short)mt9v03x_image[row + 1][col + 1];

    gx = abs(gx);
    gy = abs(gy);

    if (gx + gy > sobel_thres)
        return true;
    else
        return false;
}
void find_strips()
{
    // 清空历史数据
    memset(aindex, 0, sizeof(aindex));
    memset(whitearea, 0, sizeof(whitearea));

    // 逐�?�扫�???
    for (int y = 0; y < image_h; y++)
    {

        bool in_strip = false;
        uint8_t start_x = 0;
        uint8_t strip_cnt = 0;

        // �???向扫描像�???
        for (int x = 0; x < image_w; x++)
        {
            // 灰度值转二值化判断
            uint8_t val = mt9v03x_image[y][x];
            bool is_white = (val >= fixed_thres);

            // 白条开始�?��?
            if (!in_strip && is_white)
            {
                start_x = x;
                in_strip = true;
            }
            // 白条结束检�???
            else if (in_strip && (!is_white || x == image_w - 1))
            {
                // 计算实际结束位置
                uint8_t end_x = (x == image_w - 1 && is_white) ? x : x - 1;

                // 过滤过窄的白�???
                if (end_x - start_x + 1 >= MIN_STRIP_WIDTH)
                {
                    // 存入白条数据
                    if (strip_cnt < 10)
                    {
                        sobel_filter(y, start_x, end_x, strip_cnt);

                        strip_cnt++;
                    }
                }
                in_strip = false;
            }
        }

        // 记录�???行白条数�???
        aindex[y] = strip_cnt;
    }
}

Whitestrip constrip[image_h][10] = {0}; // 存储合并后的白条
int conindex[image_h] = {0};            // 每�?�合并后的白条数�??

void connect_strip()
{
    // 遍历图像每一�??
    for (int y = 0; y < image_h; y++)
    {
        int cnt = aindex[y]; // 当前行原始白条数�??
        conindex[y] = 0;     // 重置当前行合并后的白条�?�数

        // 无白条�?�直接跳�??
        if (cnt == 0)
            continue;

        Whitestrip merged[10]; // 临时存储合并结果
        int merge_count = 0;
        merged[0] = whitearea[y][0]; // 初�?�化�??一�??白条

        // 合并相邻白条（间距≤30�??
        for (int j = 1; j < cnt; j++)
        {
            Whitestrip curr = whitearea[y][j];
            // 检查是否与上一�??白条相邻（间距≤30�??
            if (curr.leftx - merged[merge_count].rightx <= 30)
            {
                // 合并：更新右边界和�?�度
                merged[merge_count].rightx = curr.rightx;
                merged[merge_count].width = merged[merge_count].rightx - merged[merge_count].leftx + 1;
            }
            else
            {
                // 不相邻：添加新白�??
                merged[++merge_count] = curr;
            }
        }
        merge_count++; // 实际数量 = merge_count + 1

        // 存储合并结果到constrip
        for (int j = 0; j < merge_count; j++)
        {
            // 检查是否超过最大�?�量
            if (conindex[y] < 10)
            {
                constrip[y][conindex[y]] = merged[j];
                conindex[y]++;
            }
        }
    }
}

void print_line()
{
    // for (int i = 0; i < image_h; i++)
    // {
    //     ips114_draw_point((center_line[i]), (i), RGB565_GREEN);
    //     ips114_draw_point((center_line[i] - 1), (i), RGB565_GREEN);
    //     ips114_draw_point((center_line[i] + 1), (i), RGB565_GREEN);

    //     ips114_draw_point((center_line[i] - 2), (i), RGB565_GREEN);
    //     ips114_draw_point((center_line[i] + 2), (i), RGB565_GREEN);
    // }
    for (int i = 0; i < image_h; i++)
    {
        for (int j = 0; j < aindex[i]; j++)
        {
            ips114_draw_point((whitearea[i][j].leftx), (i), RGB565_GREEN);
            ips114_draw_point((whitearea[i][j].rightx), (i), RGB565_RED);
        }
    }
}

void print_midline()
{
    for (int i = 0; i < image_h; i++)
    {

        ips114_draw_point(clip((center_line[i]), 0, 140), (i), RGB565_BROWN);
        ips114_draw_point(clip((center_line[i] + 1), 0, 140), (i), RGB565_BROWN);
        ips114_draw_point(clip((center_line[i] - 1), 0, 140), (i), RGB565_BROWN);
    }
}

uint8 last_start_center = 70;
uint8 circle_flag = 0;
#define tar_l 0
#define tar_r 140

void getline_normal()
{
    uint8 bias_count = 0;
    uint8 misscont = 0;
    uint8 cont = 0;
    int flag = 0; // 0 正常 1 左直 2 右直
                  //   memset(&connect90, 0, sizeof(Connect_90));
    int hy = 87;  // 初�?�扫描�??
    while (hy >= 50)
    {
        if (aindex[hy] == 1 && whitearea[hy][0].width > 10 && whitearea[hy][0].width < 25)
        {
            last_start_center = (whitearea[hy][0].leftx + whitearea[hy][0].rightx) / 2;

            for (int i = hy; i < 90; i++)
                center_line[i] = last_start_center;

            break;
        }
        else
            hy--;
    }
    uint8 last_center = last_start_center;
    // 更新历史�???

    for (int y = hy - 1; y >= 0; y--)
    { // �??? image_h �??? 0
        // 情况1：无白条，继承上一次�?
        if (aindex[y] == 0)
        {

            switch (flag)
            {
            case 0: // 正常
                center_line[y] = last_center;
                misscont++;
                break;
            case 1: // 左直??
                center_line[y] = 0;
                
                misscont++;
                break;
            case 2: // 右直??
                center_line[y] = 140;
                
                misscont++;
                break;
            }
        }
        else
        {

            // 情况2�???1�???白条，取其中�???
            if (aindex[y] == 1)
            {

                Whitestrip *strip = &whitearea[y][0];
                uint8_t mid = (strip->leftx + strip->rightx) / 2;
                uint8_t width = strip->width;

                // 窄白条�?�理
                if (abs(mid - last_center) < 35 )
                {
                    misscont = 0;
                    center_line[y] = mid;
                    last_center = mid; // 更新历史�???
                    if (flag)          // 如果连续多�?�都正常，�?�明判断已经失效�??
                    {
                        cont++;
                        if (cont > 15)
                        {
                            flag = 0;
                            cont = 0;
                        }
                    }
                }
                else
                {

                    switch (flag)
                    {
                    case 0: // 正常
                        center_line[y] = last_center;
                        misscont++;

                        break;
                    case 1: // 左直??
                        center_line[y] = 0;
                        
                        misscont++;
                        break;
                    case 2: // 右直??
                        center_line[y] = 140;
                        
                        misscont++;
                        break;
                    }
                }

                // Whitestrip *strip = &whitearea[y][0];
                // uint8_t mid = (strip->leftx + strip->rightx) / 2;
                // center_line[y] = mid;
                // last_center = mid; // 更新历史�???
            }
            // 情况3�???2�???白条，选距离更近的
            else if (aindex[y] == 2)
            {

                Whitestrip *strip1 = &whitearea[y][0];
                Whitestrip *strip2 = &whitearea[y][1];
                uint8_t mid1 = (strip1->leftx + strip1->rightx) / 2;
                uint8_t mid2 = (strip2->leftx + strip2->rightx) / 2;

                // 计算参考值：�??5-10行的�??心线平均�??
                int ref_start = y + 5; // 下方�??5行（因y递减�??
                int ref_end = y + 10;  // 下方�??10�??
                int total = 0;
                int count = 0;

                // 边界检查：�??保不越界
                if (ref_start >= image_h)
                    ref_start = image_h - 1;
                if (ref_end >= image_h)
                    ref_end = image_h - 1;

                // 计算有效区间内的�??心线均�?
                if (ref_start <= ref_end)
                {
                    for (int i = ref_start; i <= ref_end; i++)
                    {
                        total += center_line[i];
                        count++;
                    }
                }

                int reference;
                if (count > 0)
                {
                    reference = total / count; // 历史�??心线均�?
                }
                else
                {
                    // 历史数据不足时：使用上一行或初�?��?
                    reference = (y < image_h - 1) ? center_line[y + 1] : last_center;
                }

                if (road_status == lcircle && circle_flag == 2)
                {
                    reference = reference + 5;
                    reference = clip(reference, 0, image_w - 1); // �??保参考值在有效范围�??
                }
                else if (road_status == rcircle && circle_flag == 2)
                {
                    reference = reference - 5;
                    reference = clip(reference, 0, image_w - 1); // �??保参考值在有效范围�??
                }

                // 选择距�?�参考值更近的白条�??�??
                uint8 diff1 = abs(mid1 - reference); // �??�?? int 避免溢出
                uint8 diff2 = abs(mid2 - reference);
                uint8 min_diff = (diff1 <= diff2) ? diff1 : diff2; // 计算最小距离差

                if (min_diff <= 25)
                {
                    center_line[y] = (diff1 <= diff2) ? mid1 : mid2;
                    if (flag) // 如果连续多�?�都正常，�?�明判断已经失效�??
                    {

                        cont++;
                        if (cont > 15)
                        {
                            flag = 0;
                            cont = 0;
                        }
                    }
                    last_center = center_line[y]; // 更新历史�??�??
                }
                else
                {
                    switch (flag)
                    {
                    case 0: // 正常
                        center_line[y] = last_center;
                        misscont++;
                        break;
                    case 1: // 左直??
                        center_line[y] = 0;
                        
                        misscont++;
                        break;
                    case 2: // 右直??
                        center_line[y] = 140;
                        
                        misscont++;
                        break;
                    }
                }
            }
            // 情况4：超�???2�???白条，继承历史�?
            else
            {
                center_line[y] = last_center;
            }
            // 先定�??? 这一行�??一�???白条的左边界 �??? left_wx �??? 最后一�???白条的右边界 right_wx
            // 如果两者之�???大于40 说明�???异常�??? 比较 left_wx  right_wx �??? 0 image_h 的距�???
            // 如果左边

            for (int i = 0; i < conindex[y]; i++)
            {
                Whitestrip strip = constrip[y][i];
                int strip_width = strip.rightx - strip.leftx;

                // 检验白条�?�度�??否满足直道条件（需同时满足宽度和位�??条件�??
                if (strip_width > 30)
                {
                    // 检验右直道：白条紧贴右边界且远离左边界
                    if (abs(strip.leftx - 0) > 40 && abs(image_w - strip.rightx) <= 30)
                    {
                        cont = 0;
                        if (flag == 1)
                        {
                            flag = 0;
                            for (int i = y; i > clip(y - 10, 0, image_h); i--)
                            {
                                center_line[i] = center_line[clip(y + 5, 0, image_h)];
                            }
                            y = clip(y - 9, 0, image_h);
                        }
                        else
                        {
                            flag = 2; // 右直�??
                            bias_count = 0;
                        }
                    }
                    // 检验左直道：白条紧贴左边界且远离右边界
                    else if (abs(strip.leftx - 0) <= 30 && abs(image_w - strip.rightx) > 40)
                    {

                        cont = 0;
                        if (flag == 2)
                        {
                            flag = 0;
                            for (int i = y; i > clip(y - 10, 0, image_h); i--)
                            {
                                center_line[i] = center_line[clip(y + 5, 0, image_h)];
                            }
                            y = clip(y - 9, 0, image_h);
                        }
                        else
                        {
                            flag = 1; // 左直�??
                            bias_count = 0;
                        }
                    }
                }
            }
        }
        if (misscont > 20)
        {

            for (int i = y; i >= 0; i--)
            {
                switch (flag)
                {
                case 0: // 正常
                    center_line[i] = last_center;
                    break;
                case 1: // 左直??
                    center_line[i] = 0;
                    break;
                case 2: // 右直??
                    center_line[i] = 140;
                    break;
                }
            }

            return;
        }
    }
}

//******************************************************************用于圆环的巡�??***************************************************************************** */
int flag = 0;
int left_wx;
int right_wx;
void getline_normal_c()
{
    uint8 bias_count = 0;
    uint8 misscont = 0;
    uint8 cont = 0;
    int flag = 0; // 0 ?? 1 ?? 2 ??
                  //   memset(&connect90, 0, sizeof(Connect_90));
    int hy = 87;  // ?????
    while (hy >= 50)
    {
        if (aindex[hy] == 1 && whitearea[hy][0].width > 10 && whitearea[hy][0].width < 25)
        {
            last_start_center = (whitearea[hy][0].leftx + whitearea[hy][0].rightx) / 2;

            for (int i = hy; i < 90; i++)
                center_line[i] = last_start_center;

            break;
        }
        else
            hy--;
    }
    uint8 last_center = last_start_center;
    // ??????

    for (int y = hy - 1; y >= 0; y--)
    { // ?? image_h ?? 0
        // ??1????????????
        if (aindex[y] == 0)
        {

            switch (flag)
            {
            case 0: // ??
                center_line[y] = last_center;
                misscont++;
                break;
            case 1: // ????
                center_line[y] = clip(last_center - bias_count * 10, 0, 140);
                bias_count++;
                misscont++;
                break;
            case 2: // ????
                center_line[y] = clip(last_center + bias_count * 10, 0, 140);
                bias_count++;
                misscont++;
                break;
            }
        }
        else
        {

            // ??2??1??????????
            if (aindex[y] == 1)
            {

                Whitestrip *strip = &whitearea[y][0];
                uint8_t mid = (strip->leftx + strip->rightx) / 2;
                uint8_t width = strip->width;

                if (road_status == lcircle)
                {

                    if (mid < last_center && abs(mid - last_center) < 35)
                    {
                        misscont = 0;
                        center_line[y] = mid;
                        last_center = mid; // ??????
                        if (flag)
                        {
                            cont++;
                            if (cont > 15)
                            {
                                flag = 0;
                                cont = 0;
                            }
                        }
                    }
                    else
                    {
                        center_line[y] = last_center;
                    }

                    // Whitestrip *strip = &whitearea[y][0];
                    // uint8_t mid = (strip->leftx + strip->rightx) / 2;
                    // center_line[y] = mid;
                    // last_center = mid; // ??????
                }

                else if (road_status == rcircle)
                {

                    if (mid > last_center && abs(mid - last_center) < 35)
                    {
                        misscont = 0;
                        center_line[y] = mid;
                        last_center = mid; // ??????
                        if (flag)
                        {
                            cont++;
                            if (cont > 15)
                            {
                                flag = 0;
                                cont = 0;
                            }
                        }
                    }
                    else
                    {
                        center_line[y] = last_center;
                    }
                }
            }

            // ??3??2???????????
            else if (aindex[y] == 2)
            {

                Whitestrip *strip1 = &whitearea[y][0];
                Whitestrip *strip2 = &whitearea[y][1];
                uint8_t mid1 = (strip1->leftx + strip1->rightx) / 2;
                uint8_t mid2 = (strip2->leftx + strip2->rightx) / 2;

                // ???????5-10????????
                int ref_start = y + 5; // ???5???y???
                int ref_end = y + 10;  // ???10?
                int total = 0;
                int count = 0;

                // ??????????
                if (ref_start >= image_h)
                    ref_start = image_h - 1;
                if (ref_end >= image_h)
                    ref_end = image_h - 1;

                // ?????????????
                if (ref_start <= ref_end)
                {
                    for (int i = ref_start; i <= ref_end; i++)
                    {
                        total += center_line[i];
                        count++;
                    }
                }

                int reference;
                if (count > 0)
                {
                    reference = total / count; // ???????
                }
                else
                {
                    // ?????????????????
                    reference = (y < image_h - 1) ? center_line[y + 1] : last_center;
                }

                if (road_status == lcircle && circle_flag == 2)
                {
                    reference = reference + 5;
                    reference = clip(reference, 0, image_w - 1); // ???????????
                }
                else if (road_status == rcircle && circle_flag == 2)
                {
                    reference = reference - 5;
                    reference = clip(reference, 0, image_w - 1); // ???????????
                }

                // ??????????????
                uint8 diff1 = abs(mid1 - reference); // ?? int ????
                uint8 diff2 = abs(mid2 - reference);
                uint8 min_diff = (diff1 <= diff2) ? diff1 : diff2; // ???????

                uint8 mid= (diff1 <= diff2) ? mid1 : mid2; // ???????
                if (road_status == lcircle)
                {
                    if (min_diff <= 25 && mid < last_center)
                    {

                        center_line[y] = mid;

                        if (flag) // ???????????????????
                        {

                            cont++;
                            if (cont > 15)
                            {
                                flag = 0;
                                cont = 0;
                            }
                        }
                        last_center = center_line[y]; // ??????
                    }
                    else
                    {
                        center_line[y] = last_center;
                    }
                }
                else if(road_status == rcircle)
                {
                    if (min_diff <= 25 && mid > last_center)
                    {

                        center_line[y] = mid;

                        if (flag) // ???????????????????
                        {

                            cont++;
                            if (cont > 15)
                            {
                                flag = 0;
                                cont = 0;
                            }
                        }
                        last_center = center_line[y]; // ??????
                    }
                    else
                    {
                        center_line[y] = last_center;
                    }
                }
            }
            // ??4????2???????????
            else
            {
                center_line[y] = last_center;
            }
                // ???? ??????????????? ?? left_wx ?? ??????????? right_wx
                // ?????????40 ???????? ?? left_wx  right_wx ?? 0 image_h ????
                // ????

                for (int i = 0; i < conindex[y]; i++)
                {
                    Whitestrip strip = constrip[y][i];
                    int strip_width = strip.rightx - strip.leftx;

                    // ????????????????????????????
                    if (strip_width > 30)
                    {
                        // ???????????????????
                        if (abs(strip.leftx - 0) > 40 && abs(image_w - strip.rightx) <= 30)
                        {
                            cont = 0;
                            if (flag == 1)
                            {
                                flag = 0;
                                for (int i = y; i > clip(y - 10, 0, image_h); i--)
                                {
                                    center_line[i] = center_line[clip(y + 5, 0, image_h)];
                                }
                                y = clip(y - 9, 0, image_h);
                            }
                            else
                            {
                                flag = 2; // ???
                                bias_count = 0;
                            }
                        }
                        // ???????????????????
                        else if (abs(strip.leftx - 0) <= 30 && abs(image_w - strip.rightx) > 40)
                        {

                            cont = 0;
                            if (flag == 2)
                            {
                                flag = 0;
                                for (int i = y; i > clip(y - 10, 0, image_h); i--)
                                {
                                    center_line[i] = center_line[clip(y + 5, 0, image_h)];
                                }
                                y = clip(y - 9, 0, image_h);
                            }
                            else
                            {
                                flag = 1; // ???
                                bias_count = 0;
                            }
                        }
                    }
                }
            }
        if (misscont > 20)
        {

            for (int i = y; i >= 0; i--)
            {
                switch (flag)
                {
                case 0: // ??
                    center_line[i] = last_center;
                    break;
                case 1: // ????
                    center_line[i] = 0;
                    break;
                case 2: // ????
                    center_line[i] = 140;
                    break;
                }
            }

            return;
        }
    }
}

void circle_out()
{
    uint8 bias_count = 0;
    uint8 cont = 0;
    int flag = 0;    // 0 正常 1 左直 2 右直
                     //   memset(&connect90, 0, sizeof(Connect_90));
    int hy = 87;     // 初�?�扫描�??
    while (hy >= 50) // 如果两个选择一�?? 但是如果20cm相邻赛道   �??以考虑限制范围来解�?? （暂时未限制8.15  11:34�??
    {

        if (aindex[hy] == 2) // 左环选取右边的白条做起�?�点 右环选取左边的白条作起�?�点
        {
            if (road_status == lcircle /*&& whitearea[hy][1].rightx <image_w - 1 - 45*/)
            {
                last_start_center = (whitearea[hy][1].leftx + whitearea[hy][1].rightx) / 2;
                for (int i = hy; i < 90; i++)
                {
                    center_line[i] = last_start_center;
                }

                break;
            }
            else if (road_status == rcircle /* && whitearea[hy][0].leftx > 45*/)
            {
                last_start_center = (whitearea[hy][0].leftx + whitearea[hy][0].rightx) / 2;
                for (int i = hy; i < 90; i++)
                {
                    center_line[i] = last_start_center;
                }

                break;
            }

            // else
            // {
            //     hy--;
            //     continue;
            // }
        }
        else
            hy--;
    }
    uint8 last_center = last_start_center;
    // 更新历史�???

    for (int y = hy - 1; y >= 0; y--)
    { // �??? image_h �??? 0
        // 情况1：无白条，继承上一次�?
        if (aindex[y] == 0)
        {

            switch (flag)
            {
            case 0: // 正常
                center_line[y] = last_center;
                break;
            case 1: // 左直??
                center_line[y] = clip(last_center - bias_count * 10, 0, 140);
                bias_count++;
                break;
            case 2: // 右直??
                center_line[y] = clip(last_center + bias_count * 10, 0, 140);
                bias_count++;
                break;
            }
        }
        else
        {

            // 情况2�???1�???白条，取其中�???
            if (aindex[y] == 1)
            {

                Whitestrip *strip = &whitearea[y][0];
                uint8_t mid = (strip->leftx + strip->rightx) / 2;
                uint8_t width = strip->width;

                // 圆环处理 圆环�??�??�??连续�??
                if (abs(mid - last_center) < 12)
                {
                    center_line[y] = mid;
                    last_center = mid; // 更新历史�???
                    if (flag)          // 如果连续多�?�都正常，�?�明判断已经失效�??
                    {
                        cont++;
                        if (cont > 15)
                        {
                            flag = 0;
                            cont = 0;
                        }
                    }
                }
                else
                {
                        center_line[y] = last_center;
                }

                // Whitestrip *strip = &whitearea[y][0];
                // uint8_t mid = (strip->leftx + strip->rightx) / 2;
                // center_line[y] = mid;
                // last_center = mid; // 更新历史�???
            }
            // 情况3�???2�???白条，选距离更近的
            else if (aindex[y] == 2)
            {

                Whitestrip *strip1 = &whitearea[y][0];
                Whitestrip *strip2 = &whitearea[y][1];
                uint8_t mid1 = (strip1->leftx + strip1->rightx) / 2;
                uint8_t mid2 = (strip2->leftx + strip2->rightx) / 2;

                // 选择距�?�参考值更近的白条�??�??
                uint8 diff1 = abs(mid1 - last_center); // �??�?? int 避免溢出
                uint8 diff2 = abs(mid2 - last_center);
                uint8 min_diff = (diff1 <= diff2) ? diff1 : diff2; // 计算最小距离差

                if (min_diff <= 25)
                {
                    center_line[y] = (diff1 <= diff2) ? mid1 : mid2;
                    if (flag) // 如果连续多�?�都正常，�?�明判断已经失效�??
                    {

                        cont++;
                        if (cont > 15)
                        {
                            flag = 0;
                            cont = 0;
                        }
                    }
                    last_center = center_line[y]; // 更新历史�??�??
                }
                else
                {
                    switch (flag)
                    {
                    case 0: // 正常
                        center_line[y] = last_center;

                        break;
                    case 1: // 左直??
                        center_line[y] = clip(last_center - bias_count * 10, 0, 140);
                        bias_count++;

                        break;
                    case 2: // 右直??
                        center_line[y] = clip(last_center + bias_count * 10, 0, 140);
                        bias_count++;
                        break;
                    }
                }
            }
            // 情况4：超�???2�???白条，继承历史�?
            else
            {
                center_line[y] = last_center;
            }
            // 先定�??? 这一行�??一�???白条的左边界 �??? left_wx �??? 最后一�???白条的右边界 right_wx
            // 如果两者之�???大于40 说明�???异常�??? 比较 left_wx  right_wx �??? 0 image_h 的距�???
            // 如果左边

            for (int i = 0; i < conindex[y]; i++)
            {
                Whitestrip strip = constrip[y][i];
                int strip_width = strip.rightx - strip.leftx;

                // 检验白条�?�度�??否满足直道条件（需同时满足宽度和位�??条件�??
                if (strip_width > 30)
                {
                    // 检验右直道：白条紧贴右边界且远离左边界
                    if (abs(strip.leftx - 0) > 40 && abs(image_w - strip.rightx) <= 30)
                    {
                        cont = 0;
                        if (flag == 1)
                        {
                            flag = 0;
                            for (int i = y; i > clip(y - 10, 0, image_h); i--)
                            {
                                center_line[i] = center_line[clip(y + 5, 0, image_h)];
                            }
                            y = clip(y - 9, 0, image_h);
                        }
                        else
                        {
                            flag = 2; // 右直�??
                            bias_count = 0;
                        }
                    }
                    // 检验左直道：白条紧贴左边界且远离右边界
                    else if (abs(strip.leftx - 0) <= 30 && abs(image_w - strip.rightx) > 40)
                    {

                        cont = 0;
                        if (flag == 2)
                        {
                            flag = 0;
                            for (int i = y; i > clip(y - 10, 0, image_h); i--)
                            {
                                center_line[i] = center_line[clip(y + 5, 0, image_h)];
                            }
                            y = clip(y - 9, 0, image_h);
                        }
                        else
                        {
                            flag = 1; // 左直�??
                            bias_count = 0;
                        }
                    }
                }
            }
        }
    }
}

//******************************************************************用于圆环的巡�??***************************************************************************** */
//________________________________________________________元素_______________________________________________________________________________//

void getline_incircle()
{

    uint8_t last_center = 70;
    if (road_status == lcircle)
    {
        for (int y = image_h - 1; y >= 0; y--)
        {
            if (aindex[y] == 2 /*&& whitearea[y][1].rightx <image_w - 1 - 45*/)
            {
                // 选左边白�??
                Whitestrip *strip = &whitearea[y][0];
                uint8_t mid = (strip->leftx + strip->rightx) / 2;
                center_line[y] = mid;
                last_center = mid;
            }
            // else if(aindex[y]==3)
            // {
            //     // 选左边白�??
            //     Whitestrip *strip = &whitearea[y][0];
            //     uint8_t mid = (strip->leftx + strip->rightx) / 2;
            //     center_line[y] = mid;
            //     last_center = mid;
            // }
            else
            {
                // 没有白条，继承上一�??
                center_line[y] = last_center;
            }
        }
    } // 计算�??�??
    else if (road_status == rcircle)
    {
        for (int y = image_h - 1; y >= 0; y--)
        {
            if (aindex[y] == 2 /*&& whitearea[y][0].leftx > 45*/)
            {
                // 选右边白�??
                Whitestrip *strip = &whitearea[y][1];
                uint8_t mid = (strip->leftx + strip->rightx) / 2;
                center_line[y] = mid;
                last_center = mid;
            }
            // else if(aindex[y]==3)
            // {
            //     // 选右边白�??
            //     Whitestrip *strip = &whitearea[y][2];
            //     uint8_t mid = (strip->leftx + strip->rightx) / 2;
            //     center_line[y] = mid;
            //     last_center = mid;
            // }
            else
            {
                // 没有白条，继承上一�??
                center_line[y] = last_center;
            }
        }
    }
    // 计算�??�??
}

void getline_outcircle_2()
{
    uint8_t last_center = 70;
    if (road_status == lcircle)
    {
        for (int y = image_h - 1; y >= 0; y--)
        {
            if (aindex[y] == 2)
            {
                // 选右边白�??
                Whitestrip *strip = &whitearea[y][1];
                uint8_t mid = (strip->leftx + strip->rightx) / 2;
                center_line[y] = mid;
                last_center = mid;
            }
            else
            {
                // 没有白条，继承上一�??
                center_line[y] = last_center;
            }
        }
    } // 计算�??�??
    else if (road_status == rcircle)
    {
        for (int y = image_h - 1; y >= 0; y--)
        {
            if (aindex[y] == 2)
            {
                // 选右边白�??
                Whitestrip *strip = &whitearea[y][0];
                uint8_t mid = (strip->leftx + strip->rightx) / 2;
                center_line[y] = mid;
                last_center = mid;
            }
            else
            {
                // 没有白条，继承上一�??
                center_line[y] = last_center;
            }
        }
    }
}



element_check check;
jump coord;
void jump_init()
{
    check.bottom_jump = 0;
    check.left_jump = 0;
    check.right_jump = 0;
    check.top_jump = 0;
    check.left_jump_circle = 0;
    check.right_jump_circle = 0;
    check.top_jump_circle = 0;
}
uint8 max_top = 0;
uint8 max_bot = 0;   // 底部元素行左边界
uint8 l_top = 0;     // 顶部元素行左边界
uint8 r_top = 0;     // 顶部元素行右边界
void row_col_check() // 通过行列元素跳变次数来判�???元素，自行编写元素判�???与�?�理
{
    uint8 i, j;
    jump_init();

    // 底部元素�??
    for (i = image_h - 1; i >= image_h - 1 - 27; i -= 9)
    {

        uint8_t current_jump = 0; // 当前行的跳变点数

        for (j = 0 + max_bot; j <= image_w - 1 - max_bot; j++)
        {
            if ((mt9v03x_image[i][j] < thres && mt9v03x_image[i][j + 1] >= thres && mt9v03x_image[i][j + 2] >= thres) ||
                (mt9v03x_image[i][j] >= thres && mt9v03x_image[i][j + 1] >= thres && mt9v03x_image[i][j + 2] < thres))
            {
                current_jump++;
            }
            if (flagdebug == 5)
            {
                ips114_draw_point(j, i, RGB565_YELLOW);
            }
        }
        // 更新最大跳变点�??
        if (current_jump > check.bottom_jump)

        {
            check.bottom_jump = current_jump;
        }
        if (check.bottom_jump > 2)
            break;
    }
    /********************************************专用于圆�??的元素列***************************************/
    // 顶部元素行搜
    for (i = 30; i <= 48; i += 6)
    {
        uint8_t current_jump = 0; // 当前行的跳变点数

        for (j = 0 + max_top; j <= image_w - 1 - max_top; j++)
        {
            if ((mt9v03x_image[i][j] < thres && mt9v03x_image[i][j + 1] >= thres && mt9v03x_image[i][j + 2] >= thres) ||
                (mt9v03x_image[i][j] >= thres && mt9v03x_image[i][j + 1] >= thres && mt9v03x_image[i][j + 2] < thres))
            {
                current_jump++;
            }

            if (flagdebug == 5)
            {
                ips114_draw_point(j, i, RGB565_YELLOW);
            }
        }
        // 更新最大跳变点�??
        if (current_jump > check.top_jump_circle)
        {
            check.top_jump_circle = current_jump;
        }
        if (check.top_jump_circle > 2)
            break; // 跳变异常终�??
    }
    // �??左列
    for (j = 45; j >= 29; j -= 8)
    {
        uint8_t current_jump = 0; // 当前列的跳变点数
        for (i = image_h - 1; i > l_top; i--)
        {
            if ((mt9v03x_image[i][j] < thres && mt9v03x_image[i - 1][j] >= thres && mt9v03x_image[i - 2][j] >= thres) || (mt9v03x_image[i][j] >= thres && mt9v03x_image[i - 1][j] >= thres && mt9v03x_image[i - 2][j] < thres))
            {
                current_jump++;
            }
            if (flagdebug == 5)
            {
                ips114_draw_point(j, i, RGB565_YELLOW);
            }
        }
        if (current_jump > check.left_jump_circle)
        {
            check.left_jump_circle = current_jump;
        }
        if (check.left_jump_circle > 2)
            break; // 跳变异常终�?�扫�???
    }

    // 右部元素列搜
    for (j = image_w - 1 - 45; j <= image_w - 1 - 29; j += 8)
    {
        uint8_t current_jump = 0; // 当前列的跳变点数
        for (i = image_h - 1; i > r_top; i--)
        {
            if ((mt9v03x_image[i][j] < thres && mt9v03x_image[i - 1][j] >= thres && mt9v03x_image[i - 2][j] >= thres) || (mt9v03x_image[i][j] >= thres && mt9v03x_image[i - 1][j] >= thres && mt9v03x_image[i - 2][j] < thres))
            {
                current_jump++;
            }
            if (flagdebug == 5)
            {
                ips114_draw_point(j, i, RGB565_YELLOW);
            }
        }
        if (current_jump > check.right_jump_circle)
        {
            check.right_jump_circle = current_jump;
        }
        if (check.right_jump_circle > 2)
            break; //
    }
}

void status_judge()
{
    uint8 stop_count = 0;
    bool count_flag = false;
    for (int y = image_h; y > 0; y--)
    {
        for (int i = 0; i < conindex[y]; i++)
        {
            Whitestrip strip = constrip[y][i];
            int strip_width = strip.rightx - strip.leftx;

            // 检验白条�?�度�??否满足直道条件（需同时满足宽度和位�??条件�??
            if (strip_width > 40)
            {
                count_flag = true;
            }
        }
        if (count_flag)
        {
            count_flag = false;
            stop_count++;
        }
    }

    if (stop_count > 50)
    {
        if (memory_f)
        {
            

            N.wholeendindex = N.wholesize;
            N.wholeendpage = N.whole_page_index;
            if (flash_check(0, N.whole_page_index))
                flash_erase_page(0, N.whole_page_index);
            flash_write_page_from_buffer(0, N.whole_page_index, FLASH_PAGE_LENGTH); // 没有满页数，但是需要写�??


            flash_index_write();
            uint16 Col = 0;
            show_string(0, Col, "wholeendindex");
            show_float(8 * sizeof("wholeendindex"), Col, N.wholeendindex, 3, 1);
            Col += 1;
            show_string(0, Col, "wholeendpage");
            show_float(8 * sizeof("wholeendpage"), Col, N.wholeendpage, 3, 1);
            memory_f=0;
        }

        PID.stop_flag = 1;
        road_status = stop;
        gpio_set_level(P00_1, 1);
        return;
    }

    int nav_cnt = 0;

    if(!nav_stf&&encoder.sum>8000)
    {
        nav_stf=true;
        encoder.sum=0;
        encoder.flag=0;

    }
    if(nav_stf)
    {
        for (int i = 89; i > 49; i--)
        {
            if (aindex[i] == 0)
                nav_cnt++;
        }
        if (nav_cnt == 40)
        {
            N.Mileage_All = 0;
            Q_info.q0 = 1;
            Q_info.q1 = 0;
            Q_info.q2 = 0;
            Q_info.q3 = 0;
            eulerAngle.Dirchange = 0;
            eulerAngle.last_yaw = 0;

            N.Nag_SystemRun_Index = 2;
            N.Flash_read_f = 0;

            if (!nav_record_flag && test_index)
            {
                N.size = 0;
                N.datapage_index = (test_index - 1) * 2;
            }
            else
            {
                N.size = 0;
                N.datapage_index = N.nowelecount * 2;
            }

            road_status = cut;
            gpio_set_level(P00_1, 1); // 停�?�巡�??
            nav_stf=false;
            if(navstop_flag)
            {
                PID.stop_flag = 1; // 停�??PID
            }
            
            return;
        }
    }
    

    // if (check.top_jump_circle == 4 && check.left_jump_circle >= 2 && check.right_jump_circle == 0 && check.bottom_jump == 4)
    // {
    //     imu.broken_road_flag = 1;
    //     imu.now_angle = 0;
    //     road_status = lcircle;
    //     gpio_set_level(P00_1, 1);
    //     //  PID.stop_flag = 1; // 停�??PID

    //     return;
    // }

    // if (check.top_jump_circle == 4 && check.left_jump_circle == 0 && check.right_jump_circle >= 2 && check.bottom_jump == 4)
    // {
    //     imu.broken_road_flag = 1;
    //     imu.now_angle = 0;
    //     road_status = rcircle;
    //     gpio_set_level(P00_1, 1);
    //     // PID.stop_flag = 1; // 停�??PID
    //     return;
    // }
}
uint8 mid_weight[90] = {
    1, 1, 1, 1, 10000, 10000, 10000, 10000, 10000, 10000,                                     // 倒数81-90�???
    6 + 80, 8 + 80, 9 + 80, 10 + 80, 12 + 80, 12 + 80, 14 + 80, 17 + 80, 20 + 80, 20 + 80,    // 倒数71-80�???
    23 + 80, 25 + 80, 23 + 80, 23 + 80, 21 + 80, 20 + 80, 20 + 80, 18 + 80, 18 + 80, 15 + 80, // 倒数61-70�???
    8, 6, 5, 1, 1, 1, 1, 1, 1, 1,                                                             // 倒数51-60�???
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,                                                             // 倒数41-50�???
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,                                                             // 倒数31-40�???
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,                                                             // 倒数21-30�???
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,                                                             // 倒数11-20�???
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1};                                                            // 倒数1-10�???

uint8 mid_weight_c[90] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 倒数81-90�??    1  -10
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    25, 25, 22, 20 + 1, 17 + 10, 16 + 10, 15 + 10, 15 + 10, 13 + 10, 10 + 10,                // 倒数61-70�??    21
    25 + 10, 25 + 10, 22 + 10, 20 + 1, 17 + 10, 16 + 10, 15 + 10, 15 + 10, 13 + 10, 10 + 10, // 倒数21-30�?? //倒数51-60�??    31
    25, 25, 22, 20 + 1, 17 + 10, 16 + 10, 15 + 10, 15 + 10, 13 + 10, 10 + 10,                // 倒数41-50�?? 41
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,                                                            // 倒数31-40�?? 51
    8, 6, 5, 2, 1, 1, 1, 1, 1, 1,                                                            // 倒数21-30�??
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,                                                            // 倒数11-20�??
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1};                                                           // 倒数1-10�???
int16 err;
int32 sum_mid;
int32 errorcount;
int32 average_mid;
static bool state_entered[3] = {false};
int out_sum = 0; // 出环�??迹时的编码器计数阈�?
uint8 cir_plan = 0;
void status_control()
{
    switch (road_status)
    {
    case normal:
    {
        gpio_set_level(P00_1, 0); // 关闭蜂鸣�??
        getline_normal();
        unsigned char i;
        err = 0;
        sum_mid = 0;
        errorcount = 0;
        average_mid = 0;
        // 加权
        for (i = sight_range; i < sight_range + range_step; i++)
        {
            sum_mid += center_line[i];
            errorcount++;
        }
        average_mid = (int16)(sum_mid / errorcount);
        err = average_mid - (image_w / 2);

        // err=moving_average_filter(history_err,ERR_SIZE,err);//滑动平均滤波

        break;
    }
        /*************************************************优化日志********************************************************** */
        /*{版本1，小圆环，选择调参；并且�?�果赛道过近出环拉线�??能会错拉；a,拉线仍然�??取，选择恰当的点和手动限制范围（�??尝试）；b,出环�??迹，�??尝试，感觉有bug，但�??�??以采用只能往一�??方向长，多分几个状�?
           版本2, 40cm大环加虚线，出环点不合适，判断�??�?? ；判�??�??题解决，代码�??存在的一些小�??题，新赛道可以再推一�??；出�??点特征丢失是由于虚线，需要鲁棒性，不能乱。后面加了编码器强制出环
           版本3�?? 华东正�?�赛道，�??题较小，存在历史遗留�??题，但注意�?�不要在�??�??题时把之前�?�其他版�??的问题给反�?�修复了 }
           8.13 拉线拉一段距�?? 后面脱�?�危险用�??通循�??   不能一直拉�?? 会拉�?? 考虑最左边

           虚线





           */
    case lcircle: // 出环版本
    {

        if (beepcount > 2)
        {
            gpio_toggle_level(P00_1); // 切换蜂鸣器状
            beepcount = 0;
        }
        if (imu.now_angle > -40 && !state_entered[0])
        {
            circle_flag = 0;         // 入环�??�??
            state_entered[0] = true; // 标�?�状态已进入
        }
        else if (!state_entered[1] && imu.now_angle < -40 && imu.now_angle > -300)
        {
            circle_flag = 1;         // �??内循�??
            last_start_center = 70;  // 重置入环时的�??心线
            state_entered[1] = true; // 标�?�状态已进入
        }
        else if (!state_entered[2] && imu.now_angle < -300 && ((check.top_jump_circle > 2) || (check.bottom_jump > 2)))
        {
            circle_flag = 2;         // 出环�??�??
            state_entered[2] = true; // 标�?�状态已进入
            encoder.flag = 1;        // 出环�??迹时开�??编码�??
            encoder.sum = 0;         // 重置编码�?�??�数
        }

        switch (circle_flag)
        {
        case 0: // 入环�??�??
        {
            getline_incircle(); // 进环阶�??
            // unsigned char i;
            // err = 0;
            // sum_mid = 0;
            // errorcount = 0;
            // average_mid = 0;
            // // 加权
            // for (i = sight_range_c; i < sight_range_c + range_step_c; i++)
            // {
            //     sum_mid += center_line[i];
            //     errorcount++;
            // }
            // average_mid = (int16)(sum_mid / errorcount);
            // err = average_mid - (image_w / 2);
            break;
        }
        case 1: // �??内循�??
        {
            getline_normal_c();
            // unsigned char i;
            // err = 0;
            // sum_mid = 0;
            // errorcount = 0;
            // average_mid = 0;
            // // 加权
            // for (i = sight_range_c; i < sight_range_c + range_step_c; i++)
            // {
            //     sum_mid += center_line[i];
            //     errorcount++;
            // }
            // average_mid = (int16)(sum_mid / errorcount);
            // err = average_mid - (image_w / 2);
            break;
        }
        case 2: // 出环�??�??
        {
            if (cir_plan == 1)
            {
                getline_normal();
            }
            else if (cir_plan == 2)
            {
                getline_outcircle_2();
            }
            else if (cir_plan == 3)
            {
                circle_out(); // 出环阶�??
            }
            if (encoder.sum > out_sum)
            {
                gpio_set_level(P00_1, 0);
                imu.broken_road_flag = 0;
                road_status = normal;
                circle_flag = 0;                                 // 重置�??内循迹标�??
                memset(state_entered, 0, sizeof(state_entered)); // 解锁所有状�??
                encoder.flag = 0;                                // 出环�??迹时关闭编码�??
            }
            // unsigned char i;
            // err = 0;
            // sum_mid = 0;
            // errorcount = 0;
            // average_mid = 0;
            // // 加权
            // for (i = sight_range_c; i < sight_range_c + range_step_c; i++)
            // {
            //     if (abs(center_line[i] - 70) < 45)

            //     {
            //         sum_mid += center_line[i];
            //         errorcount++;
            //     }
            // }
            // if (errorcount == 0)
            // {
            //     errorcount = 1; // 防�?�除0错�??
            // }
            // average_mid = (int16)(sum_mid / errorcount);
            // err = average_mid - (image_w / 2);
            // if (errorcount == 0)
            // {
            //     err = 0; // 如果没有有效数据，�??�??设为0
            // }
            break;
        }

        }
        unsigned char i;
        err = 0;
        sum_mid = 0;
        errorcount = 0;
        average_mid = 0;
        // 加权
        for (i = sight_range_c; i < sight_range_c + range_step_c; i++)
        {
            sum_mid += center_line[i];
            errorcount++;
        }
        average_mid = (int16)(sum_mid / errorcount);
        err = average_mid - (image_w / 2);
        break;
    }

    case rcircle: // 出环版本
    {
        if (!state_entered[0] && imu.now_angle < 40)
        {
            circle_flag = 0;         // 入环�??�??
            state_entered[0] = true; // 标�?�状态已进入
        }

        else if (!state_entered[1] && imu.now_angle > 40 && imu.now_angle < 300)
        {
            circle_flag = 1;         // �??内循�??
            last_start_center = 70;  // 重置入环时的�??心线
            state_entered[1] = true; // 标�?�状态已进入
        }
        else if (!state_entered[2] && imu.now_angle > 300 && ((check.top_jump_circle > 2) || (check.bottom_jump > 2)))
        {
            circle_flag = 2;         // 出环�??�??
            state_entered[2] = true; // 标�?�状态已进入
            encoder.flag = 1;        // 出环�??迹时开�??编码�??
            encoder.sum = 0;         // 重置编码�?�??�数
        }

        switch (circle_flag)
        {
        case 0: // 入环�??�??
        {
            getline_incircle(); // 进环阶�??      
            //   unsigned char i;
            // err = 0;
            // sum_mid = 0;
            // errorcount = 0;
            // average_mid = 0;
            // // 加权
            // for (i = sight_range_c; i < sight_range_c + range_step_c; i++)
            // {
            //     sum_mid += center_line[i];
            //     errorcount++;
            // }
            // average_mid = (int16)(sum_mid / errorcount);
            // err = average_mid - (image_w / 2);
            break;
        }
        case 1: // �??内循�??
        {
            getline_normal_c();
            // unsigned char i;
            // err = 0;
            // sum_mid = 0;
            // errorcount = 0;
            // average_mid = 0;
            // // 加权
            // for (i = sight_range_c; i < sight_range_c + range_step_c; i++)
            // {
            //     sum_mid += center_line[i];
            //     errorcount++;
            // }
            // average_mid = (int16)(sum_mid / errorcount);
            // err = average_mid - (image_w / 2);
            break;
        }
        case 2:
        {
            if (cir_plan == 1)
            {
                getline_normal();
            }
            else if (cir_plan == 2)
            {
                getline_outcircle_2();
            }
            else if (cir_plan == 3)
            {
                circle_out(); // 出环阶�??
            }
            if (encoder.sum > out_sum) // 强制出环，最后保底；
            {
                gpio_set_level(P00_1, 0);
                imu.broken_road_flag = 0;
                road_status = normal;
                circle_flag = 0;                                 // 重置�??内循迹标�??
                memset(state_entered, 0, sizeof(state_entered)); // 解锁所有状�??
                encoder.flag = 0;                                // 出环�??迹时关闭编码�??
            }
            // unsigned char i;
            // err = 0;
            // sum_mid = 0;
            // errorcount = 0;
            // average_mid = 0;
            // // 加权
            //  for (i = sight_range_c; i < sight_range_c + range_step_c; i++)
            // {
            //     if (abs(center_line[i] - 70) < 45)

            //     {
            //         sum_mid += center_line[i];
            //         errorcount++;
            //     }
            // }
            // if (errorcount == 0)
            // {
            //     errorcount = 1; // 防�?�除0错�??
            // }
            // average_mid = (int16)(sum_mid / errorcount);
            // err = average_mid - (image_w / 2);
            // if (errorcount == 0)
            // {
            //     err = 0; // 如果没有有效数据，�??�??设为0
            // }
            break;
        }
        }
        unsigned char i;
        err = 0;
        sum_mid = 0;
        errorcount = 0;
        average_mid = 0;
        // 加权
        for (i = sight_range_c; i < sight_range_c + range_step_c; i++)
        {
            sum_mid += center_line[i];
            errorcount++;
        }
        average_mid = (int16)(sum_mid / errorcount);
        err = average_mid - (image_w / 2);

        break;
    }

    case cut:
    {

        // gpio_set_level(P00_1, 0);

        break;
    }
    }

    oriimg_error = err;
}

void image_debug()
{

    /*
    0       正常�???
    1       串口发送原图给上位�???
    2       屏幕显示原图
    3       大津�???
    4       sobel
    5       固定阈�?
    6       final
    7       �??�??
    eg.  ips114_show_gray_image(0, 0,mt9v03x_image[0] , MT9V03X_W, MT9V03X_H,MT9V03X_W, MT9V03X_H, 0);//ori_img.data


    uint16                  ips114_width_max    = 240;
    uint16                  ips114_height_max   = 135;


    */

    switch (flagdebug)
    {
    case 1:
    {
        seekfree_assistant_camera_send();

        break;
    }
    case 2:
    {
        ips114_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0); // ori_img.data
        print_line();
        show_string(0, 7, "exposure");
        show_float(8 * sizeof("exposure"), 7, exposure, 3, 1);
        break;
    }
    case 3: // otus
    {
        uint8 otus_thres = otsuThreshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
        ;
        ips114_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, otus_thres);
        show_string(0, 8, "otus_thres");
        show_float(8 * sizeof("otus_thres"), 8, otus_thres, 3, 1);
        break;
    }
    case 4: // sobel
    {

        sobelThreshold_new(mt9v03x_image[0], sobel_Image[0], sobel_thres);
        ips114_show_gray_image(0, 0, sobel_Image[0], image_w, image_h, image_w, image_h, 0); // ori_img.data
        print_line();

        show_string(0, 8, "sobel_thres");
        show_float(8 * sizeof("sobel_thres"), 8, sobel_thres, 3, 1);

        break;
    }
    case 5: // fix
    {

        ips114_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, fixed_thres);
        ips114_show_float(20, 134, error, 2, 3);
        ips114_show_float(120, 134, imu.now_angle, 4, 2);
        ips114_show_uint(70, 134, fixed_thres, 3);
        ips114_show_uint(20, 135, check.top_jump_circle, 1);
        ips114_show_uint(50, 135, check.bottom_jump, 1);
        ips114_show_uint(80, 135, check.left_jump_circle, 1);
        ips114_show_uint(110, 135, check.right_jump_circle, 1);
        ips114_show_uint(160, 135, road_status, 1);
        ips114_show_uint(180, 134, encoder.sum, 8);
        print_midline();
        break;
    }
    case 6:
    {

        for (int i = 0; i < image_h; i++)
        {
            for (int j = 0; j < aindex[i]; j++)
            {
                for (int k = whitearea[i][j].leftx; k < whitearea[i][j].rightx; k++)
                {
                    zimage[i][k] = 255;
                }
            }
        }
        ips114_show_gray_image(0, 0, zimage[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0); // ori_img.data
        memset(zimage, 0, sizeof(zimage));

        ips114_show_uint(120, 134, sobel_thres, 3);
        ips114_show_uint(160, 134, fixed_thres, 3);

        // print_line();
        print_midline();

        ips114_show_float(120, 135, error, 2, 3);
        ips114_show_uint(160, 135, road_status, 3);

        for (int i = sight_range; i < sight_range + range_step; i++)
        {
            ips114_draw_point((center_line[i]), (i), RGB565_RED);
            ips114_draw_point((center_line[i] - 1), (i), RGB565_RED);
            ips114_draw_point((center_line[i] + 1), (i), RGB565_RED);
            ips114_draw_point((center_line[i] - 2), (i), RGB565_RED);
            ips114_draw_point((center_line[i] + 2), (i), RGB565_RED);
        }

        break;
    }
    case 7:
    {
        uint16 Col = 0;

        show_string(0, Col, "stat");
        show_float(8 * sizeof("stat"), Col, road_status, 3, 1);
        show_string(100, Col, "nowelecount");
        show_float(100 + 8 * sizeof("nowelecount"), Col, N.nowelecount, 3, 1);

        Col += 1;
        show_string(0, Col, "index");
        show_float(8 * sizeof("index"), Col, N.size, 3, 1);
        show_string(100, Col, "page_index");
        show_float(100 + 8 * sizeof("page_index"), Col, N.datapage_index, 3, 1);

        Col += 1;
        show_string(0, Col, "tar_idx");
        show_float(8 * sizeof("tar_idx"), Col, N.endindex[N.nowelecount], 3, 1);
        show_string(100, Col, "tarendpage");
        show_float(100 + 8 * sizeof("tarendpage"), Col, N.endpage[N.nowelecount], 3, 1);

        Col += 1;
        show_string(0, Col, "error");
        show_float(8 * sizeof("error"), Col, error, 3, 1);
        show_string(100, Col, "judge_error");
        show_float(100 + 8 * sizeof("judge_error"), Col, N.judge_error, 3, 1);

        Col += 1;
        show_string(0, Col, "wendidx");
        show_float(8 * sizeof("wendidx"), Col, N.wholeendindex, 3, 1);
        show_string(100, Col, "wendp");
        show_float(100 + 8 * sizeof("wendp"), Col, N.wholeendpage, 3, 1);
        Col += 1;
        show_string(0, Col, "wsize");
        show_float(8 * sizeof("wsize"), Col, N.wholesize, 3, 1);
        show_string(100, Col, "wpage");
        show_float(100 + 8 * sizeof("wpage"), Col, N.whole_page_index, 3, 1);

        break;
    }
    case 8:
    {
        break;
    }
    }
    // ips114_show_uint(160, 135, PID.stop_flag, 3);
    // ips114_show_uint(160, 135, thres, 3);

    // print_line();
    //      ips114_draw_line(0, 0, 139, 0, RGB565_RED);
    //      ips114_draw_line(0, 10, 139, 10, RGB565_RED);
    //      ips114_draw_line(0, 20, 139, 20, RGB565_RED);
    //      ips114_draw_line(0, 30, 139, 30, RGB565_RED);
    //      ips114_draw_line(0, 40, 139, 40, RGB565_RED);
    //      ips114_draw_line(0, 50, 139, 50, RGB565_RED);
    //      ips114_draw_line(0, 60, 139, 60, RGB565_RED);
    //      ips114_draw_line(0, 70, 139, 70, RGB565_RED);
    //      ips114_draw_line(0, 80, 139, 80, RGB565_RED);
    //      ips114_show_uint(0, 115, coord.bottom_jump_x, 3);
    //      ips114_show_uint(30, 115, coord.bottom_jump_y, 3);
    //      ips114_show_uint(60, 115, coord.left_jump_x, 3);
    //      ips114_show_uint(90, 115, coord.left_jump_y, 3);
    //      ips114_show_uint(120, 115, coord.right_jump_x, 3);
    //      ips114_show_uint(150, 115, coord.right_jump_y, 3);
    //      ips114_show_uint(180, 115, coord.top_jump_x, 3);
    //      ips114_show_uint(210, 115, coord.top_jump_y, 3);

    //     // ips114_show_uint(0, 135, check.bottom_jump, 3);
    //     // ips114_show_uint(30, 135, check.left_jump, 3);
    //     // ips114_show_uint(60, 135, check.right_jump, 3);
    //     // ips114_show_uint(90, 135, check.top_jump, 3);

    // ips114_show_float(0, 135, eulerAngle.Dirchange,3,2);
    // ips114_show_float(40, 135, eulerAngle.yaw, 3,2);
    // ips114_show_int(0, 135, encoder.sum, 4);

    //   ips114_show_float(0, 135,distance(coord.left_jump_x,coord.left_jump_y,coord.bottom_jump_x,coord.bottom_jump_y),2,2);
}
static void draw_border(void) // 画黑色边�??
{
    // 绘制顶部
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < MT9V03X_W; j++)
        {
            mt9v03x_image[i][j] = 0;
        }
    }
    // 绘制底部
    for (int i = MT9V03X_H - 3; i < MT9V03X_H; i++)
    {
        for (int j = 0; j < MT9V03X_W; j++)
        {
            mt9v03x_image[i][j] = 0;
        }
    }

    // 绘制左侧
    for (int i = 0; i < MT9V03X_H; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            mt9v03x_image[i][j] = 0;
        }
    }

    // 绘点制右�??
    for (int i = 0; i < MT9V03X_H; i++)
    {
        for (int j = MT9V03X_W - 3; j < MT9V03X_W; j++)
        {
            mt9v03x_image[i][j] = 0;
        }
    }
}

void image_process()
{

    thres = fixed_thres; // otsuThreshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
    find_strips();
    connect_strip();
    row_col_check();
    image_debug();

    if (aindex[0] == 1)
    {
        int first_mid = (whitearea[0][0].leftx + whitearea[0][0].rightx) / 2;
        first_error = first_mid - (image_w / 2);
    }
    else
    {
        first_error = 20;
    }

    if (road_status == normal)
    {
        status_judge();
    }
    status_control();
}
