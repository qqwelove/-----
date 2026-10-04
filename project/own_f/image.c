#include "image.h"

/*图像缓冲数组*/
static uint8 xdata display_buffer[MT9V03X_H][MT9V03X_W];

static uint8 image_threshold = 0;
static uint16 frame_cnt = 0, image_fps = 0;

static uint8 adapt_threshold(uint8 xdata *image, uint16 col, uint16 row);


void image_frame_done(void)
{
    mt9v03x_finish_flag = 0;        /* 全部处理完才放行下一帧 */
}


//有新帧并已存好快照返回 1，处理完必须调 image_frame_done()
uint8 image_update(void)
{
    if(!mt9v03x_finish_flag) return 0;          /* 没有新帧就直接返回，main 不用管标志位 */

    image_threshold = adapt_threshold((uint8 xdata *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
    memcpy(display_buffer, mt9v03x_image, sizeof(display_buffer));
    

    return 1;
}


void image_show(void)
{
#if IMG_SHOW_ENABLE
    ips200_show_gray_image(0, 0, display_buffer[0],
                           MT9V03X_W, MT9V03X_H,
                           CAMERA_DIS_W, CAMERA_DIS_H,
                           adapt_threshold(display_buffer[0], MT9V03X_W, MT9V03X_H));
#endif
}


uint8 image_get_threshold(void) { return image_threshold; }


void image_draw_point(uint16 row, uint16 col, uint8 gray)
{
    if(row < MT9V03X_H && col < MT9V03X_W) display_buffer[row][col] = gray;
}


static uint8 adapt_threshold(uint8 xdata *image, uint16 col, uint16 row)
{
    int   pixelCount[256];
    float pixelPro[256];
    uint16 i, j;
    uint32 gray_sum = 0, sample_num = 0;
    float  w0 = 0, w1, u0tmp = 0, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    uint8  threshold;
    static uint8 last_th = 120;                     /* 帧间限速用 */

    for (i = 0; i < 256; i++) pixelCount[i] = 0;

    /* ★1. 采样整幅图（不要只取远处），步长 3 → 约 1/9 像素 */
    for (i = 4; i < row - 4; i += 3)
    {
        for (j = 0; j < col; j += 3)
        {
            pixelCount[(int)image[i * col + j]]++;
            gray_sum   += (uint32)image[i * col + j];
            sample_num++;
        }
    }
    if (sample_num == 0) return last_th;

    for (i = 0; i < 256; i++) pixelPro[i] = (float)pixelCount[i] / (float)sample_num;

    /* ★2. 全图均值只算一次，别再 gray_sum/pixelSum 混搭 */
    u = (float)gray_sum / (float)sample_num;

    /* ★3. 扫完 256 个 bin，取全局最大；去掉提前 break */
    for (j = 0; j < 256; j++)
    {
        w0    += pixelPro[j];
        u0tmp += (float)j * pixelPro[j];
        w1     = 1.0f - w0;
        u1tmp  = u - u0tmp;

        if (w0 < 0.0001f || w1 < 0.0001f) continue;          /* ★除零保护 */

        u0 = u0tmp / w0;
        u1 = u1tmp / w1;

        deltaTmp = w0 * (u0 - u) * (u0 - u)                  /* ★不用 pow（double 太慢） */
                 + w1 * (u1 - u) * (u1 - u);

        if (deltaTmp > deltaMax)
        {
            deltaMax  = deltaTmp;
            threshold = (uint8)j;
        }
    }

    /* ★4. 限幅：先按你正常直道时的阈值（大概 110~140）来定这个区间 */
    if (threshold < 60)  threshold = 60;
    if (threshold > 200) threshold = 200;

    /* ★5. 帧间限速：单帧最多变化 ±25，彻底堵住"突然全白" */
    if (threshold > last_th + 25) threshold = last_th + 25;
    if (threshold < last_th - 25) threshold = last_th - 25;
    last_th = threshold;

    return threshold;
}