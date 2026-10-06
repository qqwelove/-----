#include "image.h"
#include "tracking.h"

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
                           0);
                            ips200_show_uint8(0,200, track_out_flag);
                            ips200_show_uint8(60,200, track_out_cnt);  
                            ips200_show_uint8(100,200, track_out_mean_min);
                            ips200_show_uint8(140,200, dbg_cross_stage);
                            ips200_show_uint8(180,200, image_get_threshold());   /* ★本帧阈值，调 OTSU_OK_MIN/MAX 就看它 */
#endif
}


uint8 image_get_threshold(void) { return image_threshold; }


void image_draw_point(uint16 row, uint16 col, uint8 gray)
{
    if(row < MT9V03X_H && col < MT9V03X_W) display_buffer[row][col] = gray;
}


/*-------------------------------------------------------------------
 * 大津法（Otsu）阈值 + 两道记忆保护
 *   ★为什么要“记忆保护”：大津法会自适应到“这一帧没有黑线”上面去，
 *     算出 30 这种极低阈值 → 二值化整屏全白 → 巡线瞬间失去参考 → 冲出去。
 *     与其让它乱跳，不如“跳得太远就不采纳，沿用上次的值”。
 *------------------------------------------------------------------*/
#define OTSU_OK_MIN     70      /* 低于它就认为这帧阈值算废了（正常大概 100~150） */
#define OTSU_OK_MAX    190      /* 高于它同上 */
#define OTSU_MAX_STEP   20      /* 单帧最多允许变化多少（防抖动） */

static uint8 adapt_threshold(uint8 xdata *image, uint16 col, uint16 row)
{
    int   pixelCount[256];
    float pixelPro[256];
    uint16 i, j;
    uint32 gray_sum = 0, sample_num = 0;
    uint8  th_min = 255, th_max = 0;                /* 本帧实际出现的灰度范围 */
    float  w0 = 0, w1, u0tmp = 0, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    static uint8 last_th = 120;                     /* 上一次被采纳的阈值 */
    uint8  threshold = last_th;

    for (i = 0; i < 256; i++) pixelCount[i] = 0;

    /* ★1. 采样整幅图（步长 3），顺便统计实际灰度范围 + 实际采样点数 */
    for (i = 0; i < row; i += 3)
    {
        for (j = 0; j < col; j += 3)
        {
            uint8 g = image[i * col + j];
            pixelCount[g]++;
            gray_sum += (uint32)g;
            sample_num++;
            if (g < th_min) th_min = g;
            if (g > th_max) th_max = g;
        }
    }

    /* ★2. 全黑 / 全白 / 异常帧 → 直接沿用上次的阈值 */
    if (sample_num == 0 || th_max <= th_min) return last_th;

    /* ★3. 比例的分母用“实际采样点数”，不会有 1253≠1260 那种偏差 */
    for (i = 0; i < 256; i++) pixelPro[i] = (float)pixelCount[i] / (float)sample_num;

    /* ★4. 全图平均灰度只算一次（浮点算，避免整数除法丢精度） */
    u = (float)gray_sum / (float)sample_num;

    /* ★5. 只在 [th_min, th_max] 里找，省时间；扫完取全局最大，不做提前退出 */
    for (j = th_min; j <= th_max; j++)
    {
        w0    += pixelPro[j];
        u0tmp += (float)j * pixelPro[j];
        w1     = 1.0f - w0;
        u1tmp  = u - u0tmp;

        if (w0 < 0.0001f || w1 < 0.0001f) continue;             /* 除零保护 */

        u0 = u0tmp / w0;
        u1 = u1tmp / w1;

        deltaTmp = w0 * w1 * (u0 - u1) * (u0 - u1);             /* 等价形式，数值更稳，也不用 pow */
        if (deltaTmp > deltaMax)
        {
            deltaMax  = deltaTmp;
            threshold = (uint8)j;
        }
    }

    /* ★6. 记忆保护①：跳得太远就不采纳 ← 治“突然全白”的关键一条 */
    if (threshold < OTSU_OK_MIN || threshold > OTSU_OK_MAX)
    {
        threshold = last_th;
    }
    else
    {
        /* ★7. 记忆保护②：单帧最多变化 ±OTSU_MAX_STEP，防抖动 */
        if (threshold > last_th + OTSU_MAX_STEP) threshold = last_th + OTSU_MAX_STEP;
        if (threshold < last_th - OTSU_MAX_STEP) threshold = last_th - OTSU_MAX_STEP;
    }
    last_th = threshold;

    return threshold;
}