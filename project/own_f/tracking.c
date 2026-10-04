#include "tracking.h"
#include "image.h"
#include "servo.h"

/* 边线/中线缓存：0 = 最远，119 = 车头 */
uint8 left_line[IMG_H];
uint8 right_line[IMG_H];
uint8 mid_line[IMG_H];
uint8 left_find_flag[IMG_H];
uint8 right_find_flag[IMG_H];

uint8 cross_flag = 0;


/*-------------------------------------------------------------------
 * 逐行提取左右边线并合成中线
 * 赛道模型：白底 + 两侧黑边线；从上一行中线位置出发，向左右找“白→黑”跳变（黑线内沿）
 * ★丢线时保留哨兵值：左丢 left_line=0，右丢 right_line=IMG_W-1
 *    —— cross_fill_new 的 LOST_THRESHOLD 判据就是靠这两个值识别丢线的，千万别改！
 *------------------------------------------------------------------*/
void scan_lines(uint8 threshold, uint8 *left_line, uint8 *right_line, uint8 *mid_line,
                uint8 start_row, uint8 end_row)
{
    int16 row, col;
    int16 left, right, m;
    uint8 center = IMG_CENTER;

    for (row = start_row; row >= (int16)end_row; row--)
    {
        left  = -1;
        right = -1;

        /* 左边界：从 center 向左找 白→黑 跳变（黑线右沿 = 白路面左边界） */
        for (col = center; col >= 3; col--)
        {
            if (mt9v03x_image[row][col] > threshold && mt9v03x_image[row][col - 1] <= threshold)
            { left = col; break; }
        }
        /* 右边界：从 center 向右找 白→黑 跳变 */
        for (col = center; col < IMG_W - 3; col++)
        {
            if (mt9v03x_image[row][col] > threshold && mt9v03x_image[row][col + 1] <= threshold)
            { right = col; break; }
        }

        /* 存边线：丢线保留哨兵值 + 置标志位 */
        if (left  < 0) { left_find_flag[row]  = 0; left_line[row]  = 0;           }
        else           { left_find_flag[row]  = 1; left_line[row]  = (uint8)left; }

        if (right < 0) { right_find_flag[row] = 0; right_line[row] = IMG_W - 1;   }
        else           { right_find_flag[row] = 1; right_line[row] = (uint8)right;}

        /* 合成中线：单边丢线用半赛道宽推算，双边丢线继承更近的一行
           ★只写 mid_line，绝对不要回写 left_line / right_line！ */
        if      (left_find_flag[row] && right_find_flag[row])
            m = ((int16)left_line[row] + (int16)right_line[row]) / 2;
        else if (right_find_flag[row])
            m = (int16)right_line[row] - HALF_WIDTH;      /* 左丢 → 靠右线推 */
        else if (left_find_flag[row])
            m = (int16)left_line[row] + HALF_WIDTH;       /* 右丢 → 靠左线推 */
        else
            m = (row < START_ROW) ? mid_line[row + 1] : IMG_CENTER;   /* 都丢 → 继承更近那行 */

        if (m < 0)         m = 0;
        if (m > IMG_W - 1) m = IMG_W - 1;
        mid_line[row] = (uint8)m;

        /* 下一行的搜索种子 = 本行中线（限幅防跑飞） */
        center = (uint8)m;
        if (center < SEARCH_RANGE)         center = SEARCH_RANGE;
        if (center > IMG_W - SEARCH_RANGE) center = IMG_W - SEARCH_RANGE;
    }
}


//======================================== 十字路口检测与补线函数 ========================================
/**
 * @brief  十字路口检测与补线（上下双向扫描四跳变点法）
 * @note   从上到下、从下到上分别扫描左右边线的跳变点（有效→丢线），
 *         检测到 3 个或 4 个跳变点即判定为十字路口，
 *         用上下有效边界做线性插值填充十字区域的边线，中线置为图像中心直行通过。
 * @param  left_line   左边界数组（需预分配 IMG_H 大小）
 * @param  right_line  右边界数组（需预分配 IMG_H 大小）
 * @param  mid_line    中线数组（需预分配 IMG_H 大小）
 * @return uint8       检测到十字路口返回 1，否则返回 0
 */
uint8 cross_fill_new(uint8 *left_line, uint8 *right_line, uint8 *mid_line)
{
    int row;
    int top_l = -1, top_r = -1;    // 上跳变点行：自上而下扫描，有效→丢线的跳变位置
    int btm_l = -1, btm_r = -1;    // 下跳变点行：自下而上扫描，有效→丢线的跳变位置
    int break_cnt;                  // 跳变点计数
    int top_row, btm_row;          // 统一的上/下跳变点行
    int range, y_diff;
    static uint8 crossed_flag = 0; // 连续防抖计数

    #define JUMP_THRESHOLD      20   // 跳变阈值：相邻行差值超过此值即判定跳变
    #define LOST_THRESHOLD      30   // 丢线阈值：左<此值 或 右>IMG_W-此值

    // ---- 1. 自上而下扫描：找上跳变点（正常赛道→十字区域） ----
    for (row = END_ROW; row < IMG_H - 10; row++)
    {
        // 左跳变：正常值跳变为极小值（下行丢线）
        if (top_l < 0 && (int)left_line[row] - (int)left_line[row + 1] > JUMP_THRESHOLD
            && left_line[row + 1] < LOST_THRESHOLD)
            top_l = row;
        // 右跳变：正常值跳变为极大值（下行丢线）
        if (top_r < 0 && (int)right_line[row + 1] - (int)right_line[row] > JUMP_THRESHOLD
            && right_line[row + 1] > IMG_W - 1 - LOST_THRESHOLD)
            top_r = row;
        if (top_l >= 0 && top_r >= 0)
            break;
    }

    // ---- 2. 自下而上扫描：找下跳变点（正常赛道→十字区域） ----
    for (row = IMG_H - 1; row > END_ROW + 10; row--)
    {
        // 左跳变：正常值跳变为极小值（上行丢线）
        if (btm_l < 0 && (int)left_line[row - 1] - (int)left_line[row] > JUMP_THRESHOLD
            && left_line[row] < LOST_THRESHOLD)
            btm_l = row - 1;
        // 右跳变：正常值跳变为极大值（上行丢线）
        if (btm_r < 0 && (int)right_line[row] - (int)right_line[row - 1] > JUMP_THRESHOLD
            && right_line[row] > IMG_W - 1 - LOST_THRESHOLD)
            btm_r = row - 1;
        if (btm_l >= 0 && btm_r >= 0)
            break;
    }

    // ---- 3. 统计跳变点数量 ----
    break_cnt = 0;
    if (top_l >= 0) break_cnt++;
    if (top_r >= 0) break_cnt++;
    if (btm_l >= 0) break_cnt++;
    if (btm_r >= 0) break_cnt++;

    // ---- 4. 跳变点 < 3：不是十字路口，释放补线 ----
    if (break_cnt < 3)
    {
        crossed_flag = 0;
        return 0;
    }

    // ---- 5. 防抖：连续两帧检测到才触发 ----
    if (crossed_flag == 0)
    {
        crossed_flag = 1;
        return 0;
    }

    // ---- 6. 取统一的上/下跳变点行（处理只有 3 个跳变点的情况） ----
    // 上跳变点行：取两个上跳变点中靠下方的（更深入十字区域）
    top_row = -1;
    if (top_l >= 0 && top_r >= 0)
        top_row = (top_l > top_r) ? top_l : top_r;
    else if (top_l >= 0)
        top_row = top_l;
    else if (top_r >= 0)
        top_row = top_r;

    // 下跳变点行：取两个下跳变点中靠上方的
    btm_row = -1;
    if (btm_l >= 0 && btm_r >= 0)
        btm_row = (btm_l < btm_r) ? btm_l : btm_r;
    else if (btm_l >= 0)
        btm_row = btm_l;
    else if (btm_r >= 0)
        btm_row = btm_r;

    if (top_row < 0 || btm_row < 0 || top_row >= btm_row) return 0;

    // ---- 7. 线性插值填充十字区域的边线，利用四个拐点计算中线 ----
    range = btm_row - top_row;
    for (row = top_row; row <= btm_row; row++)
    {
        y_diff = row - top_row;
        // 左边线：上跳变点 → 下跳变点 线性插值
        left_line[row] = (uint8)((int)left_line[top_row]
                          + ((int)left_line[btm_row] - (int)left_line[top_row]) * y_diff / range);
        // 右边线：上跳变点 → 下跳变点 线性插值
        right_line[row] = (uint8)((int)right_line[top_row]
                           + ((int)right_line[btm_row] - (int)right_line[top_row]) * y_diff / range);
        // 中线：基于四个拐点插值后的左右边线取中点
        mid_line[row] = (uint8)(((int)left_line[row] + (int)right_line[row]) / 2);
    }
    return 1;
}


float compute_servo_angle(uint8 *mid_line, uint8 row, uint8 center_col,
                          float kp, float kd, float mid_angle, float min_angle, float max_angle)
{
    #define DEAD_ZONE   2               /* 死区，防直道抖舵 */
    static int16 last_err = 0;
    int16 err;
    float d_err, offset, target;

    err = (int16)mid_line[row] - (int16)center_col;
    if (err > -DEAD_ZONE && err < DEAD_ZONE) err = 0;

    d_err    = (float)(err - last_err);
    last_err = err;

    offset = kp * err + kd * d_err;
    target = mid_angle - offset;        /* 中线偏右(err>0) → 角度变小 → 右转 */

    if (target > max_angle) target = max_angle;
    if (target < min_angle) target = min_angle;
    return target;
}


/* 巡线总流程：主循环在 image_update() 之后、image_frame_done() 之前调用 */
void tracking_task(void)
{
    scan_lines(image_get_threshold(), left_line, right_line, mid_line, START_ROW, END_ROW);
    cross_flag = cross_fill_new(left_line, right_line, mid_line);

    servo_set_angle( compute_servo_angle(mid_line, CONTROL_ROW, IMG_CENTER,
                                         servo_kp, servo_kd,
                                         SERVO_MID, SERVO_RIGHT_MAX, SERVO_LEFT_MEX) );

    /* ★以后元素判断（环岛/斑马线/出界）就插在这一行前面，改完 mid_line 再算角度 */
}


/* 调试：把左右边线和中线画到快照上，配合 image_show() 用 */
void tracking_draw(void)
{
    uint8 row;
    for (row = END_ROW; row <= START_ROW; row++)
    {
        image_draw_point(row, left_line[row],  0);      /* 左边线 → 黑 */
        image_draw_point(row, right_line[row], 255);    /* 右边线 → 白 */
        image_draw_point(row, mid_line[row],   128);    /* 中线   → 灰 */
    }
    image_draw_point(CONTROL_ROW, mid_line[CONTROL_ROW], 255);   /* 控制行打点 */
}