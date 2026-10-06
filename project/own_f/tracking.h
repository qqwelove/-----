#ifndef __TRACKING_H
#define __TRACKING_H

#include "zf_common_headfile.h"
#include "board.h"

/*-------------------------------------------------------------------
 * ★索引约定（务必记住）：本模块一律使用“绝对图像行号”
 *     row = 0    → 图像最上一行（最远）
 *     row = 119  → 图像最下一行（车头）
 *   cross_fill_new() 就是按这个约定写的，扫线也必须按这个存
 *------------------------------------------------------------------*/
#define IMG_H               MT9V03X_H                        /* 120 */
#define IMG_W               MT9V03X_W                        /* 188 */
#define IMG_CENTER          (IMG_W / 2)                      /* 94  */

#define START_ROW           (IMG_H - 1)                      /* 119 从车头开始扫 */
#define END_ROW             5                                /* 最远处理到第 5 行 */
#define CONTROL_ROW_OFFSET  25                               /* 控制行距车头的行数 */
#define CONTROL_ROW         (START_ROW - CONTROL_ROW_OFFSET) /* ★94 前瞻控制行（不是25！） */

#define SEARCH_RANGE        10                               /* 种子中心限幅，防爬线跑飞 */

/* 边线 / 中线 / 丢线标志（定义在 tracking.c） */
extern uint8 left_line[IMG_H];
extern uint8 right_line[IMG_H];
extern uint8 mid_line[IMG_H];
extern uint8 left_find_flag[IMG_H];
extern uint8 right_find_flag[IMG_H];

extern uint8 cross_flag;        /* 1 = 本帧正在十字补线（以后给元素判断/出界保护用） */


void  tracking_task(void);      /* ★统一入口：出界保护 → 扫线 → 十字 → 舵机（主循环只调这一个） */
void  tracking_draw(void);      /* 调试：把三条线画到快照上 */

/* 出界保护（定义在 tracking.c） */
extern uint8 track_out_flag;      /* 1 = 已判出界（锁存） */
extern uint8 track_out_cnt;       /* 连续命中帧数 */
extern uint8 track_out_mean;      /* 本帧“车头前面”的平均灰度（调试用） */
extern uint8 track_out_mean_min;  /* ★调试用：开机以来最小的平均灰度，标定 TRACK_OUT_MEAN 就看它 */
uint8 track_out_check(void);      /* 返回 1 = 出界（tracking_task 里会自动调它并把电机置零） */
void  track_out_reset(void);      /* 手动清除出界锁存 + 复位最小值（调试用） */

/*变量校准区*/
extern int16 dbg_cross_a100;   /* 进入方向斜率 ×100 */
extern uint8 dbg_cross_m0;   /* 入口处中线 */
extern uint8 dbg_cross_m94;   /* 控制行(94)的中线 */

/* 十字状态机（调试用，可打到屏上看） */
extern uint8  cross_state;         /* 0 = 正常巡线  1 = 正在过十字（中线压直） */
extern uint8  cross_inside_flag;   /* 1 = 已确认进到十字内部 */
extern uint16 cross_time;          /* 进十字后的帧数 */
extern uint8  dbg_cross_nearlost;  /* 近处(100~119)两侧都丢线的行数 */


void  scan_lines(uint8 threshold, uint8 *left_line, uint8 *right_line, uint8 *mid_line,
                 uint8 start_row, uint8 end_row);
uint8 cross_fill_new(uint8 *left_line, uint8 *right_line, uint8 *mid_line);
float compute_servo_angle(uint8 *mid_line, uint8 row, uint8 center_col,
                          float kp, float kd, float mid_angle, float min_angle, float max_angle);


#endif