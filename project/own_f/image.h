#ifndef __IMAGE_H
#define __IMAGE_H

#include "zf_common_headfile.h"
#include "board.h"

/* 调试开关：不想显示就改 0*/
#define IMG_SHOW_ENABLE   (1)

uint8  image_update(void);        /* 有新帧并处理完返回 1，否则返回 0 */
void   image_show(void);          /* 只负责显示，可降频、可关闭 */

uint8  image_get_threshold(void); /* 给 track.c 做二值化用 */
void   image_draw_point(uint16 row, uint16 col, uint8 gray);  /* 给别的模块往图上画线 */

#endif