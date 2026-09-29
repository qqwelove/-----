#include "image.h"

/*图像缓冲数组*/
static uint8 xdata display_buffer[MT9V03X_H][MT9V03X_W];

static uint8 image_threshold = 0;
static uint16 frame_cnt = 0, image_fps = 0;

static uint8 adapt_threshold(uint8 xdata *image, uint16 col, uint16 row);


uint8 image_update(void)
{
    if(!mt9v03x_finish_flag) return 0;          /* 没有新帧就直接返回，main 不用管标志位 */

    image_threshold = adapt_threshold((uint8 xdata *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
    memcpy(display_buffer, mt9v03x_image, sizeof(display_buffer));
    mt9v03x_finish_flag = 0;                    /* 处理完才放行下一帧 */

    return 1;
}


void image_show(void)
{
#if IMG_SHOW_ENABLE
    ips200_show_gray_image(0, 0, display_buffer[0],
                           MT9V03X_W, MT9V03X_H,
                           CAMERA_DIS_W, CAMERA_DIS_H,
                           0);
#endif
}


uint8 image_get_threshold(void) { return image_threshold; }


void image_draw_point(uint16 row, uint16 col, uint8 gray)
{
    if(row < MT9V03X_H && col < MT9V03X_W) display_buffer[row][col] = gray;
}


static uint8 adapt_threshold(uint8 xdata *image, uint16 col, uint16 row)   
{
	#define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    uint16 i, j;
	int pixelSum = 0;
    uint8 threshold = 0;
	uint32 gray_sum=0;
	
	float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
	
	pixelSum = (width) * (height-60)/9;
	
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    
    for (i = 10; i < height-50 ; i+=3)
    {
        for (j = 0; j < width; j+=3)
        {
		pixelCount[(int)image[i * width + j]]++;  
		gray_sum+=(int)image[i * width + j];       
		}
    }

    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
		w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
		for (j = 0; j < GrayScale; j++)
		{
			w0 += pixelPro[j];  
			u0tmp += j * pixelPro[j]; 
			w1=1-w0;
			u1tmp=gray_sum/pixelSum-u0tmp;
			u0 = u0tmp / w0;              
			u1 = u1tmp / w1;              
			u = u0tmp + u1tmp;           
			deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
			if (deltaTmp > deltaMax)
			{
					deltaMax = deltaTmp;
					threshold = (uint8)j;
			}
			if (deltaTmp < deltaMax)
			{
			break;
			}
		}
    return threshold;
}
