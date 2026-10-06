#ifndef __BOARD_H
#define __BOARD_H


/***
 * 简介:板级配置,用于配置引脚,参数等
 */


/*舵机配置*/
#define SERVO_PWM_PIN       PWME_CH1P_PA0
#define SERVO_FREQ          150
#define SERVO_DUTY(x)   	((float)PWM_DUTY_MAX / (1000.0f/(float)SERVO_FREQ) * (0.5f + (float)(x)/90.0f))

/*摄像头配置*/
#define IPS200_DISPLAY_W    ( 320 )                                     // IPS200 横屏分辨率 宽 对应 ips200_set_dir(IPS200_CROSSWISE_180)
#define IPS200_DISPLAY_H    ( 240 )                                     // IPS200 横屏分辨率 高
#define CAMERA_DIS_W        ( MT9V03X_W * 1.2f)                               // 图像显示宽度 默认 1:1 显示 放大可以改成 ( MT9V03X_W * 3 / 2 )
#define CAMERA_DIS_H        ( MT9V03X_H * 1.2f)                               // 图像显示高度 默认 1:1 显示 放大可以改成 ( MT9V03X_H * 3 / 2 )

/*舵机配置*/
#define SERVO_MID          85.5f  //正中
#define SERVO_RIGHT_MAX    76.0f //最右
#define SERVO_LEFT_MEX     96.5f  //最左

/*电机配置*/
#define DIR_1               ( IO_P75 )//右电机
#define PWM_1               ( PWMB_CH1_P74 )
                              
#define DIR_2               ( IO_P77 )//左电机
#define PWM_2               ( PWMB_CH3_P76 )



#endif
