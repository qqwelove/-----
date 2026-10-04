#ifndef __MOTOR_H
#define __MOTOR_H

#include <zf_common_headfile.h>
#include "pid.h"
#include "board.h"

/*编码器测速变量*/
extern volatile int16 speed_l;//10ms 测速周期
extern volatile int16 speed_r;

extern volatile float speed_kp,speed_ki; 


void motor_init(void);
// void motor_param_imit(void);
void motor_set_target(int16 speedl,int16 speedr);
void motor_speed_control(void);     /* 由 1ms 中断分频后调用 */
void motor_stop(void);
void motor_enable_set(uint8 en);

#endif