#ifndef __SERVO_H
#define __SERVO_H

#include "zf_common_headfile.h"
#include "board.h"


extern float servo_kp, servo_kd;
void servo_init(void);
void servo_set_angle(float angle);


void servo_init(void);

#endif
