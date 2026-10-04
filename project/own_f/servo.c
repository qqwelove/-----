#include "servo.h"



/* ★定义成变量（不是宏），VOFA 才能取地址在线调 */
float servo_kp = 0.31f;
float servo_kd = 0.05f;



/*舵机初始化*/
void servo_init(void)
{
  pwm_init(SERVO_PWM_PIN, SERVO_FREQ, SERVO_DUTY(SERVO_MID));
}


void servo_set_angle(float angle)
{
    if (angle < SERVO_RIGHT_MAX) angle = SERVO_RIGHT_MAX;   /* 76  最右 */
    if (angle > SERVO_LEFT_MEX)  angle = SERVO_LEFT_MEX;    /* 99.5 最左 */
    pwm_set_duty(SERVO_PWM_PIN, (uint32)SERVO_DUTY(angle));
}