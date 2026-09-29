#include "servo.h"

/*¶æ»ú³õÊ¼»¯*/
void servo_init(void)
{
  pwm_init(SERVO_PWM_PIN, SERVO_FREQ, SERVO_DUTY(SERVO_MID));
}
