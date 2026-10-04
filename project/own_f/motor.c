#include "motor.h"

volatile int16 speed_l = 0;
volatile int16 speed_r = 0;

volatile float speed_kp = 30.0f;
volatile float speed_ki =  10.0f;

static uint16 Left_PWM,Right_PWM;
static PID_T Left_SpeedPidStructure,Right_SpeedPidStructure;

static volatile int16  target_l = 0, target_r = 0;

/*编码器初始化*/
void encoder_init(void)
{
	encoder_dir_init(PWMC_ENCODER, PWMC_ENCODER_CH1P_P40, PWMC_ENCODER_CH2P_P42);
    encoder_dir_init(PWMA_ENCODER, PWMA_ENCODER_CH1P_P60, PWMA_ENCODER_CH2P_P62);	
}

/*获取编码器(速度)值*/
void encoder_update(void)
{
	speed_l = encoder_get_count(PWMC_ENCODER);
	speed_r = -encoder_get_count(PWMA_ENCODER);

	encoder_clear_count(PWMC_ENCODER);
	encoder_clear_count(PWMA_ENCODER);
}

/*结构体成员初始化*/
void motor_param_init(void)
{
    Left_SpeedPidStructure.outmax = 8000;
    Left_SpeedPidStructure.outmin = -8000;

    Right_SpeedPidStructure.outmax = 8000;
    Right_SpeedPidStructure.outmin = -8000;
}

/*电机初始化*/
void motor_init(void) 
{
    encoder_init();
    motor_param_init();

    gpio_init(DIR_1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(DIR_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_1, 17000, 0);//17khz
    pwm_init(PWM_2, 17000, 0);
}

/*电机速度控制*/
void motor_set_target(int16 speedl,int16 speedr)
{
    target_l = speedl;
    target_r = speedr;
}

/*电机停转*/
void motor_stop(void)
{
    motor_set_target(0,0);
}


/*定时器定时控制pid函数 10ms*/
void motor_speed_control(void)
{
    encoder_update();
    Left_SpeedPidStructure.target  = target_l;
    Right_SpeedPidStructure.target = target_r;
    Left_SpeedPidStructure.actual = speed_l;
    Right_SpeedPidStructure.actual = speed_r;
    Left_SpeedPidStructure.kp = speed_kp;
    Right_SpeedPidStructure.kp = speed_kp;
    Left_SpeedPidStructure.ki = speed_ki;
    Right_SpeedPidStructure.ki = speed_ki;
    


    PID_Update(&Left_SpeedPidStructure);
    PID_Update(&Right_SpeedPidStructure);

    if(Left_SpeedPidStructure.out >= 0)
    {
        gpio_set_level(DIR_2, !0);//对于左电机,1是正转
        Left_PWM = (uint16)Left_SpeedPidStructure.out;
    }
    else
    {
        gpio_set_level(DIR_2, 0);//对于左电机,1是正转
        Left_PWM = (uint16)(-Left_SpeedPidStructure.out);
    }
    if(Right_SpeedPidStructure.out >= 0)
    {
        gpio_set_level(DIR_1, 0);//对于左电机,1是正转
        Right_PWM = (uint16)Right_SpeedPidStructure.out;
    }
    else
    {
        gpio_set_level(DIR_1, !0);//对于左电机,1是正转
        Right_PWM = (uint16)(-Right_SpeedPidStructure.out);
    }

    pwm_set_duty(PWM_2,Left_PWM);
    pwm_set_duty(PWM_1,Right_PWM);
}

