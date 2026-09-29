#include "zf_common_headfile.h"
#include "pid.h"
#include "vofa.h"                                               // VOFA+ 在线调参 / 波形显示
#include "board.h"
#include "servo.h"
#include "motor.h"
#include "image.h"



/*测试变量区*/
float speed;/*-300~300*/


/*
函数声明
*/
void vofa_send_tick (void);


//-------------------------------------------------------------------------------------------------------------------
// VOFA+ 在线调参 示例配置
//   不需要在线调参时 把 VOFA_DEMO_ENABLE 改成 0 即可（下面所有相关调用会自动消失）
//-------------------------------------------------------------------------------------------------------------------
#define VOFA_DEMO_ENABLE        (0)             // 1 = 打开 VOFA+ 在线调参示例  0 = 关闭

#if VOFA_DEMO_ENABLE
float vofa_kp = 30.0f;                          // 通道 0 待调参数
float vofa_ki =  10.0f;                          // 通道 1 待调参数
float vofa_kd =  0.0f;                          // 通道 2 待调参数

float vofa_wave_target = 0.0f;                  // 通道 3 波形通道（换成你要观察的量 例如目标速度）
float vofa_wave_now    = 0.0f;                  // 通道 4 波形通道（换成你要观察的量 例如实际速度）

// 参数指针表 数组下标就是 VOFA+ 里设置的通道号
float *vofa_param_table[4] =
 {
    (float *)&speed_kp,
    (float *)&speed_ki,
    (float *)&speed_kp,
    (float *)&speed,
};

vuint8 vofa_send_request = 0;                   // 定时器里置位 主循环里发送


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     VOFA+ 波形发送定时器回调（20ms 一次 也就是 50Hz 波形刷新）
// 备注信息     中断里只置标志位 真正的发送放到主循环里做 避免长时间阻塞中断
//-------------------------------------------------------------------------------------------------------------------
void vofa_send_tick (void)
{
    vofa_send_request = 1;
}
#endif



/*周期中断服务函数测试*/
void PIT_Function_Timer0(void)
{
    static uint16 Pid_Count;

#if VOFA_DEMO_ENABLE
    static uint8  VofaCount;                            // VOFA+ 波形发送节拍计数
		VofaCount ++;

#endif

    Pid_Count ++; 
		

#if VOFA_DEMO_ENABLE
    // VOFA+ 波形发送节拍  1ms * 20 = 20ms 发一帧(50Hz)
    // 不要再用 pit_ms_init(TIM11_PIT, 20, ...) 来产生这个节拍：库函数 system_delay() 内部占用的
    // 就是定时器 11，而 ips200_init() 里的 system_delay_ms() 会把定时器 11 的配置和中断允许位
    // 一起清掉(T11CR = 0)，之后 TIM11 中断永远不再产生，波形一帧都发不出去
    if(VofaCount >= 1)
    {
        VofaCount = 0;
        vofa_send_tick();
    }
#endif

    if(Pid_Count >= 5)
    {
        Pid_Count = 0;
        motor_speed_control();
    }
        
}

/*全部初始化*/
void init_all(void)
{
    clock_init(SYSTEM_CLOCK_96M);

#if VOFA_DEMO_ENABLE
    vofa_usb_init(vofa_param_table, 4);                                 // 初始化 VOFA+（走 USB 虚拟串口 插着下载线就行）

#endif

    /*屏幕初始化部分,注意初始化时序*/
    {
        ips200_set_dir(IPS200_CROSSWISE_180);
    }
	ips200_init();
    {
        ips200_clear(RGB565_BLUE);
        ips200_set_color(RGB565_WHITE, RGB565_BLUE);

    }

    servo_init();

    /*已包含encoder_init()*/
    motor_init();

    while (mt9v03x_init()) 
    {
        system_delay_ms(800);
    }

    pit_ms_init(TIM0_PIT,1,PIT_Function_Timer0);
}







void main(void)
{
    init_all();

    motor_set_target(0,0);
    
    while(1)
    {
        if(image_update())
        {
            image_show();
        }

        motor_set_target(speed,speed);

/*vofa调参部分*/
#if VOFA_DEMO_ENABLE
        vofa_apply_param();                                             // 把上位机发来的新参数写进变量（开销极小 每圈都调用）

        if(vofa_send_request)                                           // 20ms 时间到 发一帧波形给 VOFA+
        {
            float wave[5];

            vofa_send_request = 0;

            wave[0] = speed_r;                                          // 通道 0
            wave[1] = speed_l;                                          // 通道 1
            wave[2] = speed;                                          // 通道 2
            wave[3] = vofa_wave_target;                                 // 通道 3
            wave[4] = vofa_wave_now;                                    // 通道 4
            vofa_send_wave(wave, 5);                                    // JustFloat 发送
        }
#endif    
    }
}