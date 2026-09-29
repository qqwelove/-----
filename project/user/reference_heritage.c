#include "zf_common_headfile.h"

//255 白 0 黑
//======================================== 硬件配置 ========================================
// 舵机配置
#define SERVO_PWM_PIN       PWME_CH1P_PA0
#define SERVO_FREQ          50
#define SERVO_ANGLE_MIN     76.0f  //右转极限
#define SERVO_ANGLE_MAX     109.0f //左转极限
#define SERVO_ANGLE_MID     90.0f
#define SERVO_DUTY(x)   ((float)PWM_DUTY_MAX / (1000.0f/(float)SERVO_FREQ) * (0.5f + (float)(x)/90.0f))

#define DIR_1               ( IO_P75 )
#define PWM_1               ( PWMB_CH1_P74 )
#define DIR_2               ( IO_P77 )
#define PWM_2               ( PWMB_CH3_P76 )

//#define DIR_3               ( IO_P75 )
//#define PWM_3               ( PWMB_CH1_P74 )
//                              
//#define DIR_4               ( IO_P77 )
//#define PWM_4               ( PWMB_CH3_P76 )
                              

//======================================== 图像处理参数 ========================================
#define COEFF               1.12f

#define SEG_END_ROW         110
#define SEG_START_ROW_FAR   80
#define SEG_START_ROW_NEAR  95
#define SEG_LEFT_END        (MT9V03X_W / 2)
#define SEG_RIGHT_START     (MT9V03X_W / 2)

#define TURN_RATIO_MIN      0.10f
#define TURN_ANGLE_MAX_RIGHT  (SERVO_ANGLE_MID - SERVO_ANGLE_MIN)
#define TURN_ANGLE_MAX_LEFT   (SERVO_ANGLE_MAX - SERVO_ANGLE_MID)
#define MIN_WHITE_RATIO     0.05f
#define TURN_STRENGTH_MIN   0.2f

// 锁死持续时间
#define LOCK_DURATION_FRAMES    15
// 锁死后防重触发冷却时间（禁止再次检测环岛的帧数）
#define LOCK_COOLDOWN_FRAMES    50
// 十字前瞻检测行（固定行）
#define CROSS_DETECT_ROW       80
// 十字标志持续时间（帧数）
#define CROSS_FLAG_DURATION    20

//======================================== 全局变量 ========================================
uint8 xdata display_buffer[MT9V03X_H][MT9V03X_W];
uint8 dynamic_start_row;

// 锁死控制变量
static uint8 lock_active = 0;          // 是否处于强制转向锁死状态
static uint8 lock_direction = 0;       // 锁死方向：1左转，2右转
static uint16 lock_duration = 0;       // 锁死剩余帧数（强制转向）
static uint16 lock_cooldown = 0;       // 冷却剩余帧数（禁止再次检测）

// 十字前瞻标志
static uint8 cross_flag = 0;           // 前方是否可能为十字路口D:\电子设计\软件\keil_project\C251\contest\无敌-yyyyy-4\project
static uint16 cross_counter = 0;       // 标志剩余帧数

// 辅助函数：计算某一行的白色像素占比（整行）
float get_line_white_ratio(uint8 row, uint8 threshold)
{
    uint16 white_cnt = 0;
    uint16 total = MT9V03X_W;
    uint8 col;
    for (col = 0; col < MT9V03X_W; col++) {
        if (mt9v03x_image[row][col] > threshold) white_cnt++;
    }
    return (float)white_cnt / total;
}

// 修改后的 pic_dp：支持左右起始行分离
void pic_dp(uint8 threshold, uint8 *high_conf, float *left_ratio, float *right_ratio, float *bias_raw,
            uint8 *zuo, uint8 *you, uint8 left_start, uint8 right_start) 
{
    uint32 left_cnt = 0, right_cnt = 0;
    uint32 left_total = 0, right_total = 0;
    uint8 row, col;
   
    // 左侧区域统计（从 left_start 开始）
    for (row = left_start; row <= SEG_END_ROW; row++) 
    {
        for (col = 0; col < SEG_LEFT_END; col++) 
        {
            left_total++;
            if (mt9v03x_image[row][col] > threshold) left_cnt++;
        }
    }
    // 右侧区域统计（从 right_start 开始）
    for (row = right_start; row <= SEG_END_ROW; row++) 
    {
        for (col = SEG_RIGHT_START; col < MT9V03X_W; col++) 
        {
            right_total++;
            if (mt9v03x_image[row][col] > threshold) right_cnt++;
        }
    }

    *zuo = (left_cnt >= left_total) ? 1 : 0; //左全白 1
    *you = (right_cnt >= right_total) ? 1 : 0;//右全白 1

    *left_ratio  = (left_total > 0) ? (float)left_cnt / left_total : 0;//左白比例
    *right_ratio = (right_total > 0) ? (float)right_cnt / right_total : 0;//右白比例
    *bias_raw = *right_ratio - *left_ratio;//左右白比例差

    *high_conf = (*bias_raw > 0.0f || *bias_raw < -0.0f) ? 1 : 0;//置信度返回 没用
}
uint8 xian_long(uint8 lie_begin,uint8 threshold)
{
	  uint8 q1=MT9V03X_W/2;
	  uint8 record=0,black_1=0;
	  uint8 lie=1;
		record=0;
    for(lie=1;lie<q1;lie++)
	   {
		    if(mt9v03x_image[lie_begin][lie-1]<threshold)//黑点
					black_1=1;
				if(mt9v03x_image[lie_begin][lie]>threshold && black_1==1)//跳变
				{
					record=lie;
					break;
				}
		 }	
		 if(record>20)
			 return record;
		 else return 0;
}

uint8 xian_short(uint8 lie_begin,uint8 threshold)
{
	  uint8 q1=MT9V03X_W/4;
	  uint8 record=0,black_1=0;
	  uint8 lie=1;
		record=0;
    for(lie=1;lie<q1;lie++)
	   {
		    if(mt9v03x_image[lie_begin][lie-1]<threshold)//黑点
					black_1=1;
				if(mt9v03x_image[lie_begin][lie]>threshold && black_1==1)//跳变
				{
					record=lie;
					black_1=0;
					break;
				}
				else
					record=0;
		 }	
			 return record;
}
void motor_init(void) 
{
    gpio_init(DIR_1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(DIR_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_1, 17000, 0);
    pwm_init(PWM_2, 17000, 0);
}

void init_all(void)
{
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    ips200_init();
    system_delay_ms(100);
//    ips200_show_string(0, 16, "IPSinit success.");
    motor_init();
    pwm_init(SERVO_PWM_PIN, SERVO_FREQ, SERVO_DUTY(90.0f));
    while (mt9v03x_init()) 
    {
        system_delay_ms(100);
//        ips200_show_string(0, 18, "mt9v03x_init error, try again.");
    }
//    ips200_show_string(0, 0, "mt9v03x_init successfully.");
//    printf("System start.\r\n");
}

//======================================== 主函数 ========================================
void main(void) 
{
    uint16 x_offset, y_offset;
    uint16 i, j;
    uint32 sum = 0;  
    uint8 avg, threshold;
    float target_angle;
    uint8 high_conf;
    float bias_raw;
    float left_ratio, right_ratio;
    float sum_ratio;
    uint8 zuo, you;
    static float last_target_angle = SERVO_ANGLE_MID;
    float angle_offset, err, kp;
    float angle_abs_norm;
    float line_ratio;
    // 用于单侧全白时的区域抬高（向远看，减小行号）
    uint8 left_start_row, right_start_row;
    char lock_str[20];
	  char debug_str[20];
    uint8 reduce_left = 0;
    uint8 reduce_right = 0;
    uint8 corss_nubmer=0;
		static uint8 special_state = 0;      // 0:正常, 1:直行, 2:左转, 3:冷却
		static uint8 special_counter = 0;    // 当前阶段剩余帧数
		static uint8 angle_set_by_state = 0; // 标志位：状态机是否已设置角度

    init_all();

    while (1) 
    {
        gpio_set_level(DIR_1, 0);
        pwm_set_duty(PWM_1, 1300);
        gpio_set_level(DIR_2, 0);
        pwm_set_duty(PWM_2, 1300);
        
        if (mt9v03x_finish_flag) 
        {
						uint8 jing = (xian_long(75, threshold) < xian_long(65, threshold) && xian_long(65, threshold) > xian_long(60, threshold));
						uint8 nujing = (xian_short(115, threshold) < xian_short(112, threshold) && xian_short(112, threshold) <xian_short(109, threshold)
					&& xian_short(109, threshold) < xian_short(106, threshold)&& xian_short(106, threshold) < xian_short(103, threshold));
					
					  sprintf(debug_str, "%d",xian_long(80,threshold));
            ips200_show_string(80, 12, debug_str);
					  sprintf(debug_str, "%d",xian_long(70,threshold));
            ips200_show_string(100, 12, debug_str);
					  sprintf(debug_str, "%d",xian_long(65,threshold));
            ips200_show_string(120, 12, debug_str);
					
						sprintf(debug_str, "%d",xian_short(115,threshold));
            ips200_show_string(80, 28, debug_str);
					  sprintf(debug_str, "%d",xian_short(112,threshold));
            ips200_show_string(100, 28, debug_str);
					  sprintf(debug_str, "%d",xian_short(109,threshold));
            ips200_show_string(120, 28, debug_str);
				  	sprintf(debug_str, "%d",xian_short(106,threshold));
            ips200_show_string(140, 28, debug_str);
						sprintf(debug_str, "%d",xian_short(103,threshold));
            ips200_show_string(160, 28, debug_str);
            // ========== 1. 动态阈值计算 ==========
            sum = 0;
            for (i = 0; i < MT9V03X_H; i++) 
                for (j = 0; j < MT9V03X_W; j++) 
                    sum += mt9v03x_image[i][j];
            avg = sum / (MT9V03X_H * MT9V03X_W);
            threshold = (uint16)(avg * COEFF);
            if (threshold < 40) threshold = 40;
            if (threshold > 220) threshold = 220;

            // ========== 2. 动态起始行（根据转向幅度） ==========
            if (last_target_angle >= SERVO_ANGLE_MID)
                angle_abs_norm = (last_target_angle - SERVO_ANGLE_MID) / TURN_ANGLE_MAX_LEFT;
            else
                angle_abs_norm = (SERVO_ANGLE_MID - last_target_angle) / TURN_ANGLE_MAX_RIGHT;
            if (angle_abs_norm > 1.0f) angle_abs_norm = 1.0f;
            
            if (angle_abs_norm > 0.8f) {
                dynamic_start_row = SEG_START_ROW_NEAR;   // 90
            } else {
                dynamic_start_row = SEG_START_ROW_FAR;    // 80
            }

            // 默认左右起始行相同
            left_start_row = dynamic_start_row;
            right_start_row = dynamic_start_row;

            // ========== 3. 计算左右占比和全白标志（先用相同起始行） ==========
            pic_dp(threshold, &high_conf, &left_ratio, &right_ratio, &bias_raw, &zuo, &you, left_start_row, right_start_row);
           
            // ========== 4. 十字前瞻检测（仅在未抬高时） ==========
            if (left_start_row == dynamic_start_row && right_start_row == dynamic_start_row) 
						{
                line_ratio = get_line_white_ratio(CROSS_DETECT_ROW, threshold);
                if (line_ratio >0.95f) 
								{
								
                    cross_flag = 1; //十字标志位  全白
                    cross_counter = CROSS_FLAG_DURATION;
									
							
                }
								
            }

            // 更新十字标志计数器
            if (cross_flag) 
						{
                if (cross_counter > 0) cross_counter--;
                else cross_flag = 0;
            } 
           						
         //    非十字   					
		// ========== 状态机：先直行，再左转，最后冷却 ==========
					angle_set_by_state = 0;

					// 仅在正常状态（special_state == 0）时检测触发条件
					if (special_state == 0 && cross_flag != 1 && !nujing && jing) {
							special_state = 1;          // 进入直行阶段
							special_counter = 4;       // 直行30帧（可调）
					}

					// 状态机独立执行，不再依赖外部条件
					if (special_state == 1) {
							target_angle = SERVO_ANGLE_MID; // 强制直行
							angle_set_by_state = 1;
							special_counter--;
							if (special_counter == 0) {
									special_state = 2;          // 进入左转阶段
									special_counter = 20;       // 左转30帧（可调）
							}
					} else if (special_state == 2) {
							target_angle = 103.0f;          // 强制左转
							angle_set_by_state = 1;
							special_counter--;
							if (special_counter == 0) {
									special_state = 3;          // 进入冷却阶段
									special_counter = 20;       // 冷却20帧（可调）
							}
					} else if (special_state == 3) {
							// 冷却阶段：不干预角度，只计时，让正常巡线接管
							special_counter--;
							if (special_counter == 0) {
									special_state = 0;          // 恢复正常
							}
							// 注意：这里不设置 angle_set_by_state，因此会进入下面的正常巡线
					}
						// ========== 8. 转向决策 ==========
            if (angle_set_by_state) 
						{
    // 角度已被状态机设置，直接跳转到限幅
            } 
						else
						{
                float left_ratio_adj = left_ratio;
                float right_ratio_adj = right_ratio;
         
                if (zuo == 1 && you == 1) 
								{
                    target_angle = last_target_angle;
                } else 
								{
                    sum_ratio = left_ratio_adj + right_ratio_adj;
                    if (sum_ratio < 0.1f) 
										{
                        target_angle = SERVO_ANGLE_MID;
                    } 
										else 
										{
                        err = (right_ratio_adj - left_ratio_adj) / sum_ratio;
                        kp = 30.0f;
                        angle_offset = kp * err;
                        if (angle_offset > TURN_ANGLE_MAX_RIGHT) angle_offset = TURN_ANGLE_MAX_RIGHT;
                        if (angle_offset < -TURN_ANGLE_MAX_LEFT) angle_offset = -TURN_ANGLE_MAX_LEFT;
                        target_angle = SERVO_ANGLE_MID - angle_offset;
                    }
               }
            }

            // 限幅
            if (target_angle > SERVO_ANGLE_MAX) target_angle = SERVO_ANGLE_MAX;
            if (target_angle < SERVO_ANGLE_MIN) target_angle = SERVO_ANGLE_MIN;

            last_target_angle = target_angle;
            pwm_set_duty(SERVO_PWM_PIN, (uint32)SERVO_DUTY(target_angle));

            // ========== 9. 显示图像及调试信息 ==========
            memcpy(display_buffer, mt9v03x_image, sizeof(display_buffer));
            
            // 画基础分割线
            for (i = 0; i < MT9V03X_W; i++) {
                display_buffer[dynamic_start_row][i] = 0;
                display_buffer[SEG_END_ROW][i] = 0;
            }
            for (i = dynamic_start_row; i < MT9V03X_H; i++) {
                display_buffer[i][SEG_LEFT_END] = 0;
                display_buffer[i][SEG_RIGHT_START] = 0;
            }

            
             
            x_offset = (240 - MT9V03X_W) / 2;
            y_offset = (240 - MT9V03X_H) / 2;
            ips200_show_gray_image(x_offset, y_offset, (const uint8 *)display_buffer,
                                   MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, threshold);
           
           
								if (special_state == 1) {
									ips200_show_string(0, 200, "G O             ");
							} else if (special_state == 2) {
									ips200_show_string(0, 200, "TURNING         ");
							} else if (special_state == 3) {
									sprintf(lock_str, "COOLDOWN %3d", special_counter);
									ips200_show_string(0, 200, lock_str);
							} else {
									ips200_show_string(0, 200, "no              ");
							}
                sprintf(lock_str, "%3d",cross_flag );
                ips200_show_string(120, 200, lock_str);
							
						
            mt9v03x_finish_flag = 0;
        }
    }
}