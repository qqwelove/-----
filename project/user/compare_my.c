//255 白 0 黑
//======================================== 硬件配置 ========================================
// 舵机配置
#define SERVO_PWM_PIN       PWME_CH1P_PA0
#define SERVO_FREQ          150
#define SERVO_DUTY(x)   	((float)PWM_DUTY_MAX / (1000.0f/(float)SERVO_FREQ) * (0.5f + (float)(x)/90.0f))


//电机配置
#define DIR_1               ( IO_P75 )
#define PWM_1               ( PWMB_CH1_P74 )
                              
#define DIR_2               ( IO_P77 )
#define PWM_2               ( PWMB_CH3_P76 )

#define FREQ               (50)                                                
#define PWM_1_              (PWMF_CH1_PA1)
#define PWM_3_              (PWMF_CH3_PA5)

//======================================== 全局变量 ========================================
uint8 xdata display_buffer[MT9V03X_H][MT9V03X_W];
uint8 left_line[MT9V03X_H];                  // 左边界
uint8 right_line[MT9V03X_H];                 // 右边界
uint8 mid_line[MT9V03X_H];                   // 中线
uint8 left_find_flag[MT9V03X_H];   // 1=找到左线，0=丢失
uint8 right_find_flag[MT9V03X_H];  // 1=找到右线，0=丢失
uint8 Stop_Flag=0;
char debug_str[20];
uint8 tar_th;
uint8 left_record;//识别左边线记录
uint8 outp,aa,bb,cc;
uint8 count,flag;


uint8 my_adapt_threshold(uint8 *image, uint16 col, uint16 row)   
{
		#define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = 0;
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

void motor_callback(void);

void motor_init(void) 
{
    gpio_init(DIR_1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(DIR_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_1, 17000, 0);//17khz d
    pwm_init(PWM_2, 17000, 0);
}

void servo_init(void)
{
  pwm_init(SERVO_PWM_PIN, SERVO_FREQ, SERVO_DUTY(SERVO_MID));
}
void encoder_init(void)
{
	encoder_dir_init(PWMC_ENCODER, PWMC_ENCODER_CH1P_P40, PWMC_ENCODER_CH2P_P42);
  encoder_dir_init(PWMA_ENCODER, PWMA_ENCODER_CH1P_P60, PWMA_ENCODER_CH2P_P62);	
}
void burshless_init(void)
{
	pwm_init(PWM_1_, FREQ, 0);                   // PWM 通道1 初始化频率 50Hz  占空比初始为 0
	pwm_init(PWM_3_, FREQ, 0);                   // PWM 通道2 初始化频率 50Hz  占空比初始为 0
}
void init_all(void)
{
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    ips200_init();
    motor_init();
//    burshless_init();
	  encoder_init();
    servo_init();
	  laser_init(); 
    while (mt9v03x_init()) 
    {
        system_delay_ms(500);
    }
//		
		
		pid_init(&motor_r, 510, 17, 0);
		pid_init(&motor_l, 510, 17, 0);
}


int16 speed_l = 0;
int16 speed_r = 0;
uint8 control_row;
uint8 ok;
void encoder_update(void)
{
		speed_l = encoder_get_count(PWMC_ENCODER);
		speed_r = -encoder_get_count(PWMA_ENCODER);
	
		encoder_clear_count(PWMC_ENCODER);
	  encoder_clear_count(PWMA_ENCODER);
	
}


uint16 x_offset, y_offset;
uint8  avg,threshold;
void display(void)
{
	  
	  uint16 i, j; 
	            // ========== 9. 显示图像及调试信息 ==========
            memcpy(display_buffer, mt9v03x_image, sizeof(display_buffer));
//															
						//画计算后的中点
						for(i=0;i<5;i++)
						{
							if(display_buffer[119-control_row+i][mid_line[control_row]]>=threshold)//为白
              display_buffer[119-control_row+i][mid_line[control_row]]=0;
							else
							display_buffer[119-control_row+i][mid_line[control_row]]=255;	
						}
						for(i=0;i<5;i++)
						{
							if(display_buffer[119-control_row][mid_line[control_row]+i]>=threshold)
              display_buffer[119-control_row][mid_line[control_row]+i]=0;
							else
						  display_buffer[119-control_row][mid_line[control_row]+i]=255;
						}			
////            
            x_offset = (240 - MT9V03X_W) / 2;
            y_offset = (240 - MT9V03X_H) / 2;
            ips200_show_gray_image(x_offset, y_offset, (const uint8 *)display_buffer,
                                   MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, threshold);
            // 显示控制行和角度
//            sprintf(debug_str, "CTL:%2d  ANG:%3.1f", control_row, target_angle);
//            ips200_show_string(0, 200, debug_str);
//						
            sprintf(debug_str, "L:%3d  R:%3d  MID:%3d",
            left_line[19], right_line[19], mid_line[19]);
            ips200_show_string(0, 216, debug_str);
//				    e
//					  sprintf(debug_str, "l:%d, r:%d",speed_l, speed_r);
//            ips200_show_string(0, 30, debug_str);
//						sprintf(debug_str, "up_sum:%.1f", up_sum);
//						ips200_show_string(0, 120, debug_str);
						  
						sprintf(debug_str, "%d",ok);
						ips200_show_string(0, 30, debug_str);
//						 sprintf(debug_str, "%d",left_record);
//						ips200_show_string(30, 30, debug_str);
						
						ips200_show_uint8(0, 0, lock_mid_flag);
						ips200_show_uint8(30, 0, jump_appay[1]);
						ips200_show_uint8(0, 30, jump_appay[2]);
						ips200_show_uint8(30, 30, jump_appay[3]);
			
	
}

//======================================== 主函数 ========================================
int speed_min = 10;//10
int speed_max = 14;//14
int motor_speed = 5;
void main(void) 
{   
    static uint8 out_of_track_counter = 0;
    float target_angle;
    float last_target_angle = SERVO_MID;
    uint32 sum = 0;  
	
	  uint8 row,col;
    uint8 state_huan=0;
	  uint8 state_shizi=0;

	  uint8 cishu=0,shizi_sign=0;
	  uint8 jishu=0;
	  uint16 duty = 0;
		uint8 laser_idx = 0xFF;
	   
    init_all(); 
	
	  system_delay_ms(1000);

    pit_ms_init(TIM0_PIT, 1, motor_callback);
//	  gpio_init(IO_P96, GPO, GPIO_LOW, GPO_PUSH_PULL);//p96  左到右 P84 P66 P96 P63 p61
//    gpio_init(IO_P96, GPO, GPIO_LOW, GPO_PUSH_PULL);
//    gpio_set_level(IO_P96, 0);
	  
//    gpio_init(IO_P96, GPO, GPIO_LOW, GPO_PUSH_PULL);//p96  左到右 P84 P66 P96 P63 p61
//    gpio_set_level(IO_P96, 1);
//		system_delay_ms(500);
//		gpio_set_level(IO_P96, 0);
//		 
	
		
//    duty = 1.0 / 20 * 10000;        // (1ms/20ms * 10000)（10000是PWM的满占空比时候的值） 10000为PWM最大值
   
    while (1) 
    {

					  
			
        if (mt9v03x_finish_flag) 
        { 
								
          //大津法输出阈值
            threshold = my_adapt_threshold(mt9v03x_image[0], 188, 120);
					
						// ---------- 2. 逐行扫描提取边线 ----------
						scan_lines(threshold, left_line, right_line, mid_line,
											 START_ROW, END_ROW, SEARCH_RANGE);	
        				
					  nb();
						
						Zebra_Check(mt9v03x_image[0]);
					
            // ===== 十字补线：必须在舵机计算前修改中线数组 =====
            repire_cross(threshold, 10);

            // 补线锁定期跳过 NoLine，防止十字路口白色误判出界
            if(lock_mid_flag == 0)
            {
                NoLine(mt9v03x_image[0]);
            }

					  //============十字状态机===================

					  if(shizi_state_1() && state_huan==0)							
						{						 
						 shizi_sign=1;
						}
						if(shizi_sign==1 && shizi_state_2())
						{
						 shizi_sign=2;
						}
						if(shizi_sign==2 && shizi_state_3())
						{
						 shizi_sign=0;
						}
			
									//============圆环状态机==================
						//左环岛
					  if(left_state_1() && state_huan!=2 && cishu==0 && shizi_sign ==0) //jishu==0
						{	 
							state_huan=1;
							cishu=1;
						}
						if (state_huan==1)
						{
							Repair_huandao(1,80);						
						}		
						if(state_huan==1 && left_state_2())
						{
              state_huan=2;				
						}
						if(state_huan==2 && left_state_3())
						{
					    state_huan=3;
						}
						if(state_huan==3)
						{							
					    mid_line[CONTROL_ROW_OFFSET]=70;//73 65  94-24
						}
						if(state_huan==3 && left_state_4())
						{
						  state_huan=4;
						}
            if(state_huan==4 && left_state_5())
						{
              state_huan=5;
						}	
						if(state_huan==5)
						{
						  mid_line[CONTROL_ROW_OFFSET]=60;
	          	
						}
						if(state_huan==5 && left_state_6())
						{
							state_huan=6;			
						}
						if(state_huan==6)	
						{
						  Repair_huandao(1,83);
            }
						if(state_huan==6 && left_state_7())
						{
						  state_huan=0;							
              cishu=0;  
//							jishu=1;		
						}				
					
												// ---------- 3. 计算控制行 ----------
						control_row = CONTROL_ROW_OFFSET;   // 直接使用数组索引 35
            if (control_row >= TRACK_ROWS) control_row = TRACK_ROWS - 1;

						// ---------- 7. 计算舵机角度 ----------
					target_angle = compute_servo_angle(mid_line, control_row, IMG_CENTER,
																						 SERVO_KP, SERVO_KD, SERVO_MID, SERVO_MIN, SERVO_MAX);
					
						
  					// ---------- 5. 限幅输出 ----------
            if (target_angle > SERVO_MAX) target_angle = SERVO_MAX;
            if (target_angle < SERVO_MIN) target_angle = SERVO_MIN;
//						last_target_angle = target_angle;
		
            pwm_set_duty(SERVO_PWM_PIN, (uint32)SERVO_DUTY(target_angle));
						
						
						motor_speed = speed_min + Fuzzy_single(P_Speed_value, D_Speed_value, speed_max - speed_min, rule_single_sp, up_sum, 0);

            mt9v03x_finish_flag = 0;
        }
				display();
			}
		
}

float pwm_l = 0;
float pwm_r = 0;

void motor_callback(void)
{ 
	  encoder_update();

		// 如果停车标志置位，直接关闭电机并返回，不运行PID
		if(Stop_Flag == 1)
		{
				pwm_set_duty(PWM_1, 0);
				pwm_set_duty(PWM_2, 0);
				return;
		}
	
		pid_set_target(&motor_r, motor_speed);
		pid_set_target(&motor_l, motor_speed);
	 
		pwm_r = pid_incre(&motor_r, speed_r);
		pwm_r = limit_float(pwm_r, -3000, 3000);
		
		pwm_l = pid_incre(&motor_l, speed_l);
		pwm_l = limit_float(pwm_l, -3000, 3000);


		if(pwm_r > 0)
		{
				gpio_set_level(DIR_2, 0);
				pwm_set_duty(PWM_2, (int)pwm_r);

		}
		else 
		{
				gpio_set_level(DIR_2, 1);
				pwm_set_duty(PWM_2, (int)(-pwm_r));
		}
		
		if(pwm_l > 0)
		{
				gpio_set_level(DIR_1, 1);
				pwm_set_duty(PWM_1, (int)pwm_l);

		}
		else 
		{
				gpio_set_level(DIR_1, 0);
				pwm_set_duty(PWM_1, (int)(-pwm_l));
		}
		
    if(ok==3)
		{
			flag=1;
		}
		if(flag==1)
		 {
		count++;
		if(count==30)
		 {
			count=0;	
			gpio_set_level(IO_P96, 0);	
			flag=0;
		 }	
		}
}

/* uint8 zebra_detect(void)
{
	uint8 i=0,j=0;
    for(i=0; i<180; i++)
    {
        if(mt9v03x_image[110][i]<threshold && mt9v03x_image[110][i+1]>threshold)
        {
            j++;
        }
		//连续五次识别到条纹
		if(j>5) return 1;//识别到斑马线
    }
	return 0;
}
 */

		