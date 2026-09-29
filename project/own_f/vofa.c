/**
 * @file    vofa.c
 * @brief   VOFA+ 在线调参 / 波形显示 驱动（STC32G144K + 逐飞开源库 移植版）
 *
 * @note    1. 上行（单片机 -> VOFA+）：JustFloat 协议
 *              N 个 float(小端) + 帧尾 0x00 0x00 0x80 0x7F
 *              VOFA+ 里"数据格式"选择 JustFloat 即可显示波形
 *
 *          2. 下行（VOFA+ -> 单片机）：自定义调参帧
 *              0xAA | 通道号(0 ~ len-1) | float 4 字节(小端) | 0xFF
 *              收到后把对应通道的值写入 param[] 指向的变量
 *
 *          3. 关键点：C251 的 float/多字节变量在内存里是【大端】存放，而 PC 是小端，
 *             所以收发的 4 个字节都要做一次顺序颠倒（原来的 STM32 版本直接用 memcpy，
 *             在这个平台上是错的，会把参数解析成乱码）
 *
 *          4. 传输层用函数指针解耦：既能走 USB 虚拟串口，也能走普通串口/无线串口
 *
 * @version 2.0（适配 STC32G144K + 逐飞开源库）
 */

#include "vofa.h"

static vofa_protocal_t  vofa_protocal;                                  // 协议运行数据
static uint8            vofa_tx[vofa_channel_num * 4 + 4];              // 上行发送缓冲
static uart_index_enum  vofa_uart_index = UART_1;                       // 串口模式下使用的串口

//-------------------------------------------------------------------------------------------------------------------
//  @brief      float -> 4 字节小端（给 PC 用）
//  @param      dat     要转换的浮点数
//  @param      buff    转换结果存放地址（4 字节）
//  @retval     void
//  @note       先 memcpy 取出内存里的 4 个字节，再按 低位在前 重新排列，
//              这样不管编译器是大端还是小端，发出去的永远是 VOFA+ 要的小端顺序
//-------------------------------------------------------------------------------------------------------------------
static void vofa_float_to_bytes (float dat, uint8 *buff)
{
    uint32 raw = 0;

    memcpy(&raw, &dat, 4);                                              // 取出 4 字节原始数据

    buff[0] = (uint8)( raw        & 0x000000FF);                        // 最低字节先发
    buff[1] = (uint8)((raw >>  8) & 0x000000FF);
    buff[2] = (uint8)((raw >> 16) & 0x000000FF);
    buff[3] = (uint8)((raw >> 24) & 0x000000FF);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      4 字节小端 -> float
//  @param      buff    收到的 4 个字节（小端顺序）
//  @retval     解析出来的浮点数
//-------------------------------------------------------------------------------------------------------------------
static float vofa_bytes_to_float (const uint8 *buff)
{
    float  dat = 0;
    uint32 raw = 0;

    raw  = (uint32)buff[0];                                             // 按小端拼回一个 32 位数据
    raw |= ((uint32)buff[1] <<  8);
    raw |= ((uint32)buff[2] << 16);
    raw |= ((uint32)buff[3] << 24);

    memcpy(&dat, &raw, 4);                                              // uint32 和 float 在内存里的字节序是一致的

    return dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      USB 虚拟串口 发送（内部使用）
//-------------------------------------------------------------------------------------------------------------------
static uint16 vofa_usb_write (const uint8 *buff, uint16 len)
{
    return usb_cdc_write_buffer(buff, len);                             // 返回剩余没发完的字节数
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      普通串口 发送（内部使用）
//-------------------------------------------------------------------------------------------------------------------
static uint16 vofa_uart_write (const uint8 *buff, uint16 len)
{
    uart_write_buffer(vofa_uart_index, buff, len);                      // 该函数内部会等待发送完成

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      协议参数配置（内部使用）
//  @param      param   参数指针数组
//  @param      len     参数个数
//  @retval     void
//-------------------------------------------------------------------------------------------------------------------
static void vofa_param_config (float **param, uint8 len)
{
    if(len > vofa_channel_num)      len = vofa_channel_num;             // 限幅，防止越界

    vofa_protocal.param   = param;
    vofa_protocal.len     = len;
    vofa_protocal.channel = 0;
    vofa_protocal.state   = vofa_head;

    memset(vofa_protocal.rx_data,       0, sizeof(vofa_protocal.rx_data));
    memset(vofa_protocal.final_data, 0, sizeof(vofa_protocal.final_data));
    memset(vofa_protocal.update,     0, sizeof(vofa_protocal.update));
    memset(vofa_tx,                  0, sizeof(vofa_tx));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      USB 虚拟串口 初始化
//  @param      param   要在线修改的参数（指针数组，下标就是通道号）
//  @param      len     参数个数
//  @retval     void
//  @note       插上下载用的那根 USB 线即可，PC 上会多出一个串口
//-------------------------------------------------------------------------------------------------------------------
void vofa_usb_init (float **param, uint8 len)
{
    usb_cdc_init();                                                     // 初始化 USB（虚拟串口）

    vofa_protocal.write = vofa_usb_write;                               // 绑定发送函数
    vofa_param_config(param, len);

    usb_cdc_rx_interrupt(vofa_usb_rx_handler);                          // 注册接收回调，收到数据会自动进状态机
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      普通串口 初始化
//  @param      uart    使用的串口（UART_1 ~ UART_8）
//  @param      param   要在线修改的参数（指针数组，下标就是通道号）
//  @param      len     参数个数
//  @retval     void
//  @note       调用本函数之前请先调用 uart_init(uart, 波特率, TX引脚, RX引脚)，
//              例如：uart_init(UART_3, 115200, UART3_TX_P01, UART3_RX_P00);
//              另外 UART_1 的接收中断里有"收到 0x7F 就复位"的逻辑（在 isr.c 里），
//              调参建议避开 UART_1
//-------------------------------------------------------------------------------------------------------------------
void vofa_uart_init (uart_index_enum uart, float **param, uint8 len)
{
    vofa_uart_index      = uart;
    vofa_protocal.write  = vofa_uart_write;                             // 绑定发送函数
    vofa_param_config(param, len);

    uart_rx_interrupt(uart, ENABLE, vofa_receive_parse);                // 打开接收中断，每个字节交给状态机
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      USB 接收回调（USB 中断里被调用）
//  @param      buff    收到的数据
//  @param      len     数据长度
//  @retval     void
//  @note       这里只做状态机解析，不做任何阻塞操作
//-------------------------------------------------------------------------------------------------------------------
void vofa_usb_rx_handler (uint8 *buff, uint16 len)
{
    uint16 i;

    for(i = 0; i < len; i++)
    {
        vofa_receive_parse(buff[i]);                                    // 一个字节一个字节地喂给状态机
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      调参帧解析状态机
//  @param      dat    收到的单个字节（串口接收回调 或 USB 回调里调用）
//  @retval     void
//  @note       帧格式：0xAA | 通道号 | 4字节 float(小端) | 0xFF
//-------------------------------------------------------------------------------------------------------------------
void vofa_receive_parse (uint8 dat)
{
    static uint8 vofa_index = 0;                                        // 数据字节计数

    switch(vofa_protocal.state)
    {
        case vofa_head:                                                 // 等帧头
        {
            if(vofa_channel_head == dat)
            {
                vofa_protocal.state = vofa_channel;
            }
        }break;

        case vofa_channel:                                              // 等通道号
        {
            if(dat < vofa_protocal.len)                                // 通道号合法
            {
                vofa_protocal.channel = dat;
                vofa_index = 0;
                vofa_protocal.state = vofa_data;
            }
            else                                                        // 通道号非法，重新找帧头
            {
                vofa_protocal.state = vofa_head;
            }
        }break;

        case vofa_data:                                                 // 收 4 个数据字节
        {
            vofa_protocal.rx_data[vofa_index++] = dat;
            if(vofa_index >= 4)
            {
                vofa_protocal.state = vofa_tail;
            }
        }break;

        case vofa_tail:                                                 // 等帧尾
        {
            if(vofa_channel_tail == dat)                               // 帧尾正确才认为这一帧有效
            {
                vofa_protocal.final_data[vofa_protocal.channel] = vofa_bytes_to_float(vofa_protocal.rx_data);
                vofa_protocal.update[vofa_protocal.channel]     = 1;
            }

            vofa_index = 0;
            vofa_protocal.state = vofa_head;
        }break;

        default:                                                        // 异常状态回到起点
        {
            vofa_protocal.state = vofa_head;
        }break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      发送 num 条曲线（JustFloat 协议）
//  @param      dat     数据首地址
//  @param      num     数据个数 1 ~ vofa_channel_num
//  @retval     void
//  @note       帧 = N 个 float(小端) + 0x00 0x00 0x80 0x7F
//              只能在主循环里调用，不要放中断里
//-------------------------------------------------------------------------------------------------------------------
void vofa_send_wave (float *dat, uint8 num)
{
    uint8 i;

    if(NULL == vofa_protocal.write || NULL == dat)      return;         // 还没初始化就直接返回

    if(num > vofa_channel_num)                          num = vofa_channel_num;

    for(i = 0; i < num; i++)
    {
        vofa_float_to_bytes(dat[i], &vofa_tx[i * 4]);                   // 逐个转成小端字节
    }

    vofa_tx[num * 4 + 0] = 0x00;                                        // JustFloat 帧尾：+inf 的小端表示
    vofa_tx[num * 4 + 1] = 0x00;
    vofa_tx[num * 4 + 2] = 0x80;
    vofa_tx[num * 4 + 3] = 0x7F;

    vofa_protocal.write(vofa_tx, (uint16)(num * 4 + 4));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      发送当前所有参数值（方便在 VOFA+ 里看参数曲线）
//  @retval     void
//-------------------------------------------------------------------------------------------------------------------
void vofa_send_param (void)
{
    float tmp[vofa_channel_num];
    uint8 i;

    if(NULL == vofa_protocal.param)     return;

    for(i = 0; i < vofa_protocal.len; i++)
    {
        tmp[i] = *(vofa_protocal.param[i]);
    }

    vofa_send_wave(tmp, vofa_protocal.len);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      把收到的参数写入变量（应用新参数）
//  @retval     void
//  @note       开销极小，建议主循环里每圈都调用
//-------------------------------------------------------------------------------------------------------------------
void vofa_apply_param (void)
{
    uint8 i;

    if(NULL == vofa_protocal.param)     return;

    for(i = 0; i < vofa_protocal.len; i++)
    {
        if(vofa_protocal.update[i])                                     // 这一通道收到了新值
        {
            *(vofa_protocal.param[i]) = vofa_protocal.final_data[i];
            vofa_protocal.update[i]   = 0;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      发送参数 + 应用参数（兼容旧接口）
//  @retval     void
//  @note       原来的 STM32 版本就是"发一帧参数顺便把收到的参数应用上"，
//              现在拆成了 vofa_send_param() 和 vofa_apply_param() 两个函数，
//              需要周期发送的时候用这个接口即可
//-------------------------------------------------------------------------------------------------------------------
void vofa_update_param (void)
{
    vofa_send_param();
    vofa_apply_param();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      查询某个通道是否收到了新值
//  @param      channel     通道号
//  @retval     1 = 有新值（还没被 vofa_apply_param 应用）  0 = 没有
//-------------------------------------------------------------------------------------------------------------------
uint8 vofa_param_updated (uint8 channel)
{
    if(channel >= vofa_protocal.len)    return 0;

    return vofa_protocal.update[channel];
}
