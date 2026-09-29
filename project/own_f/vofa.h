#ifndef __VOFA_H__
#define __VOFA_H__

#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
//  VOFA+ 在线调参 / 波形显示 接口（STC32G144K + 逐飞开源库 移植版）
//
//  上行（单片机 -> VOFA+）：JustFloat 协议
//      N 个 float(小端) + 帧尾 0x00 0x00 0x80 0x7F
//      VOFA+ 里把"数据格式"选成 JustFloat，就能直接显示波形
//
//  下行（VOFA+ -> 单片机）：自定义调参帧
//      0xAA | 通道号(0 ~ len-1) | float 4 字节(小端) | 0xFF
//      收到后写入 param[] 里对应指针指向的变量，实现"边跑边调参"
//
//  最快上手（用 USB 虚拟串口，不用额外接线，插着下载线就行）：
//      float  kp = 10.0f;
//      float  ki = 0.5f;
//      float *param_table[2] = { &kp, &ki };     // 数组下标 = VOFA+ 里的通道号
//
//      vofa_usb_init(param_table, 2);            // 初始化（放在时钟初始化之后）
//
//      while(1)
//      {
//          vofa_apply_param();                   // 把上位机发来的新参数写进变量
//          vofa_send_wave(wave, n);              // 把 n 条曲线发给 VOFA+ 显示（要限速）
//      }
//
//  注意事项：
//      1. C251 的 float 在内存里是【大端】，而 PC(VOFA+) 是小端，本驱动内部已经做了字节序转换，
//         千万不要像 STM32 版本那样直接用 memcpy 收发 float，否则数值会完全错乱
//      2. vofa_send_wave / vofa_send_param 是阻塞发送，只能在主循环里调用；
//         不要在中断里调用（会长时间阻塞中断，尤其是 USB 发送）
//      3. 下行帧的通道号必须小于 vofa_channel_num，否则该帧会被丢弃
//-------------------------------------------------------------------------------------------------------------------

#define vofa_channel_num                (8)          // 通道个数上限（可调参数个数，也是单帧最多发送的曲线条数）
#define vofa_channel_head               (0xaa)       // 下行调参帧 帧头
#define vofa_channel_tail               (0xff)       // 下行调参帧 帧尾

typedef enum
{
    vofa_head = 1,                                   // 等待帧头
    vofa_channel,                                    // 等待通道号
    vofa_data,                                       // 接收 4 个数据字节
    vofa_tail                                        // 等待帧尾
}vofa_state_t;

typedef struct
{
    float   **param;                                 // 需要在线修改的参数（外部定义的指针数组）
    uint8     len;                                   // 参数个数 1 ~ vofa_channel_num

    uint8     channel;                               // 当前正在解析的通道号
    uint8     rx_data[4];                               // 当前通道收到的 4 个字节（小端顺序）
    float     final_data[vofa_channel_num];          // 解析出来的参数值
    uint8     update[vofa_channel_num];              // 新参数标志 1 = 有新值还没应用

    vofa_state_t state;                              // 解析状态机

    uint16  (*write)(const uint8 *buff, uint16 len); // 传输层写函数（USB CDC 或者串口）
}vofa_protocal_t;

void    vofa_usb_init       (float **param, uint8 len);                             // 用 USB 虚拟串口传输（推荐）
void    vofa_uart_init      (uart_index_enum uart, float **param, uint8 len);       // 用普通串口传输（需要先自己调用 uart_init）

void    vofa_usb_rx_handler (uint8 *buff, uint16 len);                              // USB 接收回调（内部注册）
void    vofa_receive_parse  (uint8 dat);                                           // 逐字节解析（串口接收回调）

void    vofa_send_wave      (float *dat, uint8 num);                                // 发送 num 条曲线（JustFloat 帧）
void    vofa_send_param     (void);                                                 // 发送当前所有参数值
void    vofa_apply_param    (void);                                                 // 应用收到的参数（建议主循环里一直调用）
void    vofa_update_param   (void);                                                 // = 发送参数 + 应用参数（兼容原来的接口）

uint8   vofa_param_updated  (uint8 channel);                                        // 查询某个通道是否收到了新值

#endif
