#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "ti_msp_dl_config.h"

// 编码器配置结构体
typedef struct {
    GPIO_Regs *port;         // 编码器GPIO端口
    uint32_t pin_a;          // A相引脚
    uint32_t pin_b;          // B相引脚
    uint32_t ppr;           // 每转脉冲数
} Encoder_Config_t;

// 编码器句柄结构体
typedef struct {
    Encoder_Config_t config;  // 配置参数
    volatile int32_t count;   // 当前计数值
    volatile float position;  // 当前位置（角度）
    volatile float velocity;  // 当前速度（RPM）
} Encoder_Handle_t;

// 初始化函数
void Encoder_Init(Encoder_Handle_t *handle, const Encoder_Config_t *config);

// 获取当前计数值
int32_t Encoder_GetCount(const Encoder_Handle_t *handle);

// 获取当前角度位置（度）
float Encoder_GetPosition(const Encoder_Handle_t *handle);

// 获取当前速度（RPM）
float Encoder_GetVelocity(const Encoder_Handle_t *handle);

// 重置编码器计数
void Encoder_Reset(Encoder_Handle_t *handle);

// 中断处理函数（在IRQHandler中调用）
void Encoder_IRQHandler(Encoder_Handle_t *handle);

#endif