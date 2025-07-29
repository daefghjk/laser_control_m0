#ifndef __LASER_H__
#define __LASER_H__

#include "ti_msp_dl_config.h"

// 定义配置结构体
typedef struct {
    GPIO_Regs *ctrl_port;    // 控制GPIO端口
    uint32_t ctrl_pin;       // 控制引脚
} LASER_CONFIG_T;

typedef struct {
    LASER_CONFIG_T config;   // 配置信息
    uint8_t is_on;          // 激光笔状态：0-关闭，1-开启
} LASER_HANDLE_T;

// 函数声明
void LASER_Init(LASER_HANDLE_T *handle, const LASER_CONFIG_T *config);
void LASER_TurnOn(LASER_HANDLE_T *handle);
void LASER_TurnOff(LASER_HANDLE_T *handle);

#endif
