#include "LASER.h"

void LASER_Init(LASER_HANDLE_T *handle, const LASER_CONFIG_T *config)
{
    // 复制配置
    handle->config = *config;
    
    // 初始化状态
    handle->is_on = 0;
    
    // 确保激光笔初始状态为关闭
    DL_GPIO_clearPins(handle->config.ctrl_port, handle->config.ctrl_pin);
}

void LASER_TurnOn(LASER_HANDLE_T *handle)
{
    if (!handle->is_on) {
        DL_GPIO_setPins(handle->config.ctrl_port, handle->config.ctrl_pin);
        handle->is_on = 1;
    }
}

void LASER_TurnOff(LASER_HANDLE_T *handle)
{
    if (handle->is_on) {
        DL_GPIO_clearPins(handle->config.ctrl_port, handle->config.ctrl_pin);
        handle->is_on = 0;
    }
}
