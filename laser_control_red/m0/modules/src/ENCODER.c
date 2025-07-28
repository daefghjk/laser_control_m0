#include "ENCODER.h"

void Encoder_Init(Encoder_Handle_t *handle, const Encoder_Config_t *config)
{
    if (handle == NULL || config == NULL) {
        return;
    }

    // 初始化配置
    handle->config = *config;
    handle->count = 0;
    handle->position = 0.0f;
    handle->velocity = 0.0f;

    // 配置GPIO中断
    DL_GPIO_enableInterrupt(handle->config.port, handle->config.pin_a);
    // DL_GPIO_setInterrupt(handle->config.port, handle->config.pin_a, DL_GPIO_EDGE_BOTH);//setInterruptEdge
}

int32_t Encoder_GetCount(const Encoder_Handle_t *handle)
{
    return handle->count;
}

float Encoder_GetPosition(const Encoder_Handle_t *handle)
{
    return (float)handle->count * 360.0f / handle->config.ppr;
}

float Encoder_GetVelocity(const Encoder_Handle_t *handle)
{
    return handle->velocity;
}

void Encoder_Reset(Encoder_Handle_t *handle)
{
    if (handle != NULL) {
        handle->count = 0;
        handle->position = 0.0f;
        handle->velocity = 0.0f;
    }
}

void Encoder_IRQHandler(Encoder_Handle_t *handle)
{
    if (handle == NULL) {
        return;
    }

    if (DL_GPIO_getEnabledInterruptStatus(handle->config.port, handle->config.pin_a)) {
        // 根据B相电平判断方向
        if (DL_GPIO_readPins(handle->config.port, handle->config.pin_b)) {
            handle->count--;
        } else {
            handle->count++;
        }
        
        // 更新位置
        handle->position = Encoder_GetPosition(handle);
        
        // 清除中断标志
        DL_GPIO_clearInterruptStatus(handle->config.port, handle->config.pin_a);
    }
}
