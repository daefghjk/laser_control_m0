#include "ti_msp_dl_config.h"
#include "JOYSTICK.h"

JOYSTICK_HANDLE_T joystick_handle = {
    .joystick_ADC_INST = ADC12_JOYSTICK_INST,
    .joystick_ADC_IRQN = ADC12_JOYSTICK_INST_INT_IRQN,
    .joystick_DMA_channel_id = DMA_CH_ADC_JOYSTICK_CHAN_ID,

    .joystick_min_Rel_voltage = 0.5f, // 认为有移动的相对中值的最小电压
    .joystick_max_voltage = 3.3f, // 最大电压

    .joystick_ADCsamples = {0},
    .joystick_x_ADC_value = 0,
    .joystick_y_ADC_value = 0,

    .joystick_x_voltage = 0.0f,
    .joystick_y_voltage = 0.0f,

    .joystick_x_direction = JOYSTICK_DIRECTION_NONE,
    .joystick_y_direction = JOYSTICK_DIRECTION_NONE,
    .joystick_x_speed = 0,
    .joystick_y_speed = 0,
};

void JOYSTICK_Init(JOYSTICK_HANDLE_T *handle)
{
    DL_DMA_setSrcAddr(DMA, handle->joystick_DMA_channel_id,
        (uint32_t) DL_ADC12_getFIFOAddress(handle->joystick_ADC_INST));
    DL_DMA_setDestAddr(DMA, handle->joystick_DMA_channel_id, (uint32_t) handle->joystick_ADCsamples);
    DL_DMA_enableChannel(DMA, handle->joystick_DMA_channel_id);

    NVIC_ClearPendingIRQ(handle->joystick_ADC_IRQN);
    NVIC_EnableIRQ(handle->joystick_ADC_IRQN);
}

void JOYSTICK_startConversion(JOYSTICK_HANDLE_T *handle)
{
    JOYSTICK_clearSamples(handle);
    DL_ADC12_clearInterruptStatus(handle->joystick_ADC_INST, DL_ADC12_INTERRUPT_DMA_DONE);
    DL_ADC12_enableConversions(handle->joystick_ADC_INST);
    DL_ADC12_startConversion(handle->joystick_ADC_INST);
}

void JOYSTICK_stopConversion(JOYSTICK_HANDLE_T *handle)
{
    DL_ADC12_stopConversion(handle->joystick_ADC_INST);
    JOYSTICK_clearSamples(handle);
}

void JOYSTICK_clearSamples(JOYSTICK_HANDLE_T *handle)
{
    for (uint8_t i = 0; i < 12; i++) {
        handle->joystick_ADCsamples[i] = 0;
    }
    handle->joystick_x_ADC_value = 0;
    handle->joystick_y_ADC_value = 0;
    handle->joystick_x_direction = JOYSTICK_DIRECTION_NONE;
    handle->joystick_y_direction = JOYSTICK_DIRECTION_NONE;
    handle->joystick_x_speed = 0;
    handle->joystick_y_speed = 0;
}

void JOYSTICK_SpeedCalculation(JOYSTICK_HANDLE_T *handle)
{
    float center_voltage = handle->joystick_max_voltage / 2.0f;  // 中值电压
    float min_threshold = handle->joystick_min_Rel_voltage;       // 相对最小电压
    
    // 计算 X 方向的方向和速度
    if (handle->joystick_x_voltage > (center_voltage + min_threshold)) {
        // 右方向：从中值+相对最小电压到最大电压
        float speed_range = handle->joystick_max_voltage - (center_voltage + min_threshold);
        float normalized_speed = (handle->joystick_x_voltage - (center_voltage + min_threshold)) / speed_range;
        handle->joystick_x_speed = (uint8_t)(normalized_speed * 100.0f);
        if (handle->joystick_x_speed > 100) handle->joystick_x_speed = 100;
        
        // 设置为右方向
        handle->joystick_x_direction = JOYSTICK_DIRECTION_RIGHT;
        if (handle->joystick_x_is_inverted)
            // 如果是反向，则将方向设置为左
            handle->joystick_x_direction = JOYSTICK_DIRECTION_LEFT;
    } 
    else if (handle->joystick_x_voltage < (center_voltage - min_threshold))
    {
        // 左方向：从中值-相对最小电压到0
        float speed_range = (center_voltage - min_threshold) - 0.0f;
        float normalized_speed = ((center_voltage - min_threshold) - handle->joystick_x_voltage) / speed_range;
        handle->joystick_x_speed = (uint8_t)(normalized_speed * 100.0f);
        if (handle->joystick_x_speed > 100) handle->joystick_x_speed = 100;
        
        // 设置为左方向
        handle->joystick_x_direction = JOYSTICK_DIRECTION_LEFT;
        if (handle->joystick_x_is_inverted)
            // 如果是反向，则将方向设置为右
            handle->joystick_x_direction = JOYSTICK_DIRECTION_RIGHT;
    } 
    else 
    {
        // X方向在死区内
        handle->joystick_x_direction = JOYSTICK_DIRECTION_NONE;
        handle->joystick_x_speed = 0;
    }
    
    // 计算 Y 方向的方向和速度
    if (handle->joystick_y_voltage > (center_voltage + min_threshold)) {
        // 上方向：从中值+相对最小电压到最大电压
        float speed_range = handle->joystick_max_voltage - (center_voltage + min_threshold);
        float normalized_speed = (handle->joystick_y_voltage - (center_voltage + min_threshold)) / speed_range;
        handle->joystick_y_speed = (uint8_t)(normalized_speed * 100.0f);
        if (handle->joystick_y_speed > 100) handle->joystick_y_speed = 100;
        
        // 设置为上方向
        handle->joystick_y_direction = JOYSTICK_DIRECTION_UP;
        if (handle->joystick_y_is_inverted)
            // 如果是反向，则将方向设置为下
            handle->joystick_y_direction = JOYSTICK_DIRECTION_DOWN;
    } 
    else if (handle->joystick_y_voltage < (center_voltage - min_threshold))
    {
        // 下方向：从中值-相对最小电压到0
        float speed_range = (center_voltage - min_threshold) - 0.0f;
        float normalized_speed = ((center_voltage - min_threshold) - handle->joystick_y_voltage) / speed_range;
        handle->joystick_y_speed = (uint8_t)(normalized_speed * 100.0f);
        if (handle->joystick_y_speed > 100) handle->joystick_y_speed = 100;
        
        // 设置为下方向
        handle->joystick_y_direction = JOYSTICK_DIRECTION_DOWN;
        if (handle->joystick_y_is_inverted)
            // 如果是反向，则将方向设置为上
            handle->joystick_y_direction = JOYSTICK_DIRECTION_UP;
    } 
    else 
    {
        // Y方向在死区内
        handle->joystick_y_direction = JOYSTICK_DIRECTION_NONE;
        handle->joystick_y_speed = 0;
    }
}

void ADC12_JOYSTICK_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(joystick_handle.joystick_ADC_INST)) {
        case DL_ADC12_IIDX_DMA_DONE:
        {
            uint32_t x_ADC_value_sum = 0, y_ADC_value_sum = 0;
            for (uint8_t i = 0; i < 6; i++) {
                x_ADC_value_sum += joystick_handle.joystick_ADCsamples[i * 2];   // X values are in even indices
                y_ADC_value_sum += joystick_handle.joystick_ADCsamples[i * 2 + 1]; // Y values are in odd indices
            }
            joystick_handle.joystick_x_ADC_value = x_ADC_value_sum / 6;
            joystick_handle.joystick_y_ADC_value = y_ADC_value_sum / 6;
            joystick_handle.joystick_x_voltage = (joystick_handle.joystick_x_ADC_value / 4095.0f) * joystick_handle.joystick_max_voltage;
            joystick_handle.joystick_y_voltage = (joystick_handle.joystick_y_ADC_value / 4095.0f) * joystick_handle.joystick_max_voltage;

            JOYSTICK_SpeedCalculation(&joystick_handle);

            DL_ADC12_clearInterruptStatus(joystick_handle.joystick_ADC_INST, DL_ADC12_INTERRUPT_DMA_DONE);
            break;
        }
        default:
            break;
    }
}
