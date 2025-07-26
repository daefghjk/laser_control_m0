#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include <stdint.h>
#include <ti/devices/msp/msp.h>

typedef enum {
    JOYSTICK_DIRECTION_NONE = 0,
    JOYSTICK_DIRECTION_UP,
    JOYSTICK_DIRECTION_DOWN,
    JOYSTICK_DIRECTION_LEFT,
    JOYSTICK_DIRECTION_RIGHT,
} JOYSTICK_DIRECTION_T;

typedef struct {
    ADC12_Regs * joystick_ADC_INST;
    IRQn_Type joystick_ADC_IRQN;
    int joystick_DMA_channel_id;
    
    float joystick_min_Rel_voltage; // 认为有移动的相对中值的最小电压
    float joystick_max_voltage; // 最大电压
    uint8_t joystick_x_is_inverted; // 是否反向
    uint8_t joystick_y_is_inverted; // 是否反向

    volatile uint16_t joystick_ADCsamples[12];
    volatile uint16_t joystick_x_ADC_value;
    volatile uint16_t joystick_y_ADC_value;

    volatile float joystick_x_voltage;
    volatile float joystick_y_voltage;

    volatile JOYSTICK_DIRECTION_T joystick_x_direction;
    volatile JOYSTICK_DIRECTION_T joystick_y_direction;
    uint8_t joystick_x_speed;//0-100
    uint8_t joystick_y_speed;//0-100
} JOYSTICK_HANDLE_T;

extern JOYSTICK_HANDLE_T joystick_handle;

void JOYSTICK_Init(JOYSTICK_HANDLE_T *handle);
void JOYSTICK_startConversion(JOYSTICK_HANDLE_T *handle);
void JOYSTICK_stopConversion(JOYSTICK_HANDLE_T *handle);
void JOYSTICK_clearSamples(JOYSTICK_HANDLE_T *handle);

#endif /* __JOYSTICK_H__ */
