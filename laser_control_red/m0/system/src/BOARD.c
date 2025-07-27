#include "ti_msp_dl_config.h"
#include "JOYSTICK.h"
#include "OLED.h"
#include "BOARD.h"

JOYSTICK_HANDLE_T joystick_handle;

void BOARD_Init(void)
{
    SYSCFG_DL_init();
    OLED_Init();

    JOYSTICK_CONFIG_T joystick_config = {
        .joystick_ADC_INST = ADC12_JOYSTICK_INST,
        .joystick_ADC_IRQN = ADC12_JOYSTICK_INST_INT_IRQN,
        .joystick_DMA_channel_id = DMA_CH_ADC_JOYSTICK_CHAN_ID,
        .joystick_min_Rel_voltage = 0.5f, // 认为有移动的相对中值的最小电压
        .joystick_max_voltage = 3.3f, // 最大电压
        .joystick_x_is_inverted = 0, // 是否反向
        .joystick_y_is_inverted = 0, // 是否反向
    };
    JOYSTICK_Init(&joystick_handle, &joystick_config);
   
}

void ADC12_JOYSTICK_INST_IRQHandler(void)
{
    JOYSTICK_CommonIRQHandler(&joystick_handle);
}
