#include "ti_msp_dl_config.h"
#include "JOYSTICK.h"
#include "OLED.h"
#include "BOARD.h"

JOYSTICK_HANDLE_T joystick_handle;
OLED_Handle_t oled_handle;

void BOARD_Init(void)
{
    SYSCFG_DL_init();
    
    // OLED_Config_t oled_config = OLED_DEFAULT_CONFIG(OLED_INST);
    // OLED_Init(&oled_handle, &oled_config);

    // JOYSTICK_CONFIG_T joystick_config = JOYSTICK_DEFAULT_CONFIG(
    //     ADC12_JOYSTICK_INST, 
    //     ADC12_JOYSTICK_INST_INT_IRQN, 
    //     DMA_CH_ADC_JOYSTICK_CHAN_ID, 
    //     0.3f
    // );
    // JOYSTICK_Init(&joystick_handle, &joystick_config);
   
}

// void ADC12_JOYSTICK_INST_IRQHandler(void)
// {
//     JOYSTICK_CommonIRQHandler(&joystick_handle);
// }
