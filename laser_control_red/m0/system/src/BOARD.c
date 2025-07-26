#include "ti_msp_dl_config.h"
#include "JOYSTICK.h"
#include "OLED.h"
#include "BOARD.h"

void BOARD_Init(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    JOYSTICK_Init(&joystick_handle);
}
