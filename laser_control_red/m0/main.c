
// #include <stdint.h>
#include "ti_msp_dl_config.h"
#include "BOARD.h"
#include "PWM.h"
#include "OLED.h"

int main(void)
{
    BOARD_Init();
    // PWM_SetFre(STEPPER_MOTOR_INST, GPIO_STEPPER_MOTOR_C0_IDX, 1000);
    // PWM_SetDuty(STEPPER_MOTOR_INST, GPIO_STEPPER_MOTOR_C0_IDX, 50);
    // PWM_Start(STEPPER_MOTOR_INST);

    while (1)
    {
        
    }
}
