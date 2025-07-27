#include "ti_msp_dl_config.h"
#include "BOARD.h"
#include "OLED.h"
#include "PWM.h"

int main(void)
{
    BOARD_Init();
    PWM_Start(STEPPER_MOTOR_INST);
    PWM_SetFre(STEPPER_MOTOR_INST, GPIO_STEPPER_MOTOR_C1_IDX, 102);
    PWM_SetDuty(STEPPER_MOTOR_INST, GPIO_STEPPER_MOTOR_C1_IDX, 25);
    PWM_Start(STEPPER_MOTOR_INST);
    // Set_Freq_Duty(1000.0f, 0.25f, STEPPER_MOTOR_INST, GPIO_STEPPER_MOTOR_C0_IDX);
    // OLED_ShowString(&oled_handle, 1, 1, "Hello, World!");

    while (1)
    {
        
    }
}
