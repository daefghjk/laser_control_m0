#include "ti_msp_dl_config.h"
#include "BOARD.h"
#include "OLED.h"
#include "PWM.h"
#include "STEPPER.h"


int main(void)
{
    BOARD_Init();

    // STEPPER_Enable(&stepper1_handle);
    // STEPPER_SetSpeed(&stepper1_handle, 500.0f); // 设置速度为100Hz
    // STEPPER_MoveAngle(&stepper1_handle, 360.0f); //

    PWM_Start(STEPPER_1_INST);
    PWM_SetDuty(STEPPER_1_INST, GPIO_STEPPER_1_C0_IDX, 50);
    // PWM_SetFre(STEPPER_1_INST, GPIO_STEPPER_1_C0_IDX, 10000);
    // PWM_Start(STEPPER_1_INST);
    // OLED_ShowString(&oled_handle, 1, 1, "Hello, World!");

    while (1)
    {
        
    }
}