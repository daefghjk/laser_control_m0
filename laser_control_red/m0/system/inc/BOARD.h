#ifndef __BOARD_H__
#define __BOARD_H__

#include "ti_msp_dl_config.h"
// 系统级别
#include "BOARD.h"
#include "DELAY.h"
#include "PWM.h"
// 硬件层
#include "OLED.h"
#include "JOYSTICK.h"
#include "BUTTON.h"
#include "BEEP.h"
#include "K230.h"
// 硬件层（可能不单独使用）
#include "ENCODER.h"
#include "LASER.h"
#include "GRAY.h"
#include "MOTOR.h"
#include "STEPPER.h"
// 控制层
#include "GIMBAL_CONTROL.h"
#include "MOTOR_CONTROL.h"

extern volatile uint64_t Systick_Count;

extern JOYSTICK_HANDLE_T joystick_handle;
extern OLED_Handle_t oled_handle;
extern STEPPER_HANDLE_T stepper1_handle, stepper2_handle;
extern BTN_GROUP_HANDLE_T btn_group;
extern LASER_HANDLE_T laser_handle;

void BOARD_Init(void);

#endif /* __BOARD_H__ */