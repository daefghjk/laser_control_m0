#ifndef __BOARD_H__
#define __BOARD_H__

#include "JOYSTICK.h"
#include "OLED.h"
#include "STEPPER.h"

extern JOYSTICK_HANDLE_T joystick_handle;
extern OLED_Handle_t oled_handle;
extern STEPPER_HANDLE_T stepper1_handle, stepper2_handle;

void BOARD_Init(void);

#endif /* __BOARD_H__ */