#ifndef __PWM_H__
#define __PWM_H__

//初始化、设置频率/占空比、获取频率/占空比、使能/禁用PWM通道

//初始化
void PWM_Init(void);
//设置频率
void PWM_SetFre(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex, uint32_t frequency_hz);
//设置占空比
void PWM_SetDuty(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex, uint32_t duty_percent);
//获取当前频率
uint32_t PWM_GetFre(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex);
//获取当前占空比
uint32_t PWM_GetDuty(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex);
//使能PWM通道
void PWM_Start(GPTIMER_Regs *gpt);
//禁用PWM通道
void PWM_Stop(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex);

#endif