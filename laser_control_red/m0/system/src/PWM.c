#include <ti/driverlib/dl_timera.h>
#include <ti/driverlib/dl_timerg.h>
#include <ti/driverlib/dl_gpio.h>
#include "ti_msp_dl_config.h"
#include "PWM.h"

//初始化
void PWM_Init(void);
//设置频率
void PWM_SetFre(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex, uint32_t frequency_hz)
{
    // 防止除零错误和频率过高导致溢出
    if (frequency_hz > CPUCLK_FREQ)
    {
        frequency_hz = CPUCLK_FREQ;
    }
    if (frequency_hz == 0)
    {
        frequency_hz = 1; // 最小频率为1Hz
    }
    
    // 计算定时器加载值
    uint32_t load_value = (CPUCLK_FREQ / frequency_hz);
    
    // 确保加载值至少为1
    if (load_value < 1)
    {
        load_value = 1;
    }
    
    // 设置计数器加载值
    DL_Timer_setLoadValue(gpt, load_value);
}

//设置占空比
void PWM_SetDuty(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex, uint32_t duty_percent)
{
    uint32_t load = DL_Timer_getLoadValue(gpt);
    uint32_t outCtl = DL_Timer_getCaptureCompareOutCtl(gpt, ccIndex);
    DL_TIMER_COUNT_MODE mode = DL_Timer_getCounterMode(gpt);

    // 是否翻转，不翻转在cmpx前是高电平，翻转在cmpx前是低电平
    uint8_t isInverted = (outCtl & DL_TIMER_CC_OCTL_INV_OUT_ENABLED);

    if (duty_percent > 100) duty_percent = 100;
    uint32_t highTime = ((load + 1) * duty_percent) / 100;

    uint32_t cmpx;

    switch (mode)
    {
        case DL_TIMER_COUNT_MODE_UP:
            cmpx = isInverted ? (load + 1 - highTime) : highTime;
            break;

        case DL_TIMER_COUNT_MODE_DOWN:
            cmpx = isInverted ? highTime : (load + 1 - highTime);
            break;

        case DL_TIMER_COUNT_MODE_UP_DOWN:
            highTime = (2 * (load + 1) * duty_percent) / 100;
            cmpx = isInverted ? (load + 1 - highTime / 2) : (highTime / 2);
            break;

        default:
            // 默认按 DOWN 模式处理
            cmpx = isInverted ? highTime : (load + 1 - highTime);
            break;
    }

    DL_Timer_setCaptureCompareValue(gpt, cmpx - 1, ccIndex);
}

//获取当前频率
uint32_t PWM_GetFre(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex)
{
    uint32_t load = DL_Timer_getLoadValue(gpt);
    uint32_t cmpx = DL_Timer_getCaptureCompareValue(gpt, ccIndex);

    if (cmpx == 0 || load == 0) return 0;

    // 计算频率
    return (load + 1) / (cmpx + 1);
}
//获取当前占空比
uint32_t PWM_GetDuty(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex)
{
    uint32_t load = DL_Timer_getLoadValue(gpt);
    uint32_t cmpx = DL_Timer_getCaptureCompareValue(gpt, ccIndex);

    if (cmpx == 0 || load == 0) return 0;

    // 计算占空比
    return ((cmpx + 1) * 100) / (load + 1);
}
//使能
void PWM_Start(GPTIMER_Regs *gpt)
{
    DL_Timer_startCounter(gpt);
}
//禁用
void PWM_Stop(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex)
{
    DL_Timer_stopCounter(gpt);
    //占空比设置为0
    DL_Timer_setCaptureCompareValue(gpt, 0, ccIndex);
}












