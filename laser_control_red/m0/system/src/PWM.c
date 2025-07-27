#include <ti/driverlib/dl_timera.h>
#include <ti/driverlib/dl_timerg.h>
#include <ti/driverlib/dl_gpio.h>
#include "ti_msp_dl_config.h"
#include "PWM.h"

//初始化
void PWM_Init(void);
void Set_Freq_Duty(float_t freq, float_t duty, GPTIMER_Regs * Timer, DL_TIMER_CC_INDEX Channel) {
    if (duty > 1.0f) duty = 1.0f;
    else if (duty < 0.0f) duty = 0.0f;
    uint32_t final_duty_val;
    uint32_t final_freq_val;
    final_freq_val = CPUCLK_FREQ / freq;                // 频率换算
    DL_TimerG_setLoadValue(Timer, final_freq_val);      // 设置频率
    final_duty_val = final_freq_val * (1.0f - duty);    // 占空比换算
    DL_TimerG_setCaptureCompareValue(Timer, final_duty_val, Channel);   // 设置占空比
}
//设置频率
void PWM_SetFre(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex, uint32_t frequency_hz)
{
    // 参数有效性检查
    if (gpt == NULL || frequency_hz == 0) {
        return;
    }
    
    // 防止频率过高
    if (frequency_hz > CPUCLK_FREQ) {
        frequency_hz = CPUCLK_FREQ;
    }
    
    // 计算总的分频系数
    uint32_t total_div = CPUCLK_FREQ / frequency_hz;
    
    // 寻找最优的a和b组合
    // 频率 = CPUCLK_FREQ / ((a+1) * b)
    // 其中 a 范围是 0-255 (对应prescale 1-256)
    uint32_t best_a = 0;
    uint32_t best_b = 1;
    uint32_t min_error = UINT32_MAX;
    
    for (uint32_t a = 0; a <= 255; a++) {
        uint32_t b = total_div / (a + 1);
        
        // 确保b是有效值 (至少为1，最大为65535)
        if (b < 1) b = 1;
        if (b > 65535) continue;
        
        // 计算实际频率和误差
        uint32_t actual_freq = CPUCLK_FREQ / ((a + 1) * b);
        uint32_t error = (actual_freq > frequency_hz) ? 
                        (actual_freq - frequency_hz) : 
                        (frequency_hz - actual_freq);
        
        // 如果误差更小，更新最优值
        if (error < min_error) {
            min_error = error;
            best_a = a;
            best_b = b;
        }
        
        // 如果找到精确匹配，直接跳出
        if (error == 0) break;
    }
    
    // 应用配置
    DL_TimerA_ClockConfig ClockConfig = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = best_a
    };
    
    DL_TimerA_PWMConfig PWMConfig = {
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
        .period = best_b - 1,  // period = b - 1
        .isTimerWithFourCC = true,
        .startTimer = DL_TIMER_STOP,
    };
    
    // 设置时钟配置
    DL_Timer_setClockConfig(gpt, (DL_TimerA_ClockConfig *)&ClockConfig);
    
    // 初始化PWM模式
    DL_TimerA_initPWMMode(gpt, (DL_TimerA_PWMConfig *)&PWMConfig);
}

//设置占空比
void PWM_SetDuty(GPTIMER_Regs *gpt, DL_TIMER_CC_INDEX ccIndex, uint32_t duty_percent)
{
    uint32_t load = DL_Timer_getLoadValue(gpt);
    uint32_t outCtl = DL_Timer_getCaptureCompareOutCtl(gpt, ccIndex);
    DL_TIMER_COUNT_MODE mode = DL_Timer_getCounterMode(gpt);

    // 是否翻转，不翻转在cmpx前是高电平，翻转在cmpx前是低电平s
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












