/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_LFXT_PORT                                                     GPIOA
#define GPIO_LFXIN_PIN                                             DL_GPIO_PIN_3
#define GPIO_LFXIN_IOMUX                                          (IOMUX_PINCM8)
#define GPIO_LFXOUT_PIN                                            DL_GPIO_PIN_4
#define GPIO_LFXOUT_IOMUX                                         (IOMUX_PINCM9)
#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     40000000



/* Defines for STEPPER_MOTOR */
#define STEPPER_MOTOR_INST                                                 TIMA0
#define STEPPER_MOTOR_INST_IRQHandler                           TIMA0_IRQHandler
#define STEPPER_MOTOR_INST_INT_IRQN                             (TIMA0_INT_IRQn)
#define STEPPER_MOTOR_INST_CLK_FREQ                                     40000000
/* GPIO defines for channel 0 */
#define GPIO_STEPPER_MOTOR_C0_PORT                                         GPIOA
#define GPIO_STEPPER_MOTOR_C0_PIN                                 DL_GPIO_PIN_21
#define GPIO_STEPPER_MOTOR_C0_IOMUX                              (IOMUX_PINCM46)
#define GPIO_STEPPER_MOTOR_C0_IOMUX_FUNC             IOMUX_PINCM46_PF_TIMA0_CCP0
#define GPIO_STEPPER_MOTOR_C0_IDX                            DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_STEPPER_MOTOR_C1_PORT                                         GPIOA
#define GPIO_STEPPER_MOTOR_C1_PIN                                 DL_GPIO_PIN_22
#define GPIO_STEPPER_MOTOR_C1_IOMUX                              (IOMUX_PINCM47)
#define GPIO_STEPPER_MOTOR_C1_IOMUX_FUNC             IOMUX_PINCM47_PF_TIMA0_CCP1
#define GPIO_STEPPER_MOTOR_C1_IDX                            DL_TIMER_CC_1_INDEX

/* Defines for PWM_1 */
#define PWM_1_INST                                                         TIMA1
#define PWM_1_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_1_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_1_INST_CLK_FREQ                                             40000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_1_C0_PORT                                                 GPIOA
#define GPIO_PWM_1_C0_PIN                                         DL_GPIO_PIN_28
#define GPIO_PWM_1_C0_IOMUX                                       (IOMUX_PINCM3)
#define GPIO_PWM_1_C0_IOMUX_FUNC                      IOMUX_PINCM3_PF_TIMA1_CCP0
#define GPIO_PWM_1_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_1_C1_PORT                                                 GPIOA
#define GPIO_PWM_1_C1_PIN                                         DL_GPIO_PIN_24
#define GPIO_PWM_1_C1_IOMUX                                      (IOMUX_PINCM54)
#define GPIO_PWM_1_C1_IOMUX_FUNC                     IOMUX_PINCM54_PF_TIMA1_CCP1
#define GPIO_PWM_1_C1_IDX                                    DL_TIMER_CC_1_INDEX

/* Defines for PWM_2 */
#define PWM_2_INST                                                         TIMG0
#define PWM_2_INST_IRQHandler                                   TIMG0_IRQHandler
#define PWM_2_INST_INT_IRQN                                     (TIMG0_INT_IRQn)
#define PWM_2_INST_CLK_FREQ                                             40000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_2_C0_PORT                                                 GPIOB
#define GPIO_PWM_2_C0_PIN                                         DL_GPIO_PIN_10
#define GPIO_PWM_2_C0_IOMUX                                      (IOMUX_PINCM27)
#define GPIO_PWM_2_C0_IOMUX_FUNC                     IOMUX_PINCM27_PF_TIMG0_CCP0
#define GPIO_PWM_2_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_2_C1_PORT                                                 GPIOB
#define GPIO_PWM_2_C1_PIN                                         DL_GPIO_PIN_11
#define GPIO_PWM_2_C1_IOMUX                                      (IOMUX_PINCM28)
#define GPIO_PWM_2_C1_IOMUX_FUNC                     IOMUX_PINCM28_PF_TIMG0_CCP1
#define GPIO_PWM_2_C1_IDX                                    DL_TIMER_CC_1_INDEX




/* Defines for OLED */
#define OLED_INST                                                           I2C0
#define OLED_INST_IRQHandler                                     I2C0_IRQHandler
#define OLED_INST_INT_IRQN                                         I2C0_INT_IRQn
#define OLED_BUS_SPEED_HZ                                                 100000
#define GPIO_OLED_SDA_PORT                                                 GPIOA
#define GPIO_OLED_SDA_PIN                                         DL_GPIO_PIN_10
#define GPIO_OLED_IOMUX_SDA                                      (IOMUX_PINCM21)
#define GPIO_OLED_IOMUX_SDA_FUNC                       IOMUX_PINCM21_PF_I2C0_SDA
#define GPIO_OLED_SCL_PORT                                                 GPIOA
#define GPIO_OLED_SCL_PIN                                         DL_GPIO_PIN_11
#define GPIO_OLED_IOMUX_SCL                                      (IOMUX_PINCM22)
#define GPIO_OLED_IOMUX_SCL_FUNC                       IOMUX_PINCM22_PF_I2C0_SCL

/* Defines for I2C_1 */
#define I2C_1_INST                                                          I2C1
#define I2C_1_INST_IRQHandler                                    I2C1_IRQHandler
#define I2C_1_INST_INT_IRQN                                        I2C1_INT_IRQn
#define I2C_1_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_1_SDA_PORT                                                GPIOA
#define GPIO_I2C_1_SDA_PIN                                        DL_GPIO_PIN_16
#define GPIO_I2C_1_IOMUX_SDA                                     (IOMUX_PINCM38)
#define GPIO_I2C_1_IOMUX_SDA_FUNC                      IOMUX_PINCM38_PF_I2C1_SDA
#define GPIO_I2C_1_SCL_PORT                                                GPIOA
#define GPIO_I2C_1_SCL_PIN                                        DL_GPIO_PIN_15
#define GPIO_I2C_1_IOMUX_SCL                                     (IOMUX_PINCM37)
#define GPIO_I2C_1_IOMUX_SCL_FUNC                      IOMUX_PINCM37_PF_I2C1_SCL


/* Defines for K230 */
#define K230_INST                                                          UART0
#define K230_INST_FREQUENCY                                             40000000
#define K230_INST_IRQHandler                                    UART0_IRQHandler
#define K230_INST_INT_IRQN                                        UART0_INT_IRQn
#define GPIO_K230_RX_PORT                                                  GPIOA
#define GPIO_K230_TX_PORT                                                  GPIOA
#define GPIO_K230_RX_PIN                                           DL_GPIO_PIN_1
#define GPIO_K230_TX_PIN                                           DL_GPIO_PIN_0
#define GPIO_K230_IOMUX_RX                                        (IOMUX_PINCM2)
#define GPIO_K230_IOMUX_TX                                        (IOMUX_PINCM1)
#define GPIO_K230_IOMUX_RX_FUNC                         IOMUX_PINCM2_PF_UART0_RX
#define GPIO_K230_IOMUX_TX_FUNC                         IOMUX_PINCM1_PF_UART0_TX
#define K230_BAUD_RATE                                                    (9600)
#define K230_IBRD_40_MHZ_9600_BAUD                                         (260)
#define K230_FBRD_40_MHZ_9600_BAUD                                          (27)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           40000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_5
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_4
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM18)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM17)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM18_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM17_PF_UART1_TX
#define UART_1_BAUD_RATE                                                  (9600)
#define UART_1_IBRD_40_MHZ_9600_BAUD                                       (260)
#define UART_1_FBRD_40_MHZ_9600_BAUD                                        (27)
/* Defines for UART_2 */
#define UART_2_INST                                                        UART2
#define UART_2_INST_FREQUENCY                                           40000000
#define UART_2_INST_IRQHandler                                  UART2_IRQHandler
#define UART_2_INST_INT_IRQN                                      UART2_INT_IRQn
#define GPIO_UART_2_RX_PORT                                                GPIOB
#define GPIO_UART_2_TX_PORT                                                GPIOA
#define GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_18
#define GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_23
#define GPIO_UART_2_IOMUX_RX                                     (IOMUX_PINCM44)
#define GPIO_UART_2_IOMUX_TX                                     (IOMUX_PINCM53)
#define GPIO_UART_2_IOMUX_RX_FUNC                      IOMUX_PINCM44_PF_UART2_RX
#define GPIO_UART_2_IOMUX_TX_FUNC                      IOMUX_PINCM53_PF_UART2_TX
#define UART_2_BAUD_RATE                                                  (9600)
#define UART_2_IBRD_40_MHZ_9600_BAUD                                       (260)
#define UART_2_FBRD_40_MHZ_9600_BAUD                                        (27)
/* Defines for UART_3 */
#define UART_3_INST                                                        UART3
#define UART_3_INST_FREQUENCY                                           40000000
#define UART_3_INST_IRQHandler                                  UART3_IRQHandler
#define UART_3_INST_INT_IRQN                                      UART3_INT_IRQn
#define GPIO_UART_3_RX_PORT                                                GPIOA
#define GPIO_UART_3_TX_PORT                                                GPIOB
#define GPIO_UART_3_RX_PIN                                        DL_GPIO_PIN_25
#define GPIO_UART_3_TX_PIN                                        DL_GPIO_PIN_12
#define GPIO_UART_3_IOMUX_RX                                     (IOMUX_PINCM55)
#define GPIO_UART_3_IOMUX_TX                                     (IOMUX_PINCM29)
#define GPIO_UART_3_IOMUX_RX_FUNC                      IOMUX_PINCM55_PF_UART3_RX
#define GPIO_UART_3_IOMUX_TX_FUNC                      IOMUX_PINCM29_PF_UART3_TX
#define UART_3_BAUD_RATE                                                  (9600)
#define UART_3_IBRD_40_MHZ_9600_BAUD                                       (260)
#define UART_3_FBRD_40_MHZ_9600_BAUD                                        (27)




/* Defines for SPI_0 */
#define SPI_0_INST                                                         SPI0
#define SPI_0_INST_IRQHandler                                   SPI0_IRQHandler
#define SPI_0_INST_INT_IRQN                                       SPI0_INT_IRQn
#define GPIO_SPI_0_PICO_PORT                                              GPIOA
#define GPIO_SPI_0_PICO_PIN                                      DL_GPIO_PIN_14
#define GPIO_SPI_0_IOMUX_PICO                                   (IOMUX_PINCM36)
#define GPIO_SPI_0_IOMUX_PICO_FUNC                   IOMUX_PINCM36_PF_SPI0_PICO
#define GPIO_SPI_0_POCI_PORT                                              GPIOA
#define GPIO_SPI_0_POCI_PIN                                      DL_GPIO_PIN_13
#define GPIO_SPI_0_IOMUX_POCI                                   (IOMUX_PINCM35)
#define GPIO_SPI_0_IOMUX_POCI_FUNC                   IOMUX_PINCM35_PF_SPI0_POCI
/* GPIO configuration for SPI_0 */
#define GPIO_SPI_0_SCLK_PORT                                              GPIOA
#define GPIO_SPI_0_SCLK_PIN                                      DL_GPIO_PIN_12
#define GPIO_SPI_0_IOMUX_SCLK                                   (IOMUX_PINCM34)
#define GPIO_SPI_0_IOMUX_SCLK_FUNC                   IOMUX_PINCM34_PF_SPI0_SCLK
#define GPIO_SPI_0_CS0_PORT                                               GPIOA
#define GPIO_SPI_0_CS0_PIN                                        DL_GPIO_PIN_2
#define GPIO_SPI_0_IOMUX_CS0                                     (IOMUX_PINCM7)
#define GPIO_SPI_0_IOMUX_CS0_FUNC                      IOMUX_PINCM7_PF_SPI0_CS0
/* Defines for SPI_1 */
#define SPI_1_INST                                                         SPI1
#define SPI_1_INST_IRQHandler                                   SPI1_IRQHandler
#define SPI_1_INST_INT_IRQN                                       SPI1_INT_IRQn
#define GPIO_SPI_1_PICO_PORT                                              GPIOB
#define GPIO_SPI_1_PICO_PIN                                      DL_GPIO_PIN_15
#define GPIO_SPI_1_IOMUX_PICO                                   (IOMUX_PINCM32)
#define GPIO_SPI_1_IOMUX_PICO_FUNC                   IOMUX_PINCM32_PF_SPI1_PICO
#define GPIO_SPI_1_POCI_PORT                                              GPIOB
#define GPIO_SPI_1_POCI_PIN                                      DL_GPIO_PIN_14
#define GPIO_SPI_1_IOMUX_POCI                                   (IOMUX_PINCM31)
#define GPIO_SPI_1_IOMUX_POCI_FUNC                   IOMUX_PINCM31_PF_SPI1_POCI
/* GPIO configuration for SPI_1 */
#define GPIO_SPI_1_SCLK_PORT                                              GPIOB
#define GPIO_SPI_1_SCLK_PIN                                      DL_GPIO_PIN_16
#define GPIO_SPI_1_IOMUX_SCLK                                   (IOMUX_PINCM33)
#define GPIO_SPI_1_IOMUX_SCLK_FUNC                   IOMUX_PINCM33_PF_SPI1_SCLK
#define GPIO_SPI_1_CS0_PORT                                               GPIOB
#define GPIO_SPI_1_CS0_PIN                                       DL_GPIO_PIN_20
#define GPIO_SPI_1_IOMUX_CS0                                    (IOMUX_PINCM48)
#define GPIO_SPI_1_IOMUX_CS0_FUNC                     IOMUX_PINCM48_PF_SPI1_CS0



/* Defines for ADC12_JOYSTICK */
#define ADC12_JOYSTICK_INST                                                 ADC0
#define ADC12_JOYSTICK_INST_IRQHandler                           ADC0_IRQHandler
#define ADC12_JOYSTICK_INST_INT_IRQN                             (ADC0_INT_IRQn)
#define ADC12_JOYSTICK_ADCMEM_X                               DL_ADC12_MEM_IDX_0
#define ADC12_JOYSTICK_ADCMEM_X_REF              DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_JOYSTICK_ADCMEM_X_REF_VOLTAGE_V                                     3.3
#define ADC12_JOYSTICK_ADCMEM_Y                               DL_ADC12_MEM_IDX_1
#define ADC12_JOYSTICK_ADCMEM_Y_REF              DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_JOYSTICK_ADCMEM_Y_REF_VOLTAGE_V                                     3.3
#define GPIO_ADC12_JOYSTICK_C0_PORT                                        GPIOA
#define GPIO_ADC12_JOYSTICK_C0_PIN                                DL_GPIO_PIN_27
#define GPIO_ADC12_JOYSTICK_C1_PORT                                        GPIOA
#define GPIO_ADC12_JOYSTICK_C1_PIN                                DL_GPIO_PIN_26

/* Defines for ADC12_BATTERY */
#define ADC12_BATTERY_INST                                                  ADC1
#define ADC12_BATTERY_INST_IRQHandler                            ADC1_IRQHandler
#define ADC12_BATTERY_INST_INT_IRQN                              (ADC1_INT_IRQn)
#define ADC12_BATTERY_ADCMEM_0                                DL_ADC12_MEM_IDX_0
#define ADC12_BATTERY_ADCMEM_0_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_BATTERY_ADCMEM_0_REF_VOLTAGE_V                                     3.3
#define GPIO_ADC12_BATTERY_C4_PORT                                         GPIOB
#define GPIO_ADC12_BATTERY_C4_PIN                                 DL_GPIO_PIN_17



/* Defines for DMA_CH_ADC_JOYSTICK */
#define DMA_CH_ADC_JOYSTICK_CHAN_ID                                          (0)
#define ADC12_JOYSTICK_INST_DMA_TRIGGER               (DMA_ADC0_EVT_GEN_BD_TRIG)


/* Port definition for Pin Group GPIO_JOYSTICK */
#define GPIO_JOYSTICK_PORT                                               (GPIOB)

/* Defines for PIN_SW: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_JOYSTICK_PIN_SW_PIN                                (DL_GPIO_PIN_13)
#define GPIO_JOYSTICK_PIN_SW_IOMUX                               (IOMUX_PINCM30)
/* Port definition for Pin Group GPIO_BEEP */
#define GPIO_BEEP_PORT                                                   (GPIOB)

/* Defines for PIN_EN: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_BEEP_PIN_EN_PIN                                    (DL_GPIO_PIN_24)
#define GPIO_BEEP_PIN_EN_IOMUX                                   (IOMUX_PINCM52)
/* Port definition for Pin Group GPIO_STEPPER_MOTOR */
#define GPIO_STEPPER_MOTOR_PORT                                          (GPIOB)

/* Defines for PIN_EN_L: GPIOB.19 with pinCMx 45 on package pin 16 */
#define GPIO_STEPPER_MOTOR_PIN_EN_L_PIN                         (DL_GPIO_PIN_19)
#define GPIO_STEPPER_MOTOR_PIN_EN_L_IOMUX                        (IOMUX_PINCM45)
/* Defines for PIN_EN_H: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_STEPPER_MOTOR_PIN_EN_H_PIN                         (DL_GPIO_PIN_21)
#define GPIO_STEPPER_MOTOR_PIN_EN_H_IOMUX                        (IOMUX_PINCM49)
/* Defines for PIN_DIR_H: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_STEPPER_MOTOR_PIN_DIR_H_PIN                        (DL_GPIO_PIN_22)
#define GPIO_STEPPER_MOTOR_PIN_DIR_H_IOMUX                       (IOMUX_PINCM50)
/* Defines for PIN_DIR_L: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GPIO_STEPPER_MOTOR_PIN_DIR_L_PIN                        (DL_GPIO_PIN_23)
#define GPIO_STEPPER_MOTOR_PIN_DIR_L_IOMUX                       (IOMUX_PINCM51)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);
void SYSCFG_DL_STEPPER_MOTOR_init(void);
void SYSCFG_DL_PWM_1_init(void);
void SYSCFG_DL_PWM_2_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_I2C_1_init(void);
void SYSCFG_DL_K230_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_2_init(void);
void SYSCFG_DL_UART_3_init(void);
void SYSCFG_DL_SPI_0_init(void);
void SYSCFG_DL_SPI_1_init(void);
void SYSCFG_DL_ADC12_JOYSTICK_init(void);
void SYSCFG_DL_ADC12_BATTERY_init(void);
void SYSCFG_DL_DMA_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
