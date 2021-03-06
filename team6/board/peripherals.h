/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_ftm.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_uart.h"
#include "fsl_adc16.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
/* Definitions for BOARD_InitPeripherals functional group */
/* Definition of peripheral ID */
#define FTM_1_PERIPHERAL FTM0
/* Definition of the clock source frequency */
#define FTM_1_CLOCK_SOURCE CLOCK_GetFreq(kCLOCK_BusClk)
/* FTM_1 interrupt vector ID (number). */
#define FTM_1_IRQN FTM0_IRQn
/* FTM_1 interrupt handler identifier. */
#define FTM_1_IRQHANDLER FTM0_IRQHandler
/* Alias for GPIOE peripheral */
#define GPIO_1_GPIO GPIOE
/* Definition of peripheral ID */
#define UART_1_PERIPHERAL UART0
/* Definition of the clock source frequency */
#define UART_1_CLOCK_SOURCE CLOCK_GetFreq(UART0_CLK_SRC)
/* Alias for GPIOB peripheral */
#define GPIO_2_GPIO GPIOB
/* Alias for ADC0 peripheral */
#define ADC16_1_PERIPHERAL ADC0
/* ADC16_1 interrupt vector ID (number). */
#define ADC16_1_IRQN ADC0_IRQn
/* ADC16_1 interrupt handler identifier. */
#define ADC16_1_IRQHANDLER ADC0_IRQHandler
/* Alias for GPIOC peripheral */
#define GPIO_3_GPIO GPIOC
/* Definition of peripheral ID */
#define FTM_2_PERIPHERAL FTM2
/* Definition of the clock source frequency */
#define FTM_2_CLOCK_SOURCE CLOCK_GetFreq(kCLOCK_BusClk)
/* FTM_2 interrupt vector ID (number). */
#define FTM_2_IRQN FTM2_IRQn
/* FTM_2 interrupt handler identifier. */
#define FTM_2_IRQHANDLER FTM2_IRQHandler
/* Definition of peripheral ID */
#define FTM_3_PERIPHERAL FTM3
/* Definition of the clock source frequency */
#define FTM_3_CLOCK_SOURCE CLOCK_GetFreq(kCLOCK_BusClk)
/* FTM_3 interrupt vector ID (number). */
#define FTM_3_IRQN FTM3_IRQn
/* FTM_3 interrupt handler identifier. */
#define FTM_3_IRQHANDLER FTM3_IRQHandler
/* Alias for GPIOD peripheral */
#define GPIO_4_GPIO GPIOD
/* Alias for ADC1 peripheral */
#define ADC16_2_PERIPHERAL ADC1
/* ADC16_2 interrupt vector ID (number). */
#define ADC16_2_IRQN ADC1_IRQn
/* ADC16_2 interrupt handler identifier. */
#define ADC16_2_IRQHANDLER ADC1_IRQHandler
/* Alias for GPIOA peripheral */
#define GPIO_5_GPIO GPIOA
/* Definition of peripheral ID */
#define UART_2_PERIPHERAL UART4
/* Definition of the clock source frequency */
#define UART_2_CLOCK_SOURCE CLOCK_GetFreq(UART4_CLK_SRC)

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
extern const ftm_config_t FTM_1_config;
extern const uart_config_t UART_1_config;
extern const adc16_config_t ADC16_1_config;
extern const adc16_channel_mux_mode_t ADC16_1_muxMode;
extern const adc16_hardware_average_mode_t ADC16_1_hardwareAverageMode;
extern const ftm_config_t FTM_2_config;
extern const ftm_config_t FTM_3_config;
extern const adc16_config_t ADC16_2_config;
extern const adc16_channel_mux_mode_t ADC16_2_muxMode;
extern const adc16_hardware_average_mode_t ADC16_2_hardwareAverageMode;
extern const uart_config_t UART_2_config;

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void);

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void);

#if defined(__cplusplus)
}
#endif

#endif /* _PERIPHERALS_H_ */
