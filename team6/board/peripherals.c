/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v1.0
processor: MK64FN1M0xxx12
package_id: MK64FN1M0VLL12
mcu_data: ksdk2_0
processor_version: 3.0.1
board: FRDM-K64F
functionalGroups:
- name: BOARD_InitPeripherals
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * FTM_1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM_1'
- type: 'ftm'
- mode: 'EdgeAligned'
- type_id: 'ftm_5e037045c21cf6f361184c371dbbbab2'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'FTM0'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - clockSource: 'kFTM_SystemClock'
      - prescale: 'kFTM_Prescale_Divide_1'
      - timerFrequency: '10000'
      - bdmMode: 'kFTM_BdmMode_0'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: ''
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM0_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
    - quick_selection: 'QuickSelectionDefault'
  - ftm_edge_aligned_mode:
    - ftm_edge_aligned_channels_config: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t FTM_1_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .bdmMode = kFTM_BdmMode_0,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = 0,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};

void FTM_1_init(void) {
  FTM_Init(FTM_1_PERIPHERAL, &FTM_1_config);
  FTM_SetTimerPeriod(FTM_1_PERIPHERAL, ((FTM_1_CLOCK_SOURCE/ (1U << (FTM_1_PERIPHERAL->SC & FTM_SC_PS_MASK))) / 10000) + 1);
  FTM_StartTimer(FTM_1_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * GPIO_1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIO_1'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOE'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTE_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - gpio_config: []
    - quick_selection: 'QS_GPIO_1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void GPIO_1_init(void) {
  /* Make sure, the clock gate for port E is enabled (e. g. in pin_mux.c) */
}

/***********************************************************************************************************************
 * UART_1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART_1'
- type: 'uart'
- mode: 'polling'
- type_id: 'uart_cd31a12aa8c79051fda42cc851a27c37'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'UART0'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - baudRate_Bps: '115200'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '0'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
    - quick_selection: 'QuickSelection1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART_1_config = {
  .baudRate_Bps = 115200,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0,
  .rxFifoWatermark = 0,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

void UART_1_init(void) {
  UART_Init(UART_1_PERIPHERAL, &UART_1_config, UART_1_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * GPIO_2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIO_2'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOB'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTB_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - gpio_config: []
    - quick_selection: 'QS_GPIO_1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void GPIO_2_init(void) {
  /* Make sure, the clock gate for port B is enabled (e. g. in pin_mux.c) */
}

/***********************************************************************************************************************
 * ADC16_1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ADC16_1'
- type: 'adc16'
- mode: 'ADC'
- type_id: 'adc16_7d827be2dc433dc756d94a7ce88cbcc5'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'ADC0'
- config_sets:
  - fsl_adc16:
    - adc16_config:
      - referenceVoltageSource: 'kADC16_ReferenceVoltageSourceVref'
      - clockSource: 'kADC16_ClockSourceAsynchronousClock'
      - enableAsynchronousClock: 'true'
      - clockDivider: 'kADC16_ClockDivider8'
      - resolution: 'kADC16_ResolutionSE12Bit'
      - longSampleMode: 'kADC16_LongSampleDisabled'
      - enableHighSpeed: 'false'
      - enableLowPower: 'false'
      - enableContinuousConversion: 'false'
    - adc16_channel_mux_mode: 'kADC16_ChannelMuxA'
    - adc16_hardware_compare_config:
      - hardwareCompareModeEnable: 'false'
    - doAutoCalibration: 'false'
    - trigger: 'false'
    - hardwareAverageConfiguration: 'kADC16_HardwareAverageDisabled'
    - enable_irq: 'false'
    - adc_interrupt:
      - IRQn: 'ADC0_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - adc16_channels_config: []
    - quick_selection: 'QS_ADC16_1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const adc16_config_t ADC16_1_config = {
  .referenceVoltageSource = kADC16_ReferenceVoltageSourceVref,
  .clockSource = kADC16_ClockSourceAsynchronousClock,
  .enableAsynchronousClock = true,
  .clockDivider = kADC16_ClockDivider8,
  .resolution = kADC16_ResolutionSE12Bit,
  .longSampleMode = kADC16_LongSampleDisabled,
  .enableHighSpeed = false,
  .enableLowPower = false,
  .enableContinuousConversion = false
};
const adc16_channel_mux_mode_t ADC16_1_muxMode = kADC16_ChannelMuxA;
const adc16_hardware_average_mode_t ADC16_1_hardwareAverageMode = kADC16_HardwareAverageDisabled;

void ADC16_1_init(void) {
  /* Initialize ADC16 converter */
  ADC16_Init(ADC16_1_PERIPHERAL, &ADC16_1_config);
  /* Make sure, that software trigger is used */
  ADC16_EnableHardwareTrigger(ADC16_1_PERIPHERAL, false);
  /* Configure hardware average mode */
  ADC16_SetHardwareAverage(ADC16_1_PERIPHERAL, ADC16_1_hardwareAverageMode);
  /* Configure channel multiplexing mode */
  ADC16_SetChannelMuxMode(ADC16_1_PERIPHERAL, ADC16_1_muxMode);
}

/***********************************************************************************************************************
 * GPIO_3 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIO_3'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOC'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTC_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - gpio_config: []
    - quick_selection: 'QS_GPIO_1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void GPIO_3_init(void) {
  /* Make sure, the clock gate for port C is enabled (e. g. in pin_mux.c) */
}

/***********************************************************************************************************************
 * FTM_2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM_2'
- type: 'ftm'
- mode: 'EdgeAligned'
- type_id: 'ftm_5e037045c21cf6f361184c371dbbbab2'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'FTM2'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - clockSource: 'kFTM_SystemClock'
      - prescale: 'kFTM_Prescale_Divide_1'
      - timerFrequency: '10000'
      - bdmMode: 'kFTM_BdmMode_0'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: ''
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM2_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
    - quick_selection: 'QuickSelectionDefault'
  - ftm_edge_aligned_mode:
    - ftm_edge_aligned_channels_config: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t FTM_2_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .bdmMode = kFTM_BdmMode_0,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = 0,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};

void FTM_2_init(void) {
  FTM_Init(FTM_2_PERIPHERAL, &FTM_2_config);
  FTM_SetTimerPeriod(FTM_2_PERIPHERAL, ((FTM_2_CLOCK_SOURCE/ (1U << (FTM_2_PERIPHERAL->SC & FTM_SC_PS_MASK))) / 10000) + 1);
  FTM_StartTimer(FTM_2_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * FTM_3 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM_3'
- type: 'ftm'
- mode: 'EdgeAligned'
- type_id: 'ftm_5e037045c21cf6f361184c371dbbbab2'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'FTM3'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - clockSource: 'kFTM_SystemClock'
      - prescale: 'kFTM_Prescale_Divide_1'
      - timerFrequency: '10000'
      - bdmMode: 'kFTM_BdmMode_0'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: ''
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM3_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
    - quick_selection: 'QuickSelectionDefault'
  - ftm_edge_aligned_mode:
    - ftm_edge_aligned_channels_config: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t FTM_3_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .bdmMode = kFTM_BdmMode_0,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = 0,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};

void FTM_3_init(void) {
  FTM_Init(FTM_3_PERIPHERAL, &FTM_3_config);
  FTM_SetTimerPeriod(FTM_3_PERIPHERAL, ((FTM_3_CLOCK_SOURCE/ (1U << (FTM_3_PERIPHERAL->SC & FTM_SC_PS_MASK))) / 10000) + 1);
  FTM_StartTimer(FTM_3_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * GPIO_4 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIO_4'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOD'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTD_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - gpio_config: []
    - quick_selection: 'QS_GPIO_1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void GPIO_4_init(void) {
  /* Make sure, the clock gate for port D is enabled (e. g. in pin_mux.c) */
}

/***********************************************************************************************************************
 * ADC16_2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ADC16_2'
- type: 'adc16'
- mode: 'ADC'
- type_id: 'adc16_7d827be2dc433dc756d94a7ce88cbcc5'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'ADC1'
- config_sets:
  - fsl_adc16:
    - adc16_config:
      - referenceVoltageSource: 'kADC16_ReferenceVoltageSourceVref'
      - clockSource: 'kADC16_ClockSourceAsynchronousClock'
      - enableAsynchronousClock: 'true'
      - clockDivider: 'kADC16_ClockDivider8'
      - resolution: 'kADC16_ResolutionSE12Bit'
      - longSampleMode: 'kADC16_LongSampleDisabled'
      - enableHighSpeed: 'false'
      - enableLowPower: 'false'
      - enableContinuousConversion: 'false'
    - adc16_channel_mux_mode: 'kADC16_ChannelMuxA'
    - adc16_hardware_compare_config:
      - hardwareCompareModeEnable: 'false'
    - doAutoCalibration: 'false'
    - trigger: 'false'
    - hardwareAverageConfiguration: 'kADC16_HardwareAverageDisabled'
    - enable_irq: 'false'
    - adc_interrupt:
      - IRQn: 'ADC1_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - adc16_channels_config: []
    - quick_selection: 'QS_ADC16_1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const adc16_config_t ADC16_2_config = {
  .referenceVoltageSource = kADC16_ReferenceVoltageSourceVref,
  .clockSource = kADC16_ClockSourceAsynchronousClock,
  .enableAsynchronousClock = true,
  .clockDivider = kADC16_ClockDivider8,
  .resolution = kADC16_ResolutionSE12Bit,
  .longSampleMode = kADC16_LongSampleDisabled,
  .enableHighSpeed = false,
  .enableLowPower = false,
  .enableContinuousConversion = false
};
const adc16_channel_mux_mode_t ADC16_2_muxMode = kADC16_ChannelMuxA;
const adc16_hardware_average_mode_t ADC16_2_hardwareAverageMode = kADC16_HardwareAverageDisabled;

void ADC16_2_init(void) {
  /* Initialize ADC16 converter */
  ADC16_Init(ADC16_2_PERIPHERAL, &ADC16_2_config);
  /* Make sure, that software trigger is used */
  ADC16_EnableHardwareTrigger(ADC16_2_PERIPHERAL, false);
  /* Configure hardware average mode */
  ADC16_SetHardwareAverage(ADC16_2_PERIPHERAL, ADC16_2_hardwareAverageMode);
  /* Configure channel multiplexing mode */
  ADC16_SetChannelMuxMode(ADC16_2_PERIPHERAL, ADC16_2_muxMode);
}

/***********************************************************************************************************************
 * GPIO_5 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIO_5'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOA'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTA_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - gpio_config: []
    - quick_selection: 'QS_GPIO_1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void GPIO_5_init(void) {
  /* Make sure, the clock gate for port A is enabled (e. g. in pin_mux.c) */
}

/***********************************************************************************************************************
 * UART_2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART_2'
- type: 'uart'
- mode: 'polling'
- type_id: 'uart_cd31a12aa8c79051fda42cc851a27c37'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'UART4'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - baudRate_Bps: '115200'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '0'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
    - quick_selection: 'QuickSelection1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART_2_config = {
  .baudRate_Bps = 115200,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0,
  .rxFifoWatermark = 0,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

void UART_2_init(void) {
  UART_Init(UART_2_PERIPHERAL, &UART_2_config, UART_2_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  FTM_1_init();
  GPIO_1_init();
  UART_1_init();
  GPIO_2_init();
  ADC16_1_init();
  GPIO_3_init();
  FTM_2_init();
  FTM_3_init();
  GPIO_4_init();
  ADC16_2_init();
  GPIO_5_init();
  UART_2_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
