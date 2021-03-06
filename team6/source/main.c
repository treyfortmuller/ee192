/*
 * EE192 Team 6 Autonomous Racecar
 * Trey Fortmuller, Charlene Shong, Joey Kroeger
 */

/* System includes. */
#include <stdio.h>

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_ftm.h"
#include "fsl_adc16.h"
#include "fsl_pit.h"  /* periodic interrupt timer */

/* Additional inclusions. */
#include "clock_config.h"
#include "pin_mux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Initialized Pins (in use) */
/* PTC1 - FTM0 - Servo PWM
 * PTB19 - FTM2 - Motor PWM
 * PTB2, ADC0_SE12 - ADC for Line Scan Camera 1
 * PTB3, ADC0_SE13 - ADC for Line Scan Camera 2
 * PTB10, ADC1_SE14 - ADC for Optical Encoder
 * PTB9 - GPIOB - SI for LSC1
 * PTB23 - GPIOE - CLK for LCS1
 *
 * Extras Pins (in case; need code)
 * PTD3 - FTM3 - Braking PWM?
 * PTA1 - GPIOA - SI for LSC2
 * PTD2 - GPIOD - CLK for LCS2
 * PTC16 - GPIOC - Encoder Timing?
 */

/* The Flex Timer Module (FTM) instance/channel used for board. See pin view or open declaration for more info */
#define BOARD_FTM_BASEADDR_SERVO FTM0
#define BOARD_FTM_BASEADDR_MOTOR FTM2
#define BOARD_FTM_CHANNEL_SERVO kFTM_Chnl_0 // pin PTC1, servo PWM
#define BOARD_FTM_CHANNEL_MOTOR kFTM_Chnl_1 // pin PTB19, motor PWM

/* Interrupt to enable and flag to read; depends on the FTM channel used */
#define FTM_CHANNEL_INTERRUPT_ENABLE_SERVO kFTM_Chnl0InterruptEnable // servo
#define FTM_CHANNEL_FLAG_SERVO kFTM_Chnl0Flag

#define FTM_CHANNEL_INTERRUPT_ENABLE_MOTOR kFTM_Chnl1InterruptEnable // motor
#define FTM_CHANNEL_FLAG_MOTOR kFTM_Chnl1Flag

/* Get source clock for FTM driver */
/* For slow PWM must pick a slow clock!!!!
 * kCLOCK_PlatClk (120 Mhz): freq > 914 hz
 * kCLOCK_BusClk (60 Mhz): freq > 457 hz
 * kCLOCK_FlexBusClk (40 Mhz): freq > 305 hz
 * kCLOCK_FlashClk (24 Mhz): freq > 183 hz
 * see fsl_clock.c for other clocks
 *
 * This example uses PWM @ 20khz so you can use any clock
 *
 */
/* Get source clock for FTM driver, we're looking for ~400Hz PWM for the motor */
#define FTM_SOURCE_CLOCK_SERVO CLOCK_GetFreq(kCLOCK_BusClk) // servo (slow PWM)
#define FTM_SOURCE_CLOCK_MOTOR CLOCK_GetFreq(kCLOCK_BusClk) // motor (high PWM)

// High Voltage(3.3V)=True for PWM
#define PWM_LEVEL kFTM_HighTrue

/* Get source clock for PIT driver */
#define PIT_IRQ_ID PIT0_IRQn
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
volatile bool pitIsrFlag = false;

/* ADC definitions s*/
// for line scan camera
#define DEMO_ADC16_BASEADDR_CAM ADC0
#define DEMO_ADC16_CHANNEL_GROUP_CAM 0U
#define DEMO_ADC16_USER_CHANNEL_CAM1 12U /* PTB2, ADC0_SE12 */

#define DEMO_ADC16_IRQn_CAM ADC0_IRQn
#define DEMO_ADC16_IRQ_HANDLER_FUNC_CAM ADC0_IRQHandler

#define VREF_BRD 3.300
#define SE_12BIT 4096.0

/* GPIO definitions */
#define GPIO_CHANNEL GPIOB
#define GPIO_PIN_SI 9U // pin PTB9, SI
#define GPIO_PIN_CLK 23U // pin PTB23, CLK

/* Line Scan definitions */
#define SI_HIGH    GPIO_PinWrite(GPIO_CHANNEL, GPIO_PIN_SI, 1) //SI = 1
#define SI_LOW     GPIO_PinWrite(GPIO_CHANNEL, GPIO_PIN_SI, 0) //SI = 0
#define CLK_HIGH   GPIO_PinWrite(GPIO_CHANNEL, GPIO_PIN_CLK, 1) //CLK = 1
#define CLK_LOW    GPIO_PinWrite(GPIO_CHANNEL, GPIO_PIN_CLK, 0) //CLK = 0
#define frame_width 128 // frame width of the camera

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */
void delay(uint32_t t);
void update_duty_cycle_servo(uint8_t updated_duty_cycle);
void update_duty_cycle_motor(uint8_t updated_duty_cycle);
void init_pwm_servo(uint32_t freq_hz, uint8_t init_duty_cycle);
void init_pwm_motor(uint32_t freq_hz, uint8_t init_duty_cycle);
float getVelocity(int transition_count);

/* Initialize ADC16 */
static void init_adc_cam(void);
static void init_board(void);
void read_ADC_cam(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool ftmIsrFlag = false;
volatile uint8_t duty_cycle = 1U;
volatile uint8_t duty_cycle_motor = 1U;
volatile uint8_t motor_min = 1U; // the minimum pwm percentage for the motor
volatile uint8_t motor_max = 1U; // the maximum pwm percentage for the motor
volatile uint32_t systime = 0; // systime updated very 100 us = 4 days ==> NEED OVERFLOW protection

// ADC variables
volatile bool g_Adc16ConversionDoneFlag = false;
volatile uint32_t g_Adc16ConversionValue_cam = 0;
adc16_channel_config_t g_adc16ChannelConfigStruct_cam;

// Line scan variables
int picture[128];
char track[frame_width]; // the telemetry output representation of the detected track
float lat_err = 0;
float old_lat_err = 0;
float lat_vel = 0;
int position = 64; //this is the fake result of the argmax over the camera frame

// PD controller variables
const float dt = 0.010; //PD timestep in SECONDS (50 Hz)
const float kp = 0.25; //Proportional Gain
const float kd = 0.025; //Derivative Gain

/*******************************************************************************
 * Code
 ******************************************************************************/
void init_board()
{
	/* Board pin, clock, debug console init */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
}

/*Initialize the Flexible Timer Module to Produce PWM with init_duty_cycle at freq_hz*/
void init_pwm_servo(uint32_t freq_hz, uint8_t init_duty_cycle)
{
	duty_cycle = init_duty_cycle;
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;

    /* Configure ftm params with for pwm freq- freq_hz, duty cycle- init_duty_cycle */
    ftmParam.chnlNumber = BOARD_FTM_CHANNEL_SERVO;
    ftmParam.level = PWM_LEVEL;
    ftmParam.dutyCyclePercent = init_duty_cycle;
    ftmParam.firstEdgeDelayPercent = 0U;

    FTM_GetDefaultConfig(&ftmInfo);
    ftmInfo.prescale = kFTM_Prescale_Divide_128;
    //set the prescaler to 128 to get very low frequencies of PWM reliably

    /* Initialize FTM module */
    FTM_Init(BOARD_FTM_BASEADDR_SERVO, &ftmInfo);

    FTM_SetupPwm(BOARD_FTM_BASEADDR_SERVO, &ftmParam, 1U, kFTM_CenterAlignedPwm, freq_hz, FTM_SOURCE_CLOCK_SERVO);

    FTM_StartTimer(BOARD_FTM_BASEADDR_SERVO, kFTM_SystemClock);
}

void init_pwm_motor(uint32_t freq_hz, uint8_t init_duty_cycle)
{
	duty_cycle = init_duty_cycle;
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;

    /* Configure ftm params with for pwm freq- freq_hz, duty cycle- init_duty_cycle */
    ftmParam.chnlNumber = BOARD_FTM_CHANNEL_MOTOR;
    ftmParam.level = PWM_LEVEL;
    ftmParam.dutyCyclePercent = init_duty_cycle;
    ftmParam.firstEdgeDelayPercent = 0U;

    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    FTM_Init(BOARD_FTM_BASEADDR_MOTOR, &ftmInfo);

    FTM_SetupPwm(BOARD_FTM_BASEADDR_MOTOR, &ftmParam, 1U, kFTM_CenterAlignedPwm, freq_hz, FTM_SOURCE_CLOCK_MOTOR);

    FTM_StartTimer(BOARD_FTM_BASEADDR_MOTOR, kFTM_SystemClock);
}

void init_gpio()
{
    // Configure an output pin with initial value 0
    gpio_pin_config_t gpio_config = {
                kGPIO_DigitalOutput, 0,
            };

    /* initialize GPIO Pin(s) */
    GPIO_PinInit(GPIO_CHANNEL, GPIO_PIN_SI, &gpio_config); // initialize GPIOB pin PTB9
    GPIO_PinInit(GPIO_CHANNEL, GPIO_PIN_CLK, &gpio_config); // initialize GPIOB pin PTB23
}

/*******************************************************************************
 * Other Code
 ******************************************************************************/

void delay (uint32_t t) {
	uint32_t end_time = t + systime;

	// block until the delay is over
	while (systime < end_time)
		{
		}
}

void update_duty_cycle_servo(uint8_t updated_duty_cycle)
{
	/* Disable channel output before updating the dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR_SERVO, BOARD_FTM_CHANNEL_SERVO, 0U);

	/* Update PWM duty cycle */
	FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR_SERVO, BOARD_FTM_CHANNEL_SERVO, kFTM_CenterAlignedPwm, updated_duty_cycle);

	/* Software trigger to update registers */
	FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR_SERVO, true);

	/* Start channel output with updated dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR_SERVO, BOARD_FTM_CHANNEL_SERVO, PWM_LEVEL);
}

void update_duty_cycle_motor(uint8_t updated_duty_cycle)
{
	/* Disable channel output before updating the dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR_MOTOR, BOARD_FTM_CHANNEL_MOTOR, 0U);

	/* Update PWM duty cycle */
	FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR_MOTOR, BOARD_FTM_CHANNEL_MOTOR, kFTM_CenterAlignedPwm, updated_duty_cycle);

	/* Software trigger to update registers */
	FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR_MOTOR, true);

	/* Start channel output with updated dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR_MOTOR, BOARD_FTM_CHANNEL_MOTOR, PWM_LEVEL);
}

static void init_adc_cam(void)
{
	EnableIRQ(DEMO_ADC16_IRQn_CAM);
    adc16_config_t adc16ConfigStruct;

    /* Configure the ADC16. All the default values follow, uncomment and adjust them if need be*/
    /*
     * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
     * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
     * adc16ConfigStruct.enableAsynchronousClock = true;
     * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
     * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
     * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
     * adc16ConfigStruct.enableHighSpeed = false;
     * adc16ConfigStruct.enableLowPower = false;
     * adc16ConfigStruct.enableContinuousConversion = false;
     */

    ADC16_GetDefaultConfig(&adc16ConfigStruct);
#if defined(BOARD_ADC_USE_ALT_VREF)
    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    ADC16_Init(DEMO_ADC16_BASEADDR_CAM, &adc16ConfigStruct);

    /* Make sure the software trigger is used. */
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASEADDR_CAM, false);
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASEADDR_CAM))
    {
        PRINTF("\r\nADC16_DoAutoCalibration() Done.");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    /* Prepare ADC channel setting */
    g_adc16ChannelConfigStruct_cam.channelNumber = DEMO_ADC16_USER_CHANNEL_CAM1;
    g_adc16ChannelConfigStruct_cam.enableInterruptOnConversionCompleted = true;

#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    g_adc16ChannelConfigStruct_cam.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
}

void read_ADC_cam(void){
	g_Adc16ConversionDoneFlag = false;
	ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR_CAM, DEMO_ADC16_CHANNEL_GROUP_CAM, &g_adc16ChannelConfigStruct_cam);

	//Block until the ADC Conversion is finished
	 while (!g_Adc16ConversionDoneFlag)
	 {
	 }
}

void DEMO_ADC16_IRQ_HANDLER_FUNC_CAM(void)
{
    g_Adc16ConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
    g_Adc16ConversionValue_cam = ADC16_GetChannelConversionValue(DEMO_ADC16_BASEADDR_CAM, DEMO_ADC16_CHANNEL_GROUP_CAM);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void init_pit()
{
   /* Structure of initialize PIT */
	pit_config_t pitConfig;

	/* start periodic interrupt timer- should be in its own file */
	PIT_GetDefaultConfig(&pitConfig);
	/* Initialize pit module */
	PIT_Init(PIT, &pitConfig);
	/* Set timer period for channel 0 */

	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(100U, PIT_SOURCE_CLOCK)); //100us timing
	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
	/* Enable at the NVIC */
	EnableIRQ(PIT_IRQ_ID);
	/* Start channel 0 */
	PIT_StartTimer(PIT, kPIT_Chnl_0);
}

void capture()
{
    int i;
    SI_HIGH;
    CLK_HIGH;
    SI_LOW;
    for (i = 0; i < 127; i++) { //loop for 128 pixels
    	CLK_LOW;
    	CLK_HIGH;
    	delay(1); //exposure delay
        read_ADC_cam();
        picture[i] = g_Adc16ConversionValue_cam; //store data in array
    }
    CLK_HIGH;
    CLK_LOW;
}

int argmax(int arr[], size_t size)
{
	/* enforce the contract */
	assert(arr && size);
	size_t i;
	int maxValue = arr[0];
	int temp = 0;
	for (i = 1; i < size; ++i) {
		if (arr[i] > maxValue) {
			maxValue = arr[i];
			temp = i;
		}
	}
	return temp;
}

int main(void)
{
	init_board();
	init_adc_cam();
	init_gpio();
	init_pit();
	init_pwm_motor(5000, 19); //start 1khz pwm at 18% duty cycle, for motor drive
	init_pwm_servo(100, 14); //start 500hz pwm at 75% duty cycle, for servo steer

	// init the motor min and max values for interpolating based on servo pwm percentages
	motor_min = 19;
	motor_max = 19;

	while (1) {
		capture();
	}
}

/*******************************************************************************
 * Interrupt functions
 ******************************************************************************/

void PIT0_IRQHandler(void) //clear interrupt flag
{
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
    pitIsrFlag = true;

    if (systime % 100 == 0) {
    	position = argmax(picture, 128);
    	lat_err = 64 - position;
    	// controller
    	if (dt > 0.0) {
    		lat_vel = (lat_err - old_lat_err)/dt;
    	} else {
    		lat_vel = 0.0;
    	}
    	old_lat_err = lat_err;
    	// update pwm
    	duty_cycle = (uint8_t) 14 - kp*lat_err - kd*lat_vel;
    	// pwm limits
    	if (duty_cycle < 7) {
    		duty_cycle = 7;
    	} else if (duty_cycle > 24) {
    		duty_cycle = 24;
    	}
    	update_duty_cycle_servo(duty_cycle);

    	// motor pwm command linearly interpolated from servo pwm
    	if (duty_cycle < 14) {
    		duty_cycle_motor = motor_max - (motor_max - motor_min)*((14-duty_cycle)/8); // 10 is the range of servo pwm
    	}
    	else {
    		duty_cycle_motor = motor_max - (motor_max - motor_min)*((duty_cycle-14)/10); // 10 is the range of the servo pwm
    	}
    	update_duty_cycle_motor(duty_cycle_motor); // update the duty cycle of the motor

//    	 the satisfied slow/fast motor pwm based on servo pwm percentage
//    		if (duty_cycle > 80) { // if we're turning right?
//    			update_duty_cycle_motor(motor_min); //
//    		}update the duty cycle of the motor to be slower
//    		else if (duty_cycle < 70) { // if we're turning left?
//    			update_duty_cycle_motor(motor_min); // update the duty cycle of the motor to be slower
//    		}
//    		else { // we're not turning
//    			update_duty_cycle_motor(motor_max); // update the duty cycle to be full speed
//    		}
    }

    systime++; /* hopefully atomic operation */
}
