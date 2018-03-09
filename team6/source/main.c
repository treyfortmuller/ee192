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
/* The Flex Timer Module (FTM) instance/channel used for board. Indicates pin PTC1, see pin view for more info */
#define BOARD_FTM_BASEADDR FTM0
#define BOARD_FTM_CHANNEL kFTM_Chnl_0

/* Interrupt number and interrupt handler for the FTM instance used */
#define FTM_INTERRUPT_NUMBER FTM0_IRQn
#define FTM_0_HANDLER FTM0_IRQHandler

/* Interrupt to enable and flag to read; depends on the FTM channel used */
#define FTM_CHANNEL_INTERRUPT_ENABLE kFTM_Chnl0InterruptEnable
#define FTM_CHANNEL_FLAG kFTM_Chnl0Flag

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
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_FlexBusClk)

/* Get source clock for PIT driver */
#define PIT_IRQ_ID PIT0_IRQn
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
volatile bool pitIsrFlag = false;

/* ADC definitions */
// High Voltage(3.3V)=True for PWM
#define PWM_LEVEL kFTM_HighTrue

#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 12U /* PTB2, ADC0_SE12 */

#define DEMO_ADC16_IRQn ADC0_IRQn
#define DEMO_ADC16_IRQ_HANDLER_FUNC ADC0_IRQHandler

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

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */
void delay(uint32_t t);
void update_duty_cycle(uint8_t updated_duty_cycle);
void init_pwm(uint32_t freq_hz, uint8_t init_duty_cycle);
float getVelocity(int transition_count);

/* Initialize ADC16 */
static void init_adc(void);
static void init_board(void);
void read_ADC(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool ftmIsrFlag = false;
volatile uint8_t duty_cycle = 1U;
volatile uint32_t systime = 0; //systime updated very 100 us = 4 days ==> NEED OVERFLOW protection

/* PWM duty cycle percentage limits */
//const int MOTOR_DUTY_MIN = 0; // for motor drive
//const int MOTOR_DUTY_MAX = 40;
const int SERVO_DUTY_MIN = 0; // for servo steering
const int SERVO_DUTY_MAX = 100;

// ADC variables
volatile bool g_Adc16ConversionDoneFlag = false;
volatile uint32_t g_Adc16ConversionValue = 0;
adc16_channel_config_t g_adc16ChannelConfigStruct;

// Velocity control variables
const float ADC_high_thresh = 2.5f; // the cutoff for a "high" signal from the optical encoder in volts
const float Kp = 0.30; // proportional gain
const float desVel = 3.0; // desired velocity [m/s]
float analog_voltage = 0;  // the voltage detected by the ADC from the optical encoder
float currVel = 0.0f; // current velocity
int ADC_state = 0; // whether the ADC is in a high or low state so we can count transitions
int prev_ADC_state = 0; // the previous loop's ADC state for comparison
int transition_count = 0; // the number of counts detected in the time step

// Line scan variables
int picture[128];
int Max, Min;

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
void init_pwm(uint32_t freq_hz, uint8_t init_duty_cycle)
{
	duty_cycle = init_duty_cycle;
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;

    /* Configure ftm params with for pwm freq- freq_hz, duty cycle- init_duty_cycle */
    ftmParam.chnlNumber = BOARD_FTM_CHANNEL;
    ftmParam.level = PWM_LEVEL;
    ftmParam.dutyCyclePercent = init_duty_cycle;
    ftmParam.firstEdgeDelayPercent = 0U;

    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    FTM_Init(BOARD_FTM_BASEADDR, &ftmInfo);

    FTM_SetupPwm(BOARD_FTM_BASEADDR, &ftmParam, 1U, kFTM_CenterAlignedPwm, freq_hz, FTM_SOURCE_CLOCK);

    /* Enable channel interrupt flag.*/
    FTM_EnableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE);

    /* Enable at the NVIC */
    EnableIRQ(FTM_INTERRUPT_NUMBER);

    FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
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

void update_duty_cycle(uint8_t updated_duty_cycle)
{
	FTM_DisableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE);

	/* Disable channel output before updating the dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR, BOARD_FTM_CHANNEL, 0U);

	/* Update PWM duty cycle */
	FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR, BOARD_FTM_CHANNEL, kFTM_CenterAlignedPwm, updated_duty_cycle);

	/* Software trigger to update registers */
	FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR, true);

	/* Start channel output with updated dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR, BOARD_FTM_CHANNEL, PWM_LEVEL);

	/* Delay to view the updated PWM dutycycle */
	// delay(); //Can be removed when using PWM for realtime applications

	/* Enable interrupt flag to update PWM dutycycle */
	FTM_EnableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE);
}

static void init_adc(void)
{
	EnableIRQ(DEMO_ADC16_IRQn);
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
    ADC16_Init(DEMO_ADC16_BASEADDR, &adc16ConfigStruct);

    /* Make sure the software trigger is used. */
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASEADDR, false);
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASEADDR))
    {
        PRINTF("\r\nADC16_DoAutoCalibration() Done.");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    /* Prepare ADC channel setting */
    g_adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    g_adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true;

#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    g_adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
}

void read_ADC(void){
	g_Adc16ConversionDoneFlag = false;
	ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP, &g_adc16ChannelConfigStruct);

	//Block until the ADC Conversion is finished
	 while (!g_Adc16ConversionDoneFlag)
	 {
	 }
}

void DEMO_ADC16_IRQ_HANDLER_FUNC(void)
{
    g_Adc16ConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
    g_Adc16ConversionValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP);
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

	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(50U, PIT_SOURCE_CLOCK)); // 50us timing
	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
	/* Enable at the NVIC */
	EnableIRQ(PIT_IRQ_ID);
	/* Start channel 0 */
	PIT_StartTimer(PIT, kPIT_Chnl_0);
}

//float getVelocity(int transition_count) {
//	// number of transitions from high to low signal off the ADC per 0.05s into vehicle velocity
//	// this function outputs a float, vehicle velocity to be used in the velocity controller
//
//	// float counts_per_sec = transition_count / 0.05; // transitions per second
//	// float revs_per_second = counts_per_sec / 4; // depending on the light and dark surfaces on the gearing
//
//	// gear ratio is 1.65:1
//	// wheel diameter is ~56mm = 0.056 m
//
//	// distance per tick = pi * diameter / (ticks per rev * gear ratio)
//	// velocity = distance per tick / time elapsed
//	float velocity = (0.056*3.14159265358979323846/(4*1.65))/0.05; // in [m/s]
//	return velocity;
//}

int *capture(void)
{
    int i;
    Max = 0, Min = 70000;
    SI_HIGH; //set SI to 1
    delay(1);
    CLK_HIGH; //set CLK to 1
    delay(1);
    SI_LOW; //set SI to 0
    for (i = 0; i < 127; i++) { //loop for 128 pixels
        CLK_HIGH;
        delay(1);
        read_ADC();
        picture[i] = g_Adc16ConversionValue; //store data in array
        CLK_LOW;
        delay(1);
    }
    return picture;
}

int main(void)
{
	init_board();
	init_adc();
	init_gpio();
	init_pit();
//	init_pwm(10000, 0); //start 10khz pwm at 0% duty cycle, for motor drive
//	int duty_cycle = 0;
	init_pwm(500, 40); //start 500hz pwm at 40% duty cycle, for servo steer

//	int duty_cycle = 25;

	while (1) {

        int *data = capture();
//        int left = 0;
//        int left_set = 0;
//        int right = 0;
//
//        for (int i = 0; i < 125; i++) {
//            int v = (data[i]/65535);
//            if (v == 0 && !left_set) {
//                left = i;
//                left_set = 1;
//            }
//            if (v == 0) {
//                right = i;
//            }
//        }
//
//        int max = (left + right)/2;
//        //some stuff to calculate what to set the duty cycle to
//        duty_cycle = 25; //20% - left = 1000, 28% - center, 36% - right = 1800
//        if (max != 0) {
//            update_duty_cycle(duty_cycle);
//        }

		PRINTF("Camera: %d \n", data[45]);

//		print the systime so we can see it increment for debugging
//		 PRINTF("\r\n\r\nThe systime is %d\r\n", systime);

//		char ch = GETCHAR(); //read from serial terminal
//		if (ch == 's') { //decrease throttle
//			if (duty_cycle - 2 < MOTOR_DUTY_MIN) { //case where duty cycle falls below 0%
//				duty_cycle = MOTOR_DUTY_MIN;
//			} else {
//				duty_cycle -= 2;
//				update_duty_cycle(duty_cycle); //subtract 2% from duty cycle
//			}
//		} else if (ch == 'w') { //increase throttle
//			if (duty_cycle + 2 > MOTOR_DUTY_MAX) { //case where duty cycle exceeds 40%
//				duty_cycle = MOTOR_DUTY_MAX;
//			} else {
//				duty_cycle += 2;
//				update_duty_cycle(duty_cycle); //add 2% to duty cycle
//			}
//		if (ch == 'a') { //turn left
//			if (duty_cycle - 2 < SERVO_DUTY_MIN) { //case where duty cycle falls below 0%
//				duty_cycle = SERVO_DUTY_MIN;
//			} else {
//				duty_cycle -= 2;
//				update_duty_cycle(duty_cycle); //subtract 2% from duty cycle
//			}
//		} else if (ch == 'd') { //turn right
//			if (duty_cycle + 2 > SERVO_DUTY_MAX) { //case where duty cycle exceeds 40%
//				duty_cycle = SERVO_DUTY_MAX;
//			} else {
//				duty_cycle += 2;
//				update_duty_cycle(duty_cycle); //add 2% to duty cycle
//			}
//		}

		// read the ADC and see if there has been a transition for velocity state estimation
//		read_ADC();
//		analog_voltage = (float)(g_Adc16ConversionValue * (VREF_BRD / SE_12BIT)); // get the analog voltage off that pin
//		if (analog_voltage > ADC_high_thresh) {
//			ADC_state = 1; // the ADC detected a high signal off the optical encoder
//		}
//		else {
//			ADC_state = 0; // the ADC detected a low signal off the optical encoder
//		}
//
//		if (prev_ADC_state != ADC_state) {
//			// there has been a transition from high to low or low to high, increment the counter
//			transition_count++;
//		}
//
//		prev_ADC_state = ADC_state;
//
//		// at a frequency of 20Hz, return the number of transitions detected on the ADC to get velocity estimate
//		if (systime*100 % 5 == 0) {
//			systime = 0; // reset the systime as to prevent overflow
//			currVel = getVelocity(transition_count); // get the velocity estimate from the number of transitions in 0.05s (20Hz) to use in our velocity controller
//			// PRINTF("\r\nVelocity: %0.3f\r\n", currVel);
//			transition_count = 0; // reset the transition counter
//		}
//
//		// proportional control
//		float error = desVel - currVel;
//		if (error > 0.0) {
//			duty_cycle = Kp*error;
//			update_duty_cycle(duty_cycle);
//		}
	}
}

/*******************************************************************************
 * Interrupt functions
 ******************************************************************************/
// Just clears the status flag. More functionality could be added here
void FTM_0_HANDLER(void)
{
    ftmIsrFlag = true;
    if ((FTM_GetStatusFlags(BOARD_FTM_BASEADDR) & FTM_CHANNEL_FLAG) == FTM_CHANNEL_FLAG)
    {
        /* Clear interrupt flag.*/
        FTM_ClearStatusFlags(BOARD_FTM_BASEADDR, FTM_CHANNEL_FLAG);
    }
}

void PIT0_IRQHandler(void) //clear interrupt flag
{
	systime++; /* hopefully atomic operation */
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
    pitIsrFlag = true;
}
