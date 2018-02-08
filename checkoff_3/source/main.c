/* Checkoff 3, team 6
 *
 *
 * The Clear BSD License

 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*System includes.*/
#include <stdio.h>

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_ftm.h"

#include "pin_mux.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* The Flextimer instance/channel used for board. Indicates pin PTC1, see pin view for more info */
#define BOARD_FTM_BASEADDR_DR FTM0 //drive
#define BOARD_FTM_CHANNEL_DR kFTM_Chnl_0

#define BOARD_FTM_BASEADDR_ST FTM1 //steer
#define BOARD_FTM_CHANNEL_ST kFTM_Chnl_0

/* Interrupt number and interrupt handler for the FTM instance used */
#define FTM_INTERRUPT_NUMBER_DR FTM0_IRQn //drive
#define FTM_0_HANDLER FTM0_IRQHandler

#define FTM_INTERRUPT_NUMBER_ST FTM1_IRQn //steer
#define FTM_1_HANDLER FTM1_IRQHandler

/* Interrupt to enable and flag to read; depends on the FTM channel used */
/* Can keep at Channel 0 for both FTM0 and FTM1 pins for now since both use Channel 0*/
#define FTM_CHANNEL_INTERRUPT_ENABLE_DR kFTM_Chnl0InterruptEnable //drive
#define FTM_CHANNEL_FLAG_DR kFTM_Chnl0Flag

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
/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK_DR CLOCK_GetFreq(kCLOCK_PlatClk) //drive
#define FTM_SOURCE_CLOCK_ST CLOCK_GetFreq(kCLOCK_BusClk) //steer (>457 hz)

//High Voltage(3.3V)=True for pwm
#define PWM_LEVEL kFTM_HighTrue


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */
void delay(void);
void update_duty_cycle(uint8_t updated_duty_cycle);
void init_pwm(uint32_t freq_hz, uint8_t init_duty_cycle);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool ftmIsrFlag = false;
volatile uint8_t duty_cycle = 1U;

const int SERVO_DUTY_MIN = 50;
const int SERVO_DUTY_CENTER = 75;
const int SERVO_DUTY_MAX = 99;

const int DRIVE_DUTY_MIN = 50;
const int DRIVE_DUTY_CENTER = 75;
const int DRIVE_DUTY_MAX = 99;

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

/*Initialize the Flexible Timer Module to Produce Driving Brushed Motor PWM with init_duty_cycle at freq_hz*/
void init_drive_pwm(uint32_t freq_hz, uint8_t init_duty_cycle)
{
	duty_cycle = init_duty_cycle;
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;

    /* Configure ftm params with for pwm freq- freq_hz, duty cycle- init_duty_cycle */
    ftmParam.chnlNumber = BOARD_FTM_CHANNEL_DR;
    ftmParam.level = PWM_LEVEL;
    ftmParam.dutyCyclePercent = init_duty_cycle;
    ftmParam.firstEdgeDelayPercent = 0U;

    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    FTM_Init(BOARD_FTM_BASEADDR_DR, &ftmInfo);

    FTM_SetupPwm(BOARD_FTM_BASEADDR_DR, &ftmParam, 1U, kFTM_CenterAlignedPwm, freq_hz, FTM_SOURCE_CLOCK_DR);

    /* Enable channel interrupt flag.*/
    FTM_EnableInterrupts(BOARD_FTM_BASEADDR_DR, FTM_CHANNEL_INTERRUPT_ENABLE_DR);

    /* Enable at the NVIC */
    EnableIRQ(FTM_INTERRUPT_NUMBER_DR);

    FTM_StartTimer(BOARD_FTM_BASEADDR_DR, kFTM_SystemClock);
}

/*Initialize the Flexible Timer Module to Produce Steering Servo PWM with init_duty_cycle at freq_hz*/
void init_steer_pwm(uint32_t freq_hz, uint8_t init_duty_cycle)
{
	duty_cycle = init_duty_cycle;
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;

    /* Configure ftm params with for pwm freq- freq_hz, duty cycle- init_duty_cycle */
    ftmParam.chnlNumber = BOARD_FTM_CHANNEL_ST;
    ftmParam.level = PWM_LEVEL;
    ftmParam.dutyCyclePercent = init_duty_cycle;
    ftmParam.firstEdgeDelayPercent = 0U;

    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    FTM_Init(BOARD_FTM_BASEADDR_ST, &ftmInfo);

    FTM_SetupPwm(BOARD_FTM_BASEADDR_ST, &ftmParam, 1U, kFTM_CenterAlignedPwm, freq_hz, FTM_SOURCE_CLOCK_ST);

    /* Enable channel interrupt flag.*/
    FTM_EnableInterrupts(BOARD_FTM_BASEADDR_ST, FTM_CHANNEL_INTERRUPT_ENABLE_ST);

    /* Enable at the NVIC */
    EnableIRQ(FTM_INTERRUPT_NUMBER);

    FTM_StartTimer(BOARD_FTM_BASEADDR_ST, kFTM_SystemClock);
}
/*******************************************************************************
 * Other Code
 ******************************************************************************/
void delay(void)
{
    volatile uint32_t i = 0U;
    for (i = 0U; i < 80000U; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

void update_drive_duty_cycle(uint8_t updated_duty_cycle)
{
	FTM_DisableInterrupts(BOARD_FTM_BASEADDR_DR, FTM_CHANNEL_INTERRUPT_ENABLE_DR);

	/* Disable channel output before updating the dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR_DR, BOARD_FTM_CHANNEL_DR, 0U);

	/* Update PWM duty cycle */
	FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR_DR, BOARD_FTM_CHANNEL_DR, kFTM_CenterAlignedPwm, updated_duty_cycle);

	/* Software trigger to update registers */
	FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR_DR, true);

	/* Start channel output with updated dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR_DR, BOARD_FTM_CHANNEL_DR, PWM_LEVEL);

	/* Delay to view the updated PWM dutycycle */
	delay(); //Can be removed when using PWM for realtime applications

	/* Enable interrupt flag to update PWM dutycycle */
	FTM_EnableInterrupts(BOARD_FTM_BASEADDR_DR, FTM_CHANNEL_INTERRUPT_ENABLE_DR);
}

void update_steer_duty_cycle(uint8_t updated_duty_cycle)
{
	FTM_DisableInterrupts(BOARD_FTM_BASEADDR_ST, FTM_CHANNEL_INTERRUPT_ENABLE_ST);

	/* Disable channel output before updating the dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR_ST, BOARD_FTM_CHANNEL_ST, 0U);

	/* Update PWM duty cycle */
	FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR_ST, BOARD_FTM_CHANNEL_ST, kFTM_CenterAlignedPwm, updated_duty_cycle);

	/* Software trigger to update registers */
	FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR_ST, true);

	/* Start channel output with updated dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR_ST, BOARD_FTM_CHANNEL_ST, PWM_LEVEL);

	/* Delay to view the updated PWM dutycycle */
	delay(); //Can be removed when using PWM for realtime applications

	/* Enable interrupt flag to update PWM dutycycle */
	FTM_EnableInterrupts(BOARD_FTM_BASEADDR_ST, FTM_CHANNEL_INTERRUPT_ENABLE_ST);
}

/*!
 * @brief Main function
 */
int main(void) {
	init_board();

	init_drive_pwm(20000, 75); //start 20kHz pwm at 75% duty cycle (for driver motor)
	init_steer_pwm(400, 75); //start 400Hz pwm at 75% duty cycle (for steering servo)

	int duty_cycle = 75;

	//  uint8_t i = 0;
	while (1) {
		char ch = GETCHAR(); //read from serial terminal
		if (ch == 'a') { //turn left
			if (duty_cycle - 1 < 50) { //case where duty cycle falls below 50%
				// PRINTF("full left");
				duty_cycle = SERVO_DUTY_MIN;
			} else {
				update_steer_duty_cycle(duty_cycle - 1); //subtract 1% from duty cycle
			}
		} else if (ch == 'd') { //turn right
			if (duty_cycle + 1 > 99) { //case where duty cycle exceeds 99%
				// PRINTF("full right");
				duty_cycle = SERVO_DUTY_RIGHT;
			} else {
				update_steer_duty_cycle(duty_cycle + 1); //add 1% to duty cycle
			}
		} else if (ch == 'w') { //increase throttle
			if (duty_cycle + 1 > 99) { //case where duty cycle exceeds 99%
				// PRINTF("full throttle");
				duty_cycle = DRIVE_DUTY_MAX;
			} else {
				update_drive_duty_cycle(duty_cycle + 1); //add 1% to duty cycle
			}
		} else if (ch == 's') { //decrease throttle
			if (duty_cycle - 1 < 50) { //case where duty cycle falls below 50%
				// PRINTF("no throttle");
				duty_cycle = DRIVE_DUTY_MIN;
			} else {
				update_drive_duty_cycle(duty_cycle - 1); //subtract 1% from duty cycle
			}
		} else {
			PRINTF("Command char not recognized.");
		}
	}

/*******************************************************************************
 * Interrupt functions
 ******************************************************************************/
// Just clears the status flag. More functionality could be added here
void FTM_0_HANDLER(void)
{
    ftmIsrFlag = true;
    if ((FTM_GetStatusFlags(BOARD_FTM_BASEADDR_DR) & FTM_CHANNEL_FLAG_DR) == FTM_CHANNEL_FLAG_DR)
    {
        /* Clear interrupt flag.*/
        FTM_ClearStatusFlags(BOARD_FTM_BASEADDR_DR, FTM_CHANNEL_FLAG_DR);
    }
}

void FTM_1_HANDLER(void) //steer
{
    ftmIsrFlag = true;
    if ((FTM_GetStatusFlags(BOARD_FTM_BASEADDR_ST) & FTM_CHANNEL_FLAG_ST) == FTM_CHANNEL_FLAG_ST)
    {
        /* Clear interrupt flag.*/
        FTM_ClearStatusFlags(BOARD_FTM_BASEADDR_ST, FTM_CHANNEL_FLAG_ST);
    }
}

