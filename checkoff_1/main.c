/*
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

/* Kernel includes. */
#include "FreeRTOS.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_pit.h"  /* periodic interrupt timer */

#include "pin_mux.h"
#include "clock_config.h"

// Use pin B23
#define GPIO_CHANNEL GPIOB
#define GPIO_PIN_NUM 23U

/*******************************************************************************
 * Periodic Interrupt Timer (PIT) Definitions
 ******************************************************************************/
#define PIT_IRQ_ID PIT0_IRQn
/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
volatile bool pitIsrFlag = false;
volatile uint32_t systime = 0; //systime updated very 100 us = 4 days ==> NEED OVERFLOW protection

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Main function
 */
int main(void)
{	float pif = 3.14159;
	double pid = 3.14159;

    // Configure an output pin with initial value 0
    gpio_pin_config_t gpio_config = {
                kGPIO_DigitalOutput, 0,
            };

   /* Structure of initialize PIT (periodic interrupt timer) */
    pit_config_t pitConfig;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    asm (".global _printf_float"); // cause linker to include floating point

    /* initialize GPIO Pin */
    GPIO_PinInit(GPIO_CHANNEL, GPIO_PIN_NUM, &gpio_config); // initialize GPIOB pin PTB23

    /* initialize LEDs */
 	//LED_GREEN_INIT(LOGIC_LED_OFF);

    /* welcome messages */
	PRINTF("Floating Point PRINTF %8.4f  %8.4lf\n\r", pif, pid); //print to UART (serial)
	PRINTF("Hello World\n"); //print to UART (serial)

	// Comment this out for release or it will not work!!!
	//	printf("Floating point printf %8.4f  %8.4lf\n\r", pif, pid); //print to semihost(debug console)

	/* start periodic interrupt timer- should be in its own file */
 	PIT_GetDefaultConfig(&pitConfig);
 	    /* Init pit module */
 	    PIT_Init(PIT, &pitConfig);
 	    /* Set timer period for channel 0 */


 	    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(250000U, PIT_SOURCE_CLOCK)); // 0.25s timing
 	    /* Enable timer interrupts for channel 0 */
 	    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
 	    /* Enable at the NVIC */
 	    EnableIRQ(PIT_IRQ_ID);
 	    /* Start channel 0 */
 	    PRINTF("\r\nStarting channel No.0 ...");
 	    PIT_StartTimer(PIT, kPIT_Chnl_0);

    for (;;);
}

/*******************************************************************************
 * Interrupt functions
 ******************************************************************************/

void PIT0_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
    pitIsrFlag = true;
    if (systime % 4 == 3) {
    	GPIO_PortToggle(GPIO_CHANNEL, 1u << GPIO_PIN_NUM); //toggle off
    } else if (systime % 4 == 0) {
    	GPIO_PortToggle(GPIO_CHANNEL, 1u << GPIO_PIN_NUM); //toggle on
    }
	systime++; /* hopefully atomic operation */
}
