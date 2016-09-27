/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
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

/***********************************************************************************************
 *	Ciunas Low Bennett
 *	22/01/2015
 *	Version 1.0
 **********************************************************************************************/
#include "ultrasonic.h"
#include "gpio.h"
#include "fsl_debug_console.h"



int initUltrasonic() {
	SIM_SCGC5 |= PORTC_CLK_ENABLE_MASK;								//enable port clock.
	PORTC_PCR9 |= GPIO_MUX_MASK;									//configure as GPIO pin.
	PORTC_PCR9 |= RISING_EDGE_INTERRUPT_MASK;						//configure interrupt rising edge.
	PORTC_PCR8 |= GPIO_MUX_MASK;									//PTC8 mux = GPIO.
	GPIOC_PDDR |= ULTRASONIC_TRIG;									//PORT data direction 1=OUTPUT.
	//PRINTF("initUltrasonic\n\r");			//Clear flag.
	return 0;
}

//function to send out wave
void UltrasonicWave(void){
	//PRINTF("Sending out pulse\n\r");			//Clear flag.
	NVIC_ClearPendingIRQ(31);					//Clear any pending interrupts on PortC.
	GPIOC_PCOR |= ULTRASONIC_TRIG;				//PCOR=1 corresponding pin is cleared
	for(int i=0; i<20; i++);					//Short delay approx 10 microseconds. Triggers the ultrasonic pulse.
	GPIOC_PSOR |= ULTRASONIC_TRIG;      		//PSOR=1 corresponding pin is set to 1
	for(int i=0; i<40; i++);					//Short delay approx 10 microseconds. Triggers the ultrasonic pulse.
	GPIOC_PCOR |= ULTRASONIC_TRIG;				//PCOR=1 corresponding pin is cleared
	NVIC_EnableIRQ(31);							//Enable interrupts from PortC module.
	return;
}




////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
