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
 *	Name Ciunas Low Bennett
 *	Date 08/12/15
 *	LAB 16
 *	This lab uses I2C functions to interface the KL26Zmicrocontroller to the accelerometer built into the board. First part otf the
 *	Program reads the WHO_AM_I_reg and TEMPERAUTRE registers then prints the value of these registers to the terminal.
 *	Second part of the program reads the FXOS8700PL_STATUS register. THis register is a 8 bit register so the values have to be masked
 *	to get the values we want. The orientation is calculated then printed  to the terminal.
 **********************************************************************************************/
#define col 10
#include "board.h"
#include "fsl_clock_manager.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "gpio.h"
#include "FXOS8700.h"
#include "i2c.h"
#include "pit_kl26z.h"
#include "string.h"
#include "math.h"
#include "accell.h"
#include "ultrasonic.h"
#include "dcMotor.h"
#include "courseCalculator.h"

//State machines values THese are all the different maneuvers the robot can perform.
enum STATES {STOP, PAUSE, FORWARD, TURNRIGHT, TUNRLEFT, CALCCOURSE, CALCOLUMN, CALROW, PRINT, FREEMEMORY}; //FREEMEMORY
enum STATES currentState = CALCCOURSE;

void initialisationSuccess();
void ultraSonicSingle();
void arrayElementsToZero(int m, int n, char arr[][col]);
void arrayPrint(int m, int n, char arr[][col]);


//Variables.

//signed int aceldataX, velocityReturnX, positionReturnX;
volatile int tickcount_1;
volatile int tickcount_2;
volatile char tickcount_3 = 0;
volatile int start, finish;
volatile int forwardEnd;
char count;
volatile int pulse;
char init[4];

//Main loop.
int main() {
	char array[10][10];
	 int rowCounter = 0, columnCounter = 0;
	volatile int  row, column, calculatedMove;
	char k = 2;
	char flag = 0;
	char  temp1 = 0;
	char dirValues[4] = {'N','E', 'S', 'W'};					//Array of elements to describe direction tank is facing.
	char direction = dirValues[2];								//Default direction west.
	hardware_init();											//Initialize the FRDM-KL26Z Board.
	i2c0_configure();											//Configure i2c
	initAccellerometer();								//COnfigure the accelerometer and magnetometer.
	initDcMotor();									//Initialise the CD motors.
	//Calibrate();
	initUltrasonic();									//Calibrate accelerometer.
	//initialisationSuccess();
	PIT0_Configure_interrupt_mode(.0001); 						//Configure NVIC.
	PIT_TCTRL0 = 0;
	PIT1_Configure_interrupt_mode(.1);								//Configure NVIC.
	PRINTF("\033[2J");														//Clear screen.
	PRINTF("\033[0;0H"); 													//set cursor to 0,0.

	//Loop Continuously.
	while (1) {

		switch (currentState) {				//Switch the different maneuvers the robot can perform.

		//This state is called to stop the robot, when the room is scanned or if there is an error.	Print out data once and stop the motors.
		case STOP:
			if (temp1 < 1) {
				temp1++;
				dcMotorStop();
				PRINTF("Room has been surveyed below is the readings from the robot.\r\n");
				arrayPrint(row, column, array);
			}

			break;

		//State to pause after each maneuver, Pause state last for one second. Then changes the state to calculate next move.
		case PAUSE:
			dcMotorStop();
			if (tickcount_1 >= 10) {
				PRINTF("PAUSE\r\n");
				currentState = CALCCOURSE;
			}
			break;
		// State to free memory when array is created using Malloc.
		case	FREEMEMORY:
			break;

		//Print the 2d array that is created. Shows the areas that have been checked in the room. Following characters are used.
		//E = empty, O = object, U = unchecked.
		case PRINT:
			arrayPrint(row, column, array);						//Send values to function.
			currentState = PAUSE;
			break;

		//State to move the robot forward. Checks for objects in the way. Will set flag to one or two to show there is an object in
		//front of the robot.
		case FORWARD:
			//Run once to reset tickcount_1 and calculate the time the robot will move forward, by setting forwardEnd.
			if (tickcount_3 < 1) {			//When  case statement is called forwardEnd is set to the same time as tickcount + 2.
				tickcount_3++; 				//This is done so to give us forward motion for 1 second.
				tickcount_1 = 0;
				//PIT_TCTRL0 |= 0x03ul;//Enable PITO timer and interrupt to calculate the time to move forward.
				forwardEnd = (tickcount_1 + 5);//when tickcount_2  reaches forwardEnd robot stops.
				pulse = 100;

			}

			//Switch statement to check for an obstacle is in the way of the  robot. Will set flag only in certain situations.
			//If robot is in element zero or in the last element of  rows the flag is set and state is  changed immediately.
			//If robot is not in element zero or not in the last element of  rows the flag is set but state is not changed.
			switch (direction) {

			case 'E'://Facing East and in the first element of the column.
				if (columnCounter == 0 && pulse > 8 && pulse < 20) {					//Check if in the first element.
					flag = 2;//Set flag to 2											//If obstacle in second element stop immediately.
					pulse = 100;														//Set pulse.
					tickcount_3 = 0;
					currentState = PAUSE;												//Return to pause to calculate the course.
				} else if (columnCounter > 0 && columnCounter < column - 1 && pulse > 8 && pulse < 20) {//If not in first element set flag
					//tickcount_1 = 0;
					pulse = 100;
					flag = 1;
					PRINTF("something in the way  'E' flag: %d\r\n", flag);
				}
				break;

			case 'W'://Facing West and in the last element of the column.
				if (columnCounter == column - 1 && (pulse > 8 && pulse < 20)) {
					flag = 2;
					pulse = 100;
					tickcount_3 = 0;
					currentState = PAUSE;
				} else if (columnCounter > 0 && columnCounter < column - 1 && pulse > 8 && pulse < 20) {//If not in last element set flag
					//tickcount_1 = 0;
					pulse = 100;
					flag = 1;
					PRINTF("something in the way  'W' flag: %d\r\n", flag);
					//currentState = PAUSE;
				}
				break;

			case 'N'://Facing North if object in front of robot set flag and change state.
				if ((rowCounter > 0) && pulse > 8 && pulse < 20) {
					flag = 2;
					pulse = 100;
					tickcount_3 = 0;
					PRINTF("something in the way  'N' flag: %d\r\n", flag);
					currentState = PAUSE;
				}
				break;

			case 'S'://Facing South if object in front of robot set flag and change state.
				if (rowCounter < (row - 1) && pulse > 8 && pulse < 20) {
					flag = 2;
					tickcount_3 = 0;
					PRINTF("something in the way  'S' flag: %d\r\n", flag);
					pulse = 100;
					currentState = PAUSE;
				}
				break;
			default:								//Default state set to INIT.
				PRINTF("ERROR: STOP\r\n");
				currentState = STOP;
				break;
			}

			//Send power to motor until timer runs down.
			if (tickcount_1 <= forwardEnd) {
				dcMotorForward();
			} else {								//When timer runs down change state to Print.
				tickcount_3 = 0;
				NVIC_DisableIRQ(31);				//Disable interrupts on port c
				PIT_TCTRL0 = 0;						//PIT0 disabled.
				currentState = PRINT;				//Print new array values.
			}
			break;

		case TURNRIGHT:								//Turn robot to the right for set amount of time.
			if (tickcount_3 < 1) {
				tickcount_3++;
				tickcount_2 = 0;
				PIT_TCTRL0 |= 0x03ul;				//Enable PITO timer and interrupt.
			}
			dcMotorRight();							//Function to turn motors
			if (tickcount_2 >= 2450) {				//If timer run out increment direction robot facing and change state to pause.
				PIT_TCTRL0 = 0;						//PIT0 disabled.
				if (++k == 4) {						//Increment k which is related to the direction the robot is facing.
					k = 0;							//If equal to four set to zero.
				}

				direction = dirValues[k];			//Set direction to value of k.
				//PRINTF("ROBOT Facing %c\r\n", direction);
				tickcount_1 = 0;
				tickcount_2 = 0;
				tickcount_3 = 0;
				currentState = PAUSE;				//Change state to pause when timer runs out.
			}
			break;

		case TUNRLEFT:								//Turn robot to the left for set amount of time.

			if (tickcount_3 < 1) {					//Only run once to reset timers and enable PIT0
				tickcount_3++;
				tickcount_2 = 0;
				PIT_TCTRL0 |= 0x03ul;				//Enable PITO timer and interrupt.
			}
			dcMotorLeft();							//Function to turn motors.
			if (tickcount_2 >= 2450) {
				PIT_TCTRL0 = 0;						//PIT0 disabled.
				if(--k == -1){						//Decrement k which is related to the direction the robot is facing.
					k = 3;							//If equal to minus one set to 3.
				}
				direction = dirValues[k];
				//PRINTF("ROBOT Facing %c\r\n", direction);
				tickcount_1 = 0;
				tickcount_2 = 0;
				tickcount_3 = 0;
				currentState = PAUSE;				//Change state to pause when timer runs out.
			}
			break;

		case CALROW:								//When alarm triggered toggle red LED.
			if (temp1 < 1) {						//Only run once to reset timer and send out one ultrasonic pulse.
				tickcount_1 = 0;
				temp1++;
				ultraSonicSingle();					//Call function to calculate distance to any object.
			}

			//Wait for interrupt on Portc. This is the interrupt caused by the ultrasonic.  A count of 1 second is used to give time
			//for the wave to come back from the ultrasonic. After one second another wave is sent out.
			//this is done because the first reading off the ultrasonic is unreliable.
			//What ever value comes back from the ultrasonic is then passed to the function distanceCalculator
			//this calculates how many columns are needed depending on the size of the room.
			if (tickcount_1 == 10) {
				tickcount_1++;
				ultraSonicSingle();
			} else if (tickcount_1 >= 20) {
				temp1 = 0;
				row = distanceCalculator(pulse);
				pulse = 100;
				PRINTF("Value of column: %d\r\n", row);
				tickcount_1 = 0;
				currentState = PAUSE;
			} else {}

			break;

		//Wait for interrupt on Portc. This is the interrupt caused by the ultrasonic.  A count of 1 second is used to give time
		//for the wave to come back from the ultrasonic. After one second another wave is sent out.
		//this is done because the first reading off the ultrasonic is unreliable.
		//What ever value comes back from the ultrasonic is then passed to the function distanceCalculator
		//this calculates how many rows are needed depending on the size of the room.
		//A 2d array is then created then using the row and column values.
		//The state is then changed to PAUSE.
		case CALCOLUMN:
			if (temp1 < 1) {
				tickcount_1 = 0;
				temp1++;
				ultraSonicSingle();
			}

			if (tickcount_1 == 10) {
				tickcount_1++;
				ultraSonicSingle();
			} else if (tickcount_1 >= 20) {
				PIT_TCTRL0 = 0;											//PIT0 disabled.
				column = distanceCalculator(pulse);
				pulse = 100;
				PRINTF("Value of Row: %d\r\n", column);
				tickcount_2 = 0;
				arrayElementsToZero(row, column, array);
				tickcount_1 = 0;
				temp1 = 0;
				currentState = PAUSE;       //pause
			} else {}
			break;

		//This case statement calls the function to calculateNexMove  which returns the next move the robot needs to do.
		case CALCCOURSE:												//Calculate the next movement of the robot
			PRINTF("Calculate course\r\n");								//CalculateCourse == 0.
			calculatedMove = calculateNextMove(array, direction, row, column , &flag, &rowCounter, &columnCounter);
			if (calculatedMove == 0) {
				PRINTF("Course returned: STOP\r\n");					//Stop == 0.
				currentState = STOP;
			} else if (calculatedMove == 1) {
				PRINTF("Course returned: PAUSE\r\n");					//Pause == 1.
				currentState = PAUSE;
			} else if (calculatedMove == 2) {
				PRINTF("Course returned: FORWARD\r\n");					//Forward == 2.
				currentState = FORWARD;
			} else if (calculatedMove == 3) {
				PRINTF("Course returned: TURNRIGHT\r\n");				//TurnRight == 3.
				currentState = TURNRIGHT;
			} else if (calculatedMove == 4) {
				PRINTF("Course returned: TUNRLEFT\r\n");				//TurnLeft == 4.
				currentState = TUNRLEFT;
			} else if (calculatedMove == 5) {
				PRINTF("Course returned: CALROW\r\n");				//CalculateRow == 5.
				currentState =  CALROW;
			} else if (calculatedMove == 6) {
				PRINTF("Course returned: CALCOLUMN\r\n");					//CalculateColumn == 6.
				currentState = CALCOLUMN;
			} else {
				PRINTF("ERROR: STOP\r\n");								//Stop == else.
				currentState = STOP;
			}
			break;

		default:									//Default state set to INIT.
			PRINTF("ERROR: STOP\r\n");
			currentState = STOP;
			break;
		}
	}
}


//Interrupt Service Routine for PIT
void PIT_IRQHandler() {
	if (PIT_TFLG1 == 0x01ul) {
		tickcount_1++;										//Increment tickcount
		PIT_TFLG1 = 0x01ul;									//Clear flag
		if (currentState == FORWARD) {						//If current state == forward call the ultrasonic() function.
//			position(&aceldataX, &positionReturnX, &velocityReturnX);//Call accelerometer function.
//			PRINTF("%d\t%d\t%d\r\n", aceldataX, velocityReturnX,positionReturnX);
			UltrasonicWave();
			start = tickcount_2;							//Set start to tickcount_2.
			PIT_TCTRL0 |= 0x03ul;							//Enable PITO timer and interrupt.
		}
	} else if(PIT_TFLG0 == 0x01ul){
		PIT_TFLG0 = 0x01ul;    								//Clear flag
		tickcount_2++;
	}
}


// Interrupt Service Routine for port c
void PORTC_PORTD_IRQHandler() {
	if (count++ == 0) {
		PORTC_PCR9 |= FALLNG_EDGE_INTERRUPT_MASK;			//Configure interrupt falling edge.
	} else {
		finish = tickcount_2;
		pulse = (finish - start);
		PIT_TCTRL0 = 0;										//PIT0 disabled.
		PORTC_PCR9 |= RISING_EDGE_INTERRUPT_MASK;			//configure interrupt rising edge.
		NVIC_DisableIRQ(31);								//Disable interrupt on port c.
		count = 0;											//Reset count.
		tickcount_2 = 0;
		PRINTF("Pulse: %d\r", pulse);
	}
	PORTC_ISFR |= ULTRASONIC_TRIG;							//Clear Interrupt flag.
}


//Function to use the ultrasonic sensor on it s own
void ultraSonicSingle() {
	UltrasonicWave();						//Call ultrasinic function this triggers the sensor to send out sonic waves.
	start = tickcount_2;					//Set start to tickcount_2.
	PIT_TCTRL0 |= 0x03ul;					//Enable PITO timer and interrupt.
}


//Function to set the values of my 2D array to zero.
void arrayElementsToZero(int row, int column, char arr[][col]) {
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			arr[i][j] = 'U';
		}
	}
}


//Function to print the values of my 2D array to terminal.
void arrayPrint(int row, int column, char arr[][col]) {
	PRINTF("\r\n");
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
	        PRINTF("[%c]", arr[i][j]);
	      }
	      PRINTF("\r\n");
	    }
}


//Function to check all the initialisation has completed successfully.
void initialisationSuccess() {
	char initSuccess;
	for (int i = 0; i < 3; i++) {
		if (init[i] != 0) {
			initSuccess = 1;
		}
	}
	if (initSuccess == 1) {
		PRINTF("Error Initialisation failed\n\r");
		currentState = PAUSE;
	} else {
		PRINTF("Initialisation completed.\n\r");
	}
}


