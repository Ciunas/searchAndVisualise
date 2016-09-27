
#include "pit_kl26z.h"
#include "gpio.h"
#include "fsl_debug_console.h"

#define col 10
int processNumber = 0;
int processPipe[10];
char i,j,k;
int timesCounter = 0;
int timesLeft = 0;
int loadBuffer = 0;
char flag1 = 0;
char flag2 = 0;
char location;
enum STATES {N, E, S, W};
enum STATES currentMoves= S;

//calculate the next moves case 0 1 and 2 are ran first these calculate the row and column values using the ultrasonic
//sensor to measure the distances. Case three runs for the rest of the program
int calculateNextMove(char array[][col], char direction, int row, int column, char * flag, int  * rowCounter ,int * columnCounter) {
	int returnValue;
	char finish = 0;

	//if theis flag is set the robot stops immediately and services the request for course recalculation.
	if( *flag == 2 ){
		//PRINTF("flag == 1\r\n");
		processNumber = 5;						//Change process number  to 5.
	}

	switch (processNumber) {					//Switch the variable processNumber.
	case 0:
		processNumber++;
		returnValue = 5;						//Calculate the row values.
		break;
	case 1:
		processNumber++;
		returnValue = 4;						//Turn left.
		break;

	case 2:
		processNumber++;
		returnValue = 6;						//Calculate the column values.
		break;

	case 3:

		//Use variable direction that is passed to function from main to set currentMoves to what ever direction the tank is facing.
		switch (direction) {
		case 'N':
			currentMoves = N;
			if (flag1 == 1 && (timesLeft == 1 || timesLeft == 6)) {
				location = 'C';
			}
			//PRINTF("currentMoves: %c\r\n",currentMoves);
			break;
		case 'E':
			currentMoves = E;
			location = 'R';
			//PRINTF("currentMoves: %c\r\n",currentMoves);
			break;
		case 'S':
			currentMoves = S;
			if (flag1 == 1 && (timesLeft == 1 || timesLeft == 6)) {
				location = 'C';
			}
			//PRINTF("currentMoves: %c\r\n",currentMoves);
			break;
		case 'W':
			currentMoves = W;
			location = 'R';
			//PRINTF("currentMoves: %c\r\n",currentMoves);
			break;
		default:
			break;
		}

		if (loadBuffer == 0) {//THis code only implements when the robot has cleared all elements in processPipe[].

			switch (direction) {//Use variable direction that is passed to function from main to set currentMoves to what ever direction the tank is facing.
			case 'N':
				currentMoves = N;
				location = 'C';
				//PRINTF("currentMoves: %c\r\n",currentMoves);
				break;
			case 'E':
				currentMoves = E;
				location = 'R';
				//PRINTF("currentMoves: %c\r\n",currentMoves);
				break;
			case 'S':
				currentMoves = S;
				location = 'C';
				//PRINTF("currentMoves: %c\r\n",currentMoves);
				break;
			case 'W':
				currentMoves = W;
				location = 'R';
				//PRINTF("currentMoves: %c\r\n",currentMoves);
				break;
			default:
				break;
			}

			//fill up buffer (processPipe[]) with the next set of moves that need to be performed.
			switch (currentMoves) {
			case N:								//Facing North.
				//location = 'C';
				timesCounter = 2;
				processPipe[0] = 2;
				processPipe[1] = 3;
				flag2 = 3;
				//PRINTF("location: %cows direction: %c\r\n",location, direction);
				break;

			case E:								//Facing East.
				//location = 'R';
				timesCounter = ( (column ) - *columnCounter);
				for (i = 0; i < ( (column - 1) - *columnCounter); i++) {
					processPipe[i] = 2;
					//PRINTF("processPipe[%d]: %d\r\n",i ,processPipe[i]);
				}
				processPipe [(column -1) - *columnCounter] = 3;
				//PRINTF("processPipe[%d]: %d\r\n",(column -1) - *columnCounter,processPipe[(column -1) - *columnCounter]);
				//PRINTF("location: %cows direction: %c timesCounter: %d\r\n",location, direction, timesCounter);
				break;

			case W:								//Facing West.
				//location = 'R';
				timesCounter = *columnCounter+1;
				for (i = 0; i < *columnCounter; i++) {
					processPipe[i] = 2;
					//PRINTF("processPipe[%d]: %d\r\n",i ,processPipe[i]);
				}
				//PRINTF("location: %cows direction: %c timesCounter: %d\r\n",location, direction, timesCounter);
				processPipe[*columnCounter ] = 4;
				//PRINTF("processPipe: %d\r\n",processPipe[*columnCounter]);
				break;

			case S:								//Facing South.
				//location = 'C';
				timesCounter = 2;
				processPipe[0] = 2;
				if(*columnCounter == column-1){
					processPipe[1] = 3;
				}else{
					processPipe[1] = 4;
				}
				flag2 = 4;
				//PRINTF("location: %cows direction: %c\r\n",location, direction);
				break;

			default:
				break;
			}
			loadBuffer++;
		} else {

			//If flag is equal to one escape from processes and create new maneuvers to avoid obstacle.
			if( *flag == 1 ){
				processNumber = 4;
				returnValue = 1;
				break;
			}
			//PRINTF("location: %c direction: %c flag2: %d\r\n",location, direction, flag2);
			//Keep track of where the tank is in the 2d array, and update the array with 1 in the element it is on.
			if (location == 'R' && direction == 'E'  && flag2 !=  11 && flag2 !=  6  && flag2 !=  3 ) {				//location == 'R' &&
				if (*columnCounter < column - 1) {
					*columnCounter = *columnCounter + 1;
					//PRINTF("adding to column\r\n");
				}

			} else if (location == 'R' && direction == 'W'  && flag2 !=  11 && flag2 !=  6 && flag2 !=  3) {		//location == 'R' &&
				//flag2 = 0;
				if (*columnCounter > 0) {
					*columnCounter = *columnCounter - 1;
					//PRINTF("SUbtracting from column\r\n");
				}

			} else if (location == 'C' && direction == 'S') {		//location == 'C' &&
				if (*rowCounter < row - 1) {
					*rowCounter = *rowCounter + 1;
					location = 'R';
				}
			} else if (location == 'C' && direction == 'N') {		//location == 'C' &&
				PRINTF("inside - row count\r\n");
				if (*rowCounter > 0) {
					*rowCounter = *rowCounter - 1;
					location = 'R';
				}
			}
			if( flag2-- <= 0){
				flag2 = 0;
			}
			PRINTF("rowCounter: %d columnCounter: %d\r\n", *rowCounter,*columnCounter);
		}

		if(timesLeft < timesCounter ){

			returnValue = processPipe[timesLeft];   				// Return the next move the robot must do.
			timesLeft++;
			array[*rowCounter][*columnCounter] = 'E';
		} else {													//when processPipe  buffer is empty reset flag loadBuffer, and set timesLeft to 0.

			//Check if all the elements in the array have been checked and return to end the program if they have.
			for (int i = 0; i < row; i++) {
				for (int j = 0; j < column; j++) {
			        if(array[i][j] == 'E' || array[i][j] == 'O'){
			        	if( ++finish == (row * column )){
			        		processNumber = 6;
			        	}
			        }
			      }
			    }
			loadBuffer = 0;
			timesLeft = 0;
			returnValue = 1;
			flag1 = 0;
		}
		break;

	case 4:				//When flag is set to one or two the following maneuvers are implemented. This is to avoid any obstacle in the course.

		if (direction == 'E') {
			//PRINTF("Inside the thing in way 'E'\r\n");
			processPipe[0] = 4;										//Left.
			processPipe[1] = 2;										//Forward.
			processPipe[2] = 3;										//Right.
			processPipe[3] = 2;										//Forward.
			processPipe[4] = 2;										//Forward.
			processPipe[5] = 3;										//Right
			processPipe[6] = 2;										//Forward
			processPipe[7] = 4;										//Left.
			int temp = *columnCounter + 1;
			array[*rowCounter][temp] = 'O';
			processNumber = 3;										//GO back to process 3
			timesCounter = 8;										//Set times to eight to complete avoidance maneuver.
			timesLeft = 0;
			location = 'C';
			flag1 = 1;
			flag2 = 11;
			*flag = 0;												//Reset  flag1

		} else if (direction == 'W') {
			//PRINTF("Inside the thing in way 'W'\r\n");
			processPipe[0] = 3;										//Right.
			processPipe[1] = 2;										//Forward.
			processPipe[2] = 4;										//Left.
			processPipe[3] = 2;										//Forward.
			processPipe[4] = 2;										//Forward.
			processPipe[5] = 4;										//Left.
			processPipe[6] = 2;										//Forward
			processPipe[7] = 3;										//Right.
			timesCounter = 8;
			int temp = *columnCounter - 1;
			array[*rowCounter][temp] = 'O';
			processNumber = 3;										//GO back to process 3
			timesCounter = 8;										//Set times to eight to complete avoidance maneuver.
			timesLeft = 0;
			location = 'C';
			flag1 = 1;
			flag2 = 11;
			*flag = 0;

		} else if ( direction == 'S') {

		} else if ( direction == 'N') {

		}
		returnValue = 1;
		break;

	//Flag is set if obstacle is directly in the way of the robot.
	//This flag is instantaneous as opposed to if the flag is set to one which only implements after the robot has performed the current maneuver.
	case 5:
		if (direction == 'E') {
				//PRINTF("Inside the thing in way 'E'\r\n");
				processPipe[0] = 4;										//Left.
				processPipe[1] = 2;										//Forward.
				processPipe[2] = 3;										//Right.
				processPipe[3] = 2;										//Forward.
				processPipe[4] = 2;										//Forward.
				processPipe[5] = 3;										//Right
				processPipe[6] = 2;										//Forward
				processPipe[7] = 4;										//Left.
				int temp = *columnCounter + 1;
				array[*rowCounter][temp] = 'O';
				processNumber = 3;										//GO back to process 3
				timesCounter = 8;										//Set times to eight to complete avoidance maneuver.
				timesLeft = 0;
				location = 'C';
				flag1 = 1;
				flag2 = 11;
				*flag = 0;												//Reset  flag1

			} else if (direction == 'W') {
				//PRINTF("Inside the thing in way 'W'\r\n");
				processPipe[0] = 3;										//Right.
				processPipe[1] = 2;										//Forward.
				processPipe[2] = 4;										//Left.
				processPipe[3] = 2;										//Forward.
				processPipe[4] = 2;										//Forward.
				processPipe[5] = 4;										//Left.
				processPipe[6] = 2;										//Forward
				processPipe[7] = 3;										//Right.
				timesCounter = 8;
				int temp = *columnCounter - 1;
				array[*rowCounter][temp] = 'O';
				processNumber = 3;										//GO back to process 3
				timesCounter = 8;										//Set times to eight to complete avoidance maneuver.
				timesLeft = 0;
				location = 'C';
				flag1 = 1;
				flag2 = 11;
				*flag = 0;

			} else if ( direction == 'S') {
				//processPipe[0] = 3;										//Right.

			} else if ( direction == 'N') {

			}
			returnValue = 1;
		break;

	case 6:
		returnValue = 0;				//Robot is finished print out array and power down the motors.

	default:									//Default state set to INIT.
		break;
	}
	return returnValue;
}




// Function to turn the ultrasonic data into a variable that can be used to create row and column values from.
int distanceCalculator(int number) {
	int returnValue;
	switch (number) {
	case 0 ... 20:
		returnValue = 1;
		break;
	case 22 ... 37:
		returnValue = 2;
		break;
	case 38 ... 56:
		returnValue = 3;
		break;
	case 57 ... 75:					//When alarm triggered toggle red LED.
		returnValue = 4;
		break;
	case 76 ... 92:					//Calculate the next movement of the robot
		returnValue = 5;
		break;
	case 93 ... 110:				//Calculate the next movement of the robot
		returnValue = 6;
		break;
	case 111 ... 127:				//Calculate the next movement of the robot
		returnValue = 7;
		break;
	case 128 ... 144:				//Calculate the next movement of the robot
		returnValue = 8;
		break;
	case 145 ... 168:				//Calculate the next movement of the robot
		returnValue = 9;
		break;
	default:									//Default state set to INIT.
		break;
	}
	return returnValue;
}


