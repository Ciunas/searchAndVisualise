#include "dcMotor.h"
#include "gpio.h"


int initDcMotor(void) {
	SIM_SCGC5 |= PORTD_CLK_ENABLE_MASK;					//turn on port D clock
	PORTD_PCR2 |= 0x1U << 8;							//PTD2 mux = GPIO    (PCR1 == pin number 1)&&(PORTD == port D)
	PORTD_PCR4 |= 0x1U << 8;							//PTD4 mux = GPIO    (PCR2 == pin number 2)&&(PORTD == port D)
	PORTD_PCR7 |= 0x1U << 8;							//PTD7 mux = GPIO    (PCR1 == pin number 1)&&(PORTD == port D)
	PORTD_PCR6 |= 0x1U << 8;							//PTD6 mux = GPIO    (PCR2 == pin number 2)&&(PORTD == port D)
	GPIOD_PDDR |= motor1Front;							//PTD7 output direction
	GPIOD_PDDR |= motor1Back;							//PTD6 output direction
	GPIOD_PDDR |= motor2Front;							//PTD4 output direction
	GPIOD_PDDR |= motor2Back;							//PTD2 output direction
	return 0;
}

//Function to move both motors forward.
void dcMotorForward(void) {
	GPIOD_PSOR |= motor1Front;
	GPIOD_PCOR |= motor1Back;
	GPIOD_PSOR |= motor2Back;
	GPIOD_PCOR |= motor2Front;
	return;
}


//Function to move both motors forward.
void dcMotorForwardSlow(void) {
	GPIOD_PSOR |= motor1Front;
	GPIOD_PCOR |= motor1Back;
	GPIOD_PSOR |= motor2Back;
	GPIOD_PCOR |= motor2Front;
	for (int i = 0; i < 10; i++);					//Short delay approximately 2 microseconds. Triggers the ultrasonic pulse.
	GPIOD_PCOR |= motor1Back;
	GPIOD_PCOR |= motor1Front;
	GPIOD_PCOR |= motor2Back;
	GPIOD_PCOR |= motor2Front;
	for (int i = 0; i < 5; i++);					//Short delay approximately .5 microseconds. Triggers the ultrasonic pulse.
	return;
}
//Function to move right motor forward and left motor backwards.
void dcMotorLeft(void) {
	GPIOD_PSOR |= motor1Front;
	GPIOD_PCOR |= motor1Back;
	GPIOD_PSOR |= motor2Front;
	GPIOD_PCOR |= motor2Back;
	//for (int i = 0; i < 10; i++);					//Short delay approximately 2 microseconds. Triggers the ultrasonic pulse.
//	GPIOD_PCOR |= motor1Back;
//	GPIOD_PCOR |= motor1Front;
//	GPIOD_PCOR |= motor2Back;
//	GPIOD_PCOR |= motor2Front;
	//for (int i = 0; i < 2; i++);					//Short delay approximately .5 microseconds. Triggers the ultrasonic pulse.
	return;
}

//Function to move left motor forward and right motor backwards.
void dcMotorRight(void) {
	GPIOD_PSOR |= motor1Back;
	GPIOD_PCOR |= motor1Front;
	GPIOD_PSOR |= motor2Back;
	GPIOD_PCOR |= motor2Front;
	//for (int i = 0; i < 10; i++);					//Short delay approximately 2 microseconds. Triggers the ultrasonic pulse.
//	GPIOD_PCOR |= motor1Back;
//	GPIOD_PCOR |= motor1Front;
//	GPIOD_PCOR |= motor2Back;
//	GPIOD_PCOR |= motor2Front;
	//for (int i = 0; i < 2; i++);					//Short delay approximately .5 microseconds. Triggers the ultrasonic pulse.
	return;
}

//Function to stop both motors.
void dcMotorStop(void) {
	GPIOD_PCOR |= motor1Back;
	GPIOD_PCOR |= motor1Front;
	GPIOD_PCOR |= motor2Back;
	GPIOD_PCOR |= motor2Front;
	return;
}


