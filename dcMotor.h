#ifndef DCMOTOR_H
#define DCMOTOR_H


#define motor1Front 0x80			//pin 7
#define motor1Back 0x40				//pin 6
#define motor2Front 0x10			//pin 4
#define motor2Back 0x4				//pin 2

int initDcMotor(void);
void dcMotorForward(void);
void dcMotorLeft(void);
void dcMotorRight(void);
void dcMotorStop(void);
void dcMotorForwardSlow(void);


#endif // BUILD_H
