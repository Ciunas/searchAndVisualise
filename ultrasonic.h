#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#define ULTRASONIC_TRIG 0x100; 			//pin 8 green
#define ULTRASONIC_ECHO 0x200; 			//pin 9 orange
//extern long tickcount;
//extern int pulse;
int initUltrasonic(void);
void UltrasonicWave(void);


#endif // BUILD_H
