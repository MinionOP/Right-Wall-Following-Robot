#ifndef DRIVER_H_
#define DRIVER_H_


/*
#define BASE_WIDTH 300
#define PERIOD 400
#define BASE_DUTY 75	//75% duty cycle
*/


#define BASE_WIDTH 340
#define PERIOD 400
#define BASE_DUTY 85	//85% duty cycle


void InitHardware(void);
void InitTimer(void);
void InitMotors(void);
void InitBluetooth(void);
void InitAnalog(void);

#endif /* DRIVER_H_ */

