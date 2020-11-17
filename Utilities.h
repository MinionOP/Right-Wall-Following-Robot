#ifndef UTILITIES_H_
#define UTILITIES_H_

#define BASE_WIDTH 300						//75% duty cycle
#define BASE_DUTY (BASE_WIDTH/PERIOD) * 100		//75% duty cycle
#define PERIOD 400

//read Front Distance Sensor
double readFront(void);
//read Right Distance Sensor
double readRight(void);

//Motor Control
//set speed for left and right motor
void wheelDuty(double lDuty, double rDuty);
//wheelNum(0) = left, (1) = right, (2) = both. dir(0) = reverse dir(1) = forward
void wheelDir(uint32_t wheelNum, uint32_t dir);
//wheel(0) = left, (1) = right, (2) = both. power(off) = stop motor, power(on) = start motor
void wheelPower(uint32_t wheelNum, char* power);
void rightTurn(void);
void uTurn(void);

//Light sensor
uint8_t lightSensor(char colorLine);

//Print to console
void print(char* string);

#endif /* UTILITIES_H_ */
