#ifndef DRIVER_H_
#define DRIVER_H_

#define BASE_WIDTH 300						//75% duty cycle
#define BASE_DUTY 75						//75% duty cycle
#define PERIOD 400

void InitHardware(void);
void InitTimer(void);
void InitMotors(void);
void InitBluetooth(void);
void InitAnalog(void);

#endif /* DRIVER_H_ */


