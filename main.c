//Team 9: Charle Nguyen, Edward Sotelo, Josh McHenry

#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include "ti/sysbios/knl/Semaphore.h"
#include <xdc/cfg/global.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/System.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"

#include "PIDController.h"
#include "Driver.h"
#include "Utilities.h"

#define BASE_WIDTH 340
#define PERIOD 400
#define BASE_DUTY 85	//85% duty cycle

enum state_enum {FORWARD, RIGHT, LEFT, STOP};

//Task
void lightSensorTask(void);
void PIDControllerTask(void);
void AcquireDataTask(void);
void TxDataTask(void);
void swapBuffer(void);

//Timer interrupt
void TimerInt(void);

uint8_t overLine = 0; //Will be true if robot crosses over line
char colorLine = 'b'; //Black crossline
volatile uint32_t time = 0;
volatile uint32_t timeForMaze = 0;
volatile uint32_t time2 = 1000;
uint8_t countDownStatus = 0; //Stop count down
uint8_t dataCollectStatus = 0;
int finishStatus = 0;
double currentLDuty = BASE_DUTY;
double currentLWidth = BASE_WIDTH;

PIDController pidS;
PIDController* pid = &pidS;
uint8_t state = FORWARD;

void main(void)
{
	InitPID(pid, PERIOD, BASE_WIDTH);
	InitHardware();
	BIOS_start();
}

//Timer interrupt. Interrupt number: 39
void TimerInt(void){
	//Clear timer interrupt
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	Swi_post(toDoHandle);
}

//software interrupt
void toDo(void){
	//increment time
	time = time+1;
	timeForMaze++;
	//Make sure robot doesn't count same line twice
	if(countDownStatus == 1 && time2 >0){
		time2--;
	}
	else if(time2 == 0){ // 
		time2 = 1000;
		countDownStatus = 0;
	}
	time = time%1000; // reset timer after 1 second
	//Post to lightSensor
	if(time%30 == 0 && countDownStatus == 0 && overLine < 3){
		Semaphore_post(LightSem);
	}
	//Post to PIDController every 50ms
	if(time%50 == 0 && !finishStatus){
		//Post semaphore, pend semaphore in PIDControllerLoop
		Semaphore_post(PIDSem);
	}
	//Post to AcquireData every 100ms
	if(time%100 == 0 && dataCollectStatus == 1 && overLine < 3){
		Semaphore_post(AcquireDataSem);
	}
}

//Task Handle: PIDHandle
void PIDTask(void){
	while(1){
		//Pend semaphore
		Semaphore_pend(PIDSem,BIOS_WAIT_FOREVER);
		double rightWall = readRight();
		double frontWall = readFront();
		double Correction = PIDUpdate(pid, rightWall);
		switch(state){
			case FORWARD:{
				wheelDuty(BASE_DUTY, Correction);
				if(overLine == 3){
					state = STOP;
				}
				else if(rightWall > 15 ){ // need to turn right
					setPIDRight(pid);
					state = RIGHT;
				}
				else if(rightWall <15 && frontWall < 6){ //  need to u turn
					wheelDuty(BASE_DUTY, BASE_DUTY);
					wheelDir(0,0);
					state = LEFT;
				}
				break;
			}
			case RIGHT:{
				wheelDuty(BASE_DUTY, Correction); // turn right
				if(rightWall < 9){ // go forward
					setPIDForward(pid);
					state = FORWARD;
				}
				break;
			}
			case LEFT:{
				if(rightWall < 15 && frontWall > 30){
					wheelDir(0,1);
					state = FORWARD;
				}
				break;
			}
			case STOP:{
				currentLWidth/=1.3;
				setPIDBaseWidth(pid, currentLWidth);
				if(currentLWidth < 50){
					wheelPower(2,0);
					finishStatus = 1;
					UARTprintf("You finished the maze in %d seconds!\n", timeForMaze/1000);
				}
				break;
			}
		}
	}
}

void lightSensorTask(void){
	while(1){
		//Pend semaphore
		Semaphore_pend(LightSem, BIOS_WAIT_FOREVER);
		uint8_t temp;
		temp = lightSensor(colorLine, dataCollectStatus);
		//Number of times robot cross over line
		overLine+=temp;

		if(temp == 1){
			countDownStatus = 1; 
		}
		//Check whether the robot should start or stop collecting data
		if(dataCollectStatus == 0 && temp == 1 || dataCollectStatus == 1 && temp == 0){
			dataCollectStatus = 1;
		}
		//Will clear remaining data once the robot have cross over the 2nd line
		else if(dataCollectStatus == 1 && temp == 1){
			//Set dataStatus equal to 0. Will stop the timer interrupt from posting to DataAcquire semaphore
			dataCollectStatus = 0;
			//Clear remaining Data
			Semaphore_post(TxDataSem);
		}
		else{
			dataCollectStatus = 0;
		}
	}
}

double Buffer1[20];
double Buffer2[20];

double *activeBuffer = Buffer1;
double *backBuffer = Buffer2;
uint8_t bufferPtr = 0;

int currentSample = 0;
int currentSample2 = 0;

void AcquireDataTask(void){
	while(1){
		Semaphore_pend(AcquireDataSem,BIOS_WAIT_FOREVER);
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x4);
		backBuffer[currentSample] = getPrevError(pid);
		//increment position
		currentSample++;
		currentSample2++;
		//Check if there's 20 data in the array
		if(currentSample == 20){
			currentSample = 0;
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x0);
			//Post to TxData. Begin transferring data back to pc
			Semaphore_post(TxDataSem);
		}
	}
}

void TxDataTask(void){
	while(1){
		Semaphore_pend(TxDataSem,BIOS_WAIT_FOREVER);
		currentSample2%=21;
		//Check whether 2nd black line had been crossed
		if(currentSample2 == 20){
			//Swap buffer
			swapBuffer();
			//Turn on green LED
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x8);
			UARTprintf("-------Transmission Start-------\n");
		}
		else{
			//2nd line has been crossed
			//Turn off blue led and turn on green led
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x0);
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x8);
			UARTprintf("-------Remaining Data-------\n");
		}

		int i;
		char str[8];
		//Outputting data to pc
		for(i=0;i<currentSample2;i++){
			sprintf(str,"%.2f",activeBuffer[i]);
			UARTprintf("%s ",str);
		}
		UARTprintf("\n");
		currentSample2 = 0;
		UARTprintf("-------Transmission End-------\n");
		//Turn off green led
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0);
	}
}

void swapBuffer(void){
	if(bufferPtr == 0){
		bufferPtr = 1;
		activeBuffer = Buffer2;
		backBuffer = Buffer1;
	}
	else{
		bufferPtr = 0;
		activeBuffer = Buffer1;
		backBuffer = Buffer2;
	}
}
