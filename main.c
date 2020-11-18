//Team 9: Charle Nguyen, Edward Sotelo, Josh McHenry
//Milestone 9

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


#define BASE_WIDTH 300
#define PERIOD 400
#define BASE_DUTY (BASE_WIDTH/PERIOD) * 100		//75% duty cycle

enum state {stopS, forwardS, rightS, leftS};

//Task
void lightSensorTask(void);
void PIDControllerTask(void);

//Timer interrupt
void TimerInt(void);
void delay(uint32_t wait);


void AcquireDataTask(void);
void TxDataTask(void);
void swapBuffer(void);

uint8_t lightSensor(char colorLine, int currentStatus);


PIDController pidS;
PIDController* pid = &pidS;

void main(void)
{
	InitPID(pid, PERIOD, BASE_WIDTH);
	InitHardware();
	BIOS_start();

}

//---------------------------------------------------------------------------------------------
//If robot is currently attempting to turn right
uint32_t rightState = 0;

//If delay has been called
uint8_t delayStatus = 0;

//Will be true if robot crosses over line
uint8_t overLine = 0;

//Black or white crossline
char colorLine = 'b';			//w = white and b = black
volatile uint32_t time = 0;

uint8_t dataCollectStatus = 0;

//Timer interrupt. Interrupt number: 39
void TimerInt(void){
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT); 									//Clear timer interrupt
	//increment time
	time = time+1;
	//Check if delay function is currently running or if the line has been crossed

	if(delayStatus == 0 && overLine == 0){
		time = time%1000;
		//Post to lightSensor every 15ms
		if(time%15 == 0){
			Semaphore_post(LightSem);
		}
		//Post to PIDController every 50ms
		if(time%50 == 0){
			Semaphore_post(PIDSem);														//Post semaphore, pend semaphore in PIDControllerLoop
		}
		if(time%100 == 0 && dataCollectStatus == 1){
			//Semaphore_post(AcquireDataSem);
		}
	}
}


//---------------------------------------------------------------------------------------------
void delay(uint32_t wait){	//1ms
	delayStatus = 1;				//Set delayStatus = 1, so interrupt will not post semaphore
	uint32_t initial = time;
	while(time - initial <wait);
	delayStatus= 0;
}


//---------------------------------------------------------------------------------------------
//Task Handle: PIDHandle
void PIDTask(void){
	while(1){
		Semaphore_pend(PIDSem,BIOS_WAIT_FOREVER);										//Pend semaphore
		int Correction = PIDUpdate(pid, readRight());
		wheelDuty(BASE_DUTY, Correction);

	}
}

//---------------------------------------------------------------------------------------------
void lightSensorTask(void){
	while(1){
		Semaphore_pend(LightSem, BIOS_WAIT_FOREVER);										//Pend semaphore
		uint8_t temp = 0;
		temp = lightSensor(colorLine, dataCollectStatus);
		if(dataCollectStatus != temp){
			dataCollectStatus = 1;
		}
		else{
			dataCollectStatus = 0;
		}
	}
}

//---------------------------------------------------------------------------------------------
double Buffer1[20];
double Buffer2[20];

double *activeBuffer = Buffer1;
double *backBuffer = Buffer2;
uint8_t bufferPtr = 0;

int currentSample = 0;

void AcquireDataTask(void){
	while(1){
		Semaphore_pend(AcquireDataSem,BIOS_WAIT_FOREVER);
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x8);
		backBuffer[currentSample] = getPrevError(pid);
		currentSample++;
		if(currentSample == 20){
			currentSample = 0;
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0);
			Semaphore_post(TxDataSem);
		}
	}
}

void TxDataTask(void){
	while(1){
		Semaphore_pend(TxDataSem,BIOS_WAIT_FOREVER);
		swapBuffer();
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x4);
		UARTprintf("-------Transmission Start-------\n");
		int i;
		char str[8];
		for(i=0;i<20;i++){
			sprintf(str,"%.2f",activeBuffer[i]);
			UARTprintf("%s\n",str);

		}
		UARTprintf("-------Transmission End-------\n");
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x0);
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






