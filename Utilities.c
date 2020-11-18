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
#include "Driver.h"
#include "Utilities.h"


//GPIOB pin 4 (Ain 10) = Front Distance Sensor
double readFront(void){
	uint32_t DistSensorADCData[8];
	double distInCm;

	ADCProcessorTrigger(ADC0_BASE, 0);									//Trigger ADC
	while(!ADCIntStatus(ADC0_BASE, 0, false)){}
	ADCIntClear(ADC0_BASE, 0);											//Clear ADC interrupt
	ADCSequenceDataGet(ADC0_BASE, 0, &DistSensorADCData[0]);			//Get data
	distInCm = pow((9011.8/DistSensorADCData[0]),(1/.743));				//Voltage[mV] = 9011.8(distance)^(-0.743)
	return distInCm;
}

//GPIOB pin 5 (Ain 11) = Right Distance Sensor
double readRight(void){
	uint32_t DistSensorADCData[8];
	double distInCm;

	ADCProcessorTrigger(ADC0_BASE, 0);									//Trigger ADC
	while(!ADCIntStatus(ADC0_BASE, 0, false)){}
	ADCIntClear(ADC0_BASE, 0);											//Clear ADC interrupt
	ADCSequenceDataGet(ADC0_BASE, 0, &DistSensorADCData[0]);			//Get data
	distInCm = pow((17427/DistSensorADCData[1]),(1/1.065));				//Voltage[mV] = 17427(distance)^(-1.065)
	return distInCm;
}


//---------------------------------------------------------------------------------------------

uint8_t lightSensor(char colorLine, int currentStatus){
	int counter = 0;
	uint8_t overBlackLine = 0;
    uint8_t overLine = 0;

    int temp;
    //Configure pin B6 as digital output
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);
    //Output high to pin B6
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0x40);
    //Wait 1 mircosecond for capacitor to charge
    SysCtlDelay(100);
    //Change pin B6 from digital output to digital input
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6);
    //Measure the time it takes the capacitor to discharge, until Pin B6 read low.
    while(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6)>0){
    	counter++;
    	//Set max counter to 400 if white
    	/*if(counter >=400 && colorLine == 'w'){
    				break;
    			}
    			//Set max counter to 1500 if black
    			else if(counter >=1500 && colorLine == 'b'){
    				overBlackLine = 1;
    				break;
    			}*/
    }
    //Print value to bluetooth
    temp = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6);
    UARTprintf("%d\n",counter);

	switch(colorLine){
	//White crosslines
	case 'w':{
		if(counter <200){
			UARTprintf("Crossed White Line\n");
            SysCtlDelay(SysCtlClockGet());
			//PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, false);
			overLine = 1;
            SysCtlDelay(SysCtlClockGet()*5);
			break;
		}
	}
	//Black crosslines
	case 'b':{
		if(overBlackLine){
			UARTprintf("Crossed Black Line\n");
			if(currentStatus == 1){
	            SysCtlDelay(SysCtlClockGet());
				PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, false);
	            SysCtlDelay(SysCtlClockGet()*5);
			}
			overLine = 1;
			break;
		}
	}
	}



    return overLine;
}



//---------------------------------------------------------------------------------------------
//Controling duty cycle of motors
void wheelDuty(double lDuty, double rDuty){
	uint32_t currentWidthL = (lDuty/100) * PERIOD;
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, currentWidthL);					//Set duty cycle for left motor

	uint32_t currentWidthR = (rDuty/100) * PERIOD;
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, currentWidthR);					//Set duty cycle for right motor
}

//---------------------------------------------------------------------------------------------
//M1PWM2 Module 1 PWM Generator 1	Pin A6 Right
//PWM_OUT_1_BIT Left
//PWM_OUT_2_BIT Right


//GPIOD pin 0 = Left Wheel Dir
//GPIOD pin 1 = Left Wheel PWM (M1PWM1)

//GPIOD pin 2 = Right Wheel Dir
//GPIOA pin 6 = Right Wheel PWM (M1PWM2)



//wheelNum(0) = left, (1) = right, (2) = both
//dir(0) = reverse, dir(1) = forward
void wheelDir(uint32_t wheelNum, uint32_t dir){
	uint32_t bothWheelStatus = 0;
	if(wheelNum == 2){
		bothWheelStatus = 1;
		wheelNum = 0;
	}

	switch(wheelNum){
	case 0:{
		if(dir == 1){
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x1);
		}
		else{
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x0);
		}
		if(!bothWheelStatus){
			break;
		}
	}
	case 1:{
		if(dir == 1){
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x4);
		}
		else{
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x0);
		}
		break;
	}
	}
}

//---------------------------------------------------------------------------------------------
//wheelNum(0) = left, (1) = right, (2) = both
void wheelPower(uint32_t wheelNum, char* power){
	uint8_t status;
	if(strcmp(power, "on") == 0){
		status = 1;
	}
	else{
		status = 0;
	}
	switch(wheelNum){
	case 0:{
		PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, status);
	}
	case 1:{
		PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, status);
	}
	case 2:{
		PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT, status);
	}
}
}

//---------------------------------------------------------------------------------------------
void rightTurn(void){
	//Attempting right turn, set rightState to true

	//rightState = 1;

	//Reset right motor duty cycle
	wheelDuty(1,BASE_DUTY);
	//Right wheel reverse for a short amount of time
	wheelDir(1,0);
//	delay(37);
	//Return right wheel direction back to forward
	wheelDir(1,1);
	//Turn both wheels off. PIDControllerLoop will check if there's a right wall
	wheelPower(2,"off");
}


void uTurn(void){
	wheelDuty(1,BASE_DUTY);
	wheelDir(0,0);
	//delay(75);
	wheelDir(0,1);
	wheelPower(2,"off");
}

//---------------------------------------------------------------------------------------------


