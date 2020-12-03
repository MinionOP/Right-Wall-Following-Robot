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

uint32_t lightSensor(char colorLine, int currentStatus){
	int counter = 0;
	uint8_t overBlackLine = 0;
	uint8_t status = 0;
	//Configure pin E0 as digital output
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
	//Output high to pin B6
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x2);
	//Wait 1 mircosecond for capacitor to charge
	SysCtlDelay((SysCtlClockGet()/100000)-1);
	//Change pin E0 from digital output to digital input
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
	//Measure the time it takes the capacitor to discharge, until Pin B6 read low.
	while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1)>0){
		counter++;
		//Set max counter
		if(counter >=2000 && colorLine == 'b'){
			overBlackLine = 1;
			break;
		}
	}
	if(overBlackLine){
		UARTprintf("Crossed Black Line\n");
		status = 1;
	}
	return status;
}

//---------------------------------------------------------------------------------------------
//Controling duty cycle of motors
void wheelDuty(double lDuty, double rDuty){
	//Set duty cycle for left motor
	double currentWidthL = (lDuty/100) * PERIOD;
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, currentWidthL);

	//Set duty cycle for right motor
	double currentWidthR = (rDuty/100) * PERIOD;
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, currentWidthR);
}

//---------------------------------------------------------------------------------------------
//PWM_OUT_1_BIT Left
//GPIOD pin 0 = Left Wheel Dir
//GPIOD pin 1 = Left Wheel PWM (M1PWM1)

//GPIOD pin 2 = Right Wheel Dir
//GPIOA pin 6 = Right Wheel PWM (M1PWM2)
//PWM_OUT_2_BIT Right

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
