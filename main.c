//Team 9: Charle Nguyen, Edward Sotelo, Josh McHenry
//Milestone 8

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


#define BASE_WIDTH 300						//75% duty cycle

void setRDutyCycle(double);
void setLDutyCycle(double);
double readFDistSensor(void);
double readRDistSensor(void);
void rWheelForward(void);
void rWheelReverse(void);
void lWheelForward(void);
void lWheelReverse(void);
void LightSensor(void);

void delay(uint32_t wait);
void rightTurn(void);
void uTurn(void);

void InitHardware(void);

void TimerInt(void);						//Timer interrupt
void PIDControllerLoop(void);				//Task



PIDController pidS;
PIDController* pid = &pidS;

void main(void)
{
	InitPIDController(pid);
	InitHardware();							//Initialize hardware
	BIOS_start();

}


void InitHardware(void){
	//Set Clock to 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);									//Enable GPIOF Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)));							//Wait until GPIOF is ready
	GPIOPinConfigure(GPIO_PF1_M1PWM5);												//Configure GPIOF_pin 1 and pin 2 for PWM
	GPIOPinConfigure(GPIO_PF2_M1PWM6);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1| GPIO_PIN_2);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_4);				//Configure Pin 3 and Pin 4 as output. To control motor phase
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_4, 0x18);					//Output high for pin 3 and 4

//---------------------------------------------------------------------------------------------
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);										//Enable PWM1 Peripheral
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1));								//Wait until PWM1 is ready

	PWMGenConfigure(PWM1_BASE,PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);	//Configure PWM gen 1 and gen 2
	PWMGenConfigure(PWM1_BASE,PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 400);										//Set period: 400 clock ticks
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 400);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, BASE_WIDTH);								//50% duty cycle
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, BASE_WIDTH);

	PWMGenEnable(PWM1_BASE, PWM_GEN_3);												//Right Wheel
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);												//Left Wheel
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);					//Enable output

//---------------------------------------------------------------------------------------------
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);									//Enable GPIOB Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)));							//Wait until GPIOB is ready

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);									//Enable GPIOB Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)));							//Wait until GPIOB is ready
	GPIOPinConfigure(GPIO_PB0_U1RX);												//Configure GPIOB0 as receiver
	GPIOPinConfigure(GPIO_PB1_U1TX);												//Configure GPIOB1 as transmitter
	GPIOPinTypeUART(GPIO_PORTB_BASE , GPIO_PIN_0 | GPIO_PIN_1);						//Configure as UART pin

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);									//Enable UART Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_UART1)));							//Wait until UART1 is ready

	UARTDisable(UART1_BASE);
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200, 						//Configure UART
						UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
	UARTEnable(UART1_BASE);
	UARTStdioConfig(1, 115200, SysCtlClockGet());

//---------------------------------------------------------------------------------------------
	GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);						//Configure pin 4 and pin 5 as ADC
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);										//Enable ADC0 Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)));							//Wait until ADC0 is ready
	ADCSequenceDisable(GPIO_PORTB_BASE, 0);											//Disable before configuring
	ADCSequenceConfigure(ADC0_BASE, 0,  ADC_TRIGGER_PROCESSOR, 0);					//Configure ADC0. Trigger by processor. Sequence Num = 0.
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH10);						//Ain10 = PB4 Ain11 = PB5
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH11|ADC_CTL_END|ADC_CTL_IE);
    ADCSequenceEnable(ADC0_BASE, 0);												//Enable ADC0

//---------------------------------------------------------------------------------------------
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2); 									//Enable Timer Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)));							//Wait until peripheral is ready
	TimerConfigure(TIMER2_BASE,TIMER_CFG_PERIODIC);									//Configure a full width periodic timer
	TimerLoadSet(TIMER2_BASE,TIMER_A,((SysCtlClockGet())/100)-1);					//Set timer load value.
	TimerIntEnable(TIMER2_BASE,TIMER_TIMA_TIMEOUT);									//Enable timerA interrupt
	TimerEnable(TIMER2_BASE,TIMER_A);												//Enable timer

}

//---------------------------------------------------------------------------------------------
//Current width for left and right motor
double currentWidthL =  BASE_WIDTH;
double currentWidthR =  BASE_WIDTH;

//If robot is currently attempting to turn right
uint32_t rightState = 0;

//If delay has been called
uint8_t delayStatus = 0;

//Will be true if robot crosses over line
uint8_t overLine = 0;

//Black or white crossline
char colorLine = 'b';			//w = white and b = black
volatile uint32_t time = 0;


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
	}
}


//---------------------------------------------------------------------------------------------
void delay(uint32_t wait){			//1ms
	delayStatus = 1;				//Set delayStatus = 1, so interrupt will not post semaphore
	uint32_t initial = time;
	while(time - initial <wait);
	delayStatus= 0;
}

void rightTurn(void){
	//Attempting right turn, set rightState to true
	rightState = 1;
	//Reset right motor duty cycle
	setRDutyCycle(BASE_WIDTH);
	//Right wheel reverse for a short amount of time
	rWheelReverse();
	delay(37);
	//Return right wheel direction back to forward
	rWheelForward();
	//Turn both wheels off. PIDControllerLoop will check if there's a right wall
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, false);

}

void uTurn(void){
	setRDutyCycle(BASE_WIDTH);
	lWheelReverse();
	delay(75);
	lWheelForward();
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, false);
}

//---------------------------------------------------------------------------------------------
//Task Handle: PIDHandle
void PIDControllerLoop(void){
	while(1){
		Semaphore_pend(PIDSem,BIOS_WAIT_FOREVER);										//Pend semaphore
		double distRMeasured = readRDistSensor();
		double distFMeasured = readFDistSensor();
		//If robot have just finished rotating right
		if(rightState == 1){
			//Check if sensor can detect right wall. If it does not, move a short amount of distance forward
			if(distRMeasured > 15){
			PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);
			delay(25);
			PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, false);
			}
			//Right wall detect, set rightState to 0.
			else{
				PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);
				rightState = 0;
			}
		}
		//If robot does not detect right wall, rotate right
		if(distRMeasured > 15 && rightState == 0){	//20
			PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);
			delay(40);
			rightTurn();
		}
		//If dead end is detected, make a u turn
		else if(distRMeasured <15 && distFMeasured < 15){
			PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);
			uTurn();
		}
		//Drive forward using pid
		else if(distRMeasured > 0 && rightState == 0){
			PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);
			currentWidthR = PIDControllerUpdate(pid, readRDistSensor(), 400, BASE_WIDTH);
			setRDutyCycle(currentWidthR);													//Set new duty cycle
		}
	}
}

//---------------------------------------------------------------------------------------------
void LightSensor(void){
	while(1){
		Semaphore_pend(LightSem, BIOS_WAIT_FOREVER);										//Pend semaphore
		int counter = 0;
		uint8_t overBlackLine = 0;
		//Configure pin B6 as digital output
		GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);
		//Output high to pin B6
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0x40);
		//Wait 1 mircosecond for capacitor to charge
		SysCtlDelay((SysCtlClockGet()/100000)-1);
		//Change pin B6 from digital output to digital input
		GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6);
		//Measure the time it takes the capacitor to discharge, until Pin B6 read low.
		while(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6)){
			counter++;
			//Set max counter to 400 if white
			if(counter >=400 && colorLine == 'w'){
				break;
			}
			//Set max counter to 1500 if black
			else if(counter >=1500 && colorLine == 'b'){
				overBlackLine = 1;
				break;
			}
		}
		//Print value to bluetooth
		UARTprintf("%d\n",counter);

		switch(colorLine){
		//White crosslines
		case 'w':{
			if(counter <200){
				UARTprintf("Crossed White Line. END----------\n");
				delay(100);
				PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, false);
				overLine = 1;
				//delay(1000);
				break;
			}
		}
		//Black crosslines
		case 'b':{
			if(overBlackLine){
				UARTprintf("Crossed Black Line. END----------\n");
				delay(100);
				PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, false);
				overLine = 1;
				//delay(1000);
				break;
			}
		}
		}

	}
}

//---------------------------------------------------------------------------------------------
//Controling duty cycle of motors

void setRDutyCycle(double widthR){
	currentWidthR = widthR;
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, currentWidthR);					//Set duty cycle for right motor
}
void setLDutyCycle(double widthL){
	currentWidthL = widthL;
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, currentWidthL);					//Set duty cycle for left motor
}


//---------------------------------------------------------------------------------------------
//GPIOF pin 4 = Right Wheel Phase
//GPIOF pin 2 = Right Wheel PWM (M1PWM6)
void rWheelForward(void){
	UARTprintf("\nRight wheel forward\n");
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x10);
}
void rWheelReverse(void){
	UARTprintf("\nRight wheel reverse\n");
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x0);
}

//---------------------------------------------------------------------------------------------

//GPIOF pin 3 = Left Wheel Phase
//GPIOF pin 1 = Left Wheel PWM (M1PWM5)
void lWheelForward(void){
	UARTprintf("\nLeft wheel forward\n");
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x8);
}
void lWheelReverse(void){
	UARTprintf("\nLeft wheel reverse\n");
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0);
}

//---------------------------------------------------------------------------------------------
//GPIOB pin 4 (Ain 10) = Front Distance Sensor
double readFDistSensor(void){
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
double readRDistSensor(void){
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











