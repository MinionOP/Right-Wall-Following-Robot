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



#define NUM_OF_CMD 12						//Number of cmds
#define CMD_TABLE_SIZE (NUM_OF_CMD + 10)	//Cmd Table size
#define BASE_WIDTH 300						//75%

//Function for Console-------------------------------------------------------------------------

void motorStop(void);
void rWheelForward(void);
void rWheelReverse(void);
void lWheelForward(void);
void lWheelReverse(void);
void readFDistSensorToConsole(void);
void readRDistSensorToConsole(void);
void driveStart(void);
void setRDutyCycleFromConsole(void);
void setLDutyCycleFromConsole(void);
void enableTheTimer(void);

//---------------------------------------------------------------------------------------------

void setRDutyCycle(double);
void setLDutyCycle(double);
double readFDistSensor(void);
double readRDistSensor(void);

void readFromConsole(void);
void printTableToConsole(void);
void charArrFromConsole(char* arr, int numOfChar);
void InitHardware(void);

void TimerInt(void);
void UartInt(void);

typedef struct {
	char cmdName[2];
	void (*functionPtr)(void);
}Cmd_type;

typedef struct{
	double Kp, Ki, Kd;
	double setPoint;
	double port;
	double integrator;
	double differentiator, timeConstant;
	double prevError, prevMeasurement;
	double output;
	double iMax, iMin;

}PIDController;

void InitPIDController(PIDController *pid);
void setPIDController(void);
void PIDControllerLoop(void);
void PIDControllerUpdate(PIDController *pid, double distMeasure);


void InitPIDController(PIDController *pid){
	pid->Kp = 2.0;		//Duty cycle 60: kp = 2.0 ki = 0.0 kd = 0.3
	pid->Ki = 0.0;
	pid->Kd = 0.5;
	pid->iMax = 20;
	pid->iMin = -20;
	pid->prevError = 0;
	pid->prevMeasurement = 0;
	pid->output = 0;
	pid->setPoint = 10.0;
}


uint32_t hash(char* cmdName);			//Create key
void InitCmdTable(void);				//Initalize cmd table
void cmdLookUp(char* cmdName);			//Look up cmd
void cmdTableInsert(Cmd_type *toAdd);	//Insert new cmd to cmd table


Cmd_type ListOfCmd[NUM_OF_CMD] = {
	{"P0", &motorStop},
	{"RF", &rWheelForward},
	{"RR", &rWheelReverse},
	{"LF", &lWheelForward},
	{"LR", &lWheelReverse},
	{"R0", &readFDistSensorToConsole},
	{"R1", &readRDistSensorToConsole},
	{"DS", &driveStart},
	{"DR", &setRDutyCycleFromConsole},
	{"DL", &setLDutyCycleFromConsole},
	{"SP", &setPIDController},
	{"ET", &enableTheTimer}
};


Cmd_type* CmdTable[CMD_TABLE_SIZE];

//Create and return a key based on cmdName
uint32_t hash(char* cmdName){
	uint32_t key = (int)cmdName[0] + (int)cmdName[1];
	return key % (CMD_TABLE_SIZE);
}

//Initalize cmd table. Fill all element with NULL
void InitCmdTable(void){
	uint8_t i;
	for(i=0;i<CMD_TABLE_SIZE;i++){CmdTable[i] = NULL;}
	for(i = 0; i < NUM_OF_CMD;i++){cmdTableInsert(&ListOfCmd[i]);}
}

//Insert new cmd.
void cmdTableInsert(Cmd_type *toAdd){
	uint32_t index = hash(toAdd->cmdName); uint32_t i;
	for(i = 0; i<CMD_TABLE_SIZE;i++){
		index = (i+index) % (CMD_TABLE_SIZE);
		if(CmdTable[index] == NULL){
			CmdTable[index] = toAdd;
			break;}
	}
}


//Look up cmd from char[2].
void cmdLookUp(char* cmdName){
	uint32_t index = hash(cmdName); uint32_t i;
	for(i=0;i<CMD_TABLE_SIZE;i++){
		index = (i+index) % (CMD_TABLE_SIZE);
		if((CmdTable[index]->cmdName[0] == cmdName[0]) && (CmdTable[index]->cmdName[1] == cmdName[1])){
			CmdTable[index]->functionPtr();
			return;
		}
	}
	UARTprintf("\nInvalid Command\n");
}

PIDController pidS;
PIDController* pid = &pidS;

void main(void)
{

	InitPIDController(pid);
	InitCmdTable();				//Initialize cmd table
	InitHardware();				//Initialize hardware
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

	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);										//Enable PWM1 Peripheral
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1));								//Wait until PWM1 is ready

	PWMGenConfigure(PWM1_BASE,PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);	//Configure PWM gen 1 and gen 2
	PWMGenConfigure(PWM1_BASE,PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 400);										//Set period: 400 clock ticks
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 400);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, BASE_WIDTH);									//50% duty cycle
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, BASE_WIDTH);


	PWMGenEnable(PWM1_BASE, PWM_GEN_3);							//Right Wheel
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);							//Left Wheel

	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);					//Enable signal


	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);									//Enable GPIOB Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)));							//Wait until GPIOB is ready
	GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);						//Configure pin 4 and pin 5 as ADC

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
	//UARTIntEnable(UART1_BASE,UART_INT_TX|UART_INT_RT);
	UARTEnable(UART1_BASE);
	UARTStdioConfig(1, 115200, SysCtlClockGet());

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);										//Enable ADC0 Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)));							//Wait until ADC0 is ready
	ADCSequenceDisable(GPIO_PORTB_BASE, 0);											//Disable before configuring
	ADCSequenceConfigure(ADC0_BASE, 0,  ADC_TRIGGER_PROCESSOR, 0);					//Configure ADC0. Trigger by processor. Sequence Num = 0.
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH10);						//Ain10 = PB4 Ain11 = PB5
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH11|ADC_CTL_END|ADC_CTL_IE);
    ADCSequenceEnable(ADC0_BASE, 0);												//Enable ADC0

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2); 									//Enable Timer Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)));							//Wait until peripheral is ready
	TimerConfigure(TIMER2_BASE,TIMER_CFG_PERIODIC);									//Configure a full width periodic timer
	TimerLoadSet(TIMER2_BASE,TIMER_A,((SysCtlClockGet())/2)-1);						//Set timer load value.
	TimerIntEnable(TIMER2_BASE,TIMER_TIMA_TIMEOUT);									//Enable timerA interrupt
	TimerEnable(TIMER2_BASE,TIMER_A);												//Enable timer

}

//---------------------------------------------------------------------------------------------
double currentWidthL =  BASE_WIDTH;
double currentWidthR =  BASE_WIDTH;

//Timer interrupt. Interrupt number: 39
void TimerInt(void){
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT); 								//Clear timer interrupt
	Semaphore_post(PIDSem);
}

void enableTheTimer(void){
	UARTprintf("\nEnabling TimerA\n");
	TimerEnable(TIMER2_BASE,TIMER_A);												//Enable timer
	UARTprintf("Drive Start\n");
	PWMGenEnable(PWM1_BASE, PWM_GEN_3);												//Right Wheel
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);												//Left Wheel
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);
}



void PIDControllerLoop(void){
	while(1){
	Semaphore_pend(PIDSem,BIOS_WAIT_FOREVER);
	PIDControllerUpdate(pid, readRDistSensor());
	}
}

void PIDControllerUpdate(PIDController *pid, double distMeasure){
	double error = distMeasure - pid->setPoint;
	if(abs(error) <20){
		double PWMPeriod = PWMGenPeriodGet(PWM1_BASE,PWM_GEN_3);

		pid-> port = pid->Kp * error;
		pid->integrator = pid->integrator + pid->Ki *(error + pid->prevError);
		if(pid->integrator > pid ->iMax){
			pid->integrator = pid->iMax;
		}
		else if(pid->integrator < pid->iMin){
			pid->integrator = pid->iMin;
		}
		pid->differentiator = pid->Kd * (error - pid->prevError);
		pid->prevError = error;
		pid->output = pid->port + pid->integrator + pid->differentiator;

		if(pid->output > (PWMPeriod - BASE_WIDTH)){
			pid->output = PWMPeriod - BASE_WIDTH;
		}

		currentWidthR = BASE_WIDTH - (pid->output * PWMPeriod)/100;
		setRDutyCycle(currentWidthR);
	}
}

void setPIDController(void){
	UARTprintf("\nSetting para for PID (KpKiKd)");
	char kp[3], ki[3], kd[3];
	UARTprintf("\nEnter kp: ");	charArrFromConsole(kp,3);
	UARTprintf("\nEnter ki: ");	charArrFromConsole(ki,3);
	UARTprintf("\nEnter kd: ");	charArrFromConsole(kd,3);

	pid->Kp = atof(kp);
	pid->Ki = atof(ki);
	pid->Kd = atof(kd);
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
//Controling duty cycle of motors

void setRDutyCycle(double widthR){
	currentWidthR = widthR;
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, currentWidthR);					//Set duty cycle for right motor
}
void setLDutyCycle(double widthL){
	currentWidthR = widthL;
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, currentWidthL);					//Set duty cycle for left motor
}

void setRDutyCycleFromConsole(void){
	char dutyCycleArr[4];
	int32_t duty;
	UARTprintf("\nEnter duty cycle for right motor: ");
	charArrFromConsole(dutyCycleArr,4);
	duty = atoi(dutyCycleArr);										//Convert to int
	if(duty > 100){
		UARTprintf("\nInvalid duty cycle\n");
	}
	currentWidthR = (duty * (PWMGenPeriodGet(PWM1_BASE,PWM_GEN_3)))/100;
	setRDutyCycle(currentWidthR);
}

void setLDutyCycleFromConsole(void){
	char dutyCycleArr[4];
	int32_t duty;
	UARTprintf("\nEnter duty cycle for left motor: ");
	charArrFromConsole(dutyCycleArr,4);
	duty = atoi(dutyCycleArr);										//Convert to int
	if(duty > 100){
		UARTprintf("Invalid duty cycle\n");
	}
	currentWidthL = (duty * (PWMGenPeriodGet(PWM1_BASE,PWM_GEN_2)))/100;
	setLDutyCycle(currentWidthL);
}


//---------------------------------------------------------------------------------------------
//Disable PWM
void motorStop(void){
	UARTprintf("\nMotor Stop\n");
	//PWMGenDisable(PWM1_BASE, PWM_GEN_3);	//Right Wheel
	//PWMGenDisable(PWM1_BASE, PWM_GEN_2);	//Left Wheel
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, false);
}
//---------------------------------------------------------------------------------------------
//Start the motor. Enable PWM and enable signal output
void driveStart(void){
	UARTprintf("\nDrive Start\n");
	PWMGenEnable(PWM1_BASE, PWM_GEN_3);		//Right Wheel
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);	//Left Wheel
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);
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

void readFDistSensorToConsole(void){
	uint8_t i;
	double dist;
	//Will take data 50 times. To test
	for(i=0;i<50;i++){
		dist = readFDistSensor();
		printf("Front Distance Sensor: %.2f[cm]\n", dist);	//Print front distance sensor data
	}

}

void readRDistSensorToConsole(void){
	uint8_t i;
	double dist;
	//Will take data 50 times. Can modify
	for(i=0;i<50;i++){
		dist = readRDistSensor();
		printf("Right Distance Sensor: %.2f[cm]\n", dist);	//Print right distance sensor data
	}
}


//---------------------------------------------------------------------------------------------
void readFromConsole(void){
	printTableToConsole();
	UARTprintf("\nEnter Command: ");
	char cmdInput[2];
	charArrFromConsole(cmdInput,2);
	TimerDisable(TIMER2_BASE,TIMER_A);												//Disable Timer
	cmdLookUp(cmdInput);
}


void printTableToConsole(void){
	UARTprintf("Command Table\n");
	UARTprintf("RF: Right Wheel Forward\n");
	UARTprintf("RR: Right Wheel Reverse\n");
	UARTprintf("LF: Left Wheel Forward\n");
	UARTprintf("LR: Left Wheel Reverse\n");
	UARTprintf("R0: Read Front Distance Sensor\n");
	UARTprintf("R1: Read Right Distance Sensor\n");
	UARTprintf("DR: Set Duty Cycle for Right Motor\n");
	UARTprintf("DL: Set Duty Cycle for Left Motor\n");
	UARTprintf("SP: Set PIDController (Kp,Ki,Kd)\n");
	UARTprintf("P0: Motor Brake\n");
	UARTprintf("DS: Drive Start\n");
	UARTprintf("ET: Enable Timer\n");
}


void charArrFromConsole(char* arr, int numOfChar){
	int i;
	for(i=0;i<numOfChar;i++){
		while(!UARTCharsAvail(UART1_BASE));									//Wait until user input char
		arr[i] = UARTCharGetNonBlocking(UART1_BASE);						//Store char into arr
		UARTCharPut(UART1_BASE, arr[i]);									//Echo back to console
		if(arr[i]== 0x0D){													//Break if new line
			UARTprintf("\n");
			break;
		}
	}
}













