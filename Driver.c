/*
	Configure Motors, Bluetooth, LEDs, light sensor, and Distance sensors
*/

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


void InitHardware(void){
	InitBluetooth();
	UARTprintf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n-------------------------------------\n");
	UARTprintf("Bluetooth Connected\n");
	InitMotors();
	UARTprintf("Motor Initialized\n");
	InitAnalog();
	UARTprintf("ADC Initialized\n");
	InitTimer();
	UARTprintf("Timer Initialized\n");
	UARTprintf("------------------------------------------------\n");
}

void InitMotors(void){
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);									//Enable GPIOD Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)));							//Wait until GPIOD is ready
	GPIOPinConfigure(GPIO_PD1_M1PWM1);												//Configure GPIOD_pin 1 for PWM
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2);				//Configure Pin 0 and Pin 2 as output. To control motor phase
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2, 0x5);					//Output high for pin 3 and 4
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);									//Enable GPIOA Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)));							//Wait until GPIOA is ready
	GPIOPinConfigure(GPIO_PA6_M1PWM2);												//Configure GPIOA_pin 6 for PWM
	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);										//Enable PWM1 Peripheral
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1));								//Wait until PWM1 is ready
	PWMGenConfigure(PWM1_BASE,PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);	//Configure PWM gen 0 and gen 1
	PWMGenConfigure(PWM1_BASE,PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, 400);										//Set period: 400 clock ticks
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 400);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, BASE_WIDTH);								//Set duty cycle
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, BASE_WIDTH);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);												//Left Wheel
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);												//Right Wheel
	PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT, true);					//Enable signal
}

void InitBluetooth(void){
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
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
}

void InitAnalog(void){
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);									//Enable GPIOF Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)));							//Wait until GPIOF is ready
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);		//LED
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);									//Enable GPIOE Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)));							//Wait until GPIOE is ready
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_1);								//Light sensor output
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);									//Enable GPIOB Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)));							//Wait until GPIOB is ready
	GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);						//Configure pin 4 and pin 5 as ADC
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);										//Enable ADC0 Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)));							//Wait until ADC0 is ready
	ADCSequenceDisable(GPIO_PORTB_BASE, 0);											//Disable before configuring
	ADCSequenceConfigure(ADC0_BASE, 0,  ADC_TRIGGER_PROCESSOR, 0);					//Configure ADC0. Trigger by processor. Sequence Num = 0.
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH10);						//Ain10 = PB4 Ain11 = PB5
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH11|ADC_CTL_END|ADC_CTL_IE);
	ADCSequenceEnable(ADC0_BASE, 0);												//Enable ADC0
}

void InitTimer(void){
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2); 									//Enable Timer Peripheral
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)));							//Wait until peripheral is ready
	TimerConfigure(TIMER2_BASE,TIMER_CFG_PERIODIC);									//Configure a full width periodic timer
	TimerLoadSet(TIMER2_BASE,TIMER_A,((SysCtlClockGet())/1000)-1);					//Set timer load value.
	TimerIntEnable(TIMER2_BASE,TIMER_TIMA_TIMEOUT);									//Enable timerA interrupt
	TimerEnable(TIMER2_BASE,TIMER_A);												//Enable timer
}
