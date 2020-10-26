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



#define NUM_OF_CMD 10                       //Number of cmds
#define CMD_TABLE_SIZE (NUM_OF_CMD + 10)    //Cmd Table size


void motorStop(void);
void rWheelForward(void);
void rWheelReverse(void);
void lWheelForward(void);
void lWheelReverse(void);
void readFDistSensor(void);
void readRDistSensor(void);
void driveStart(void);
void setRDutyCycle(void);
void setLDutyCycle(void);
void followWall(void);

void readFromConsole(void);
void printTableToConsole(void);
void InitHardware(void);

typedef struct {
    char cmdName[2];
    void (*functionPtr)(void);
}Cmd_type;

uint32_t hash(char* cmdName);           //Create key
void InitCmdTable(void);                //Initalize cmd table
void cmdLookUp(char* cmdName);          //Look up cmd
void cmdTableInsert(Cmd_type *toAdd);   //Insert new cmd to cmd table


Cmd_type ListOfCmd[NUM_OF_CMD] = {
    {"P0", &motorStop},
    {"RF", &rWheelForward},
    {"RR", &rWheelReverse},
    {"LF", &lWheelForward},
    {"LR", &lWheelReverse},
    {"R0", &readFDistSensor},
    {"R1", &readRDistSensor},
    {"DS", &driveStart},
    {"DR", &setRDutyCycle},
    {"DL", &setLDutyCycle},


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
    System_printf("Invalid Command\n");
    System_flush();
}

//FOR WALL FOLLOW
int WF_Right = 0;
int WF_Front = 0;
int Right_Status = 0;
int Front_Status = 0;

float Kp = 13;              // THESE ARE RANDOM GUESSES
float Ki = 0.00002;         // THESE ARE RANDOM GUESSES
float Kd = 2;               // THESE ARE RANDOM GUESSES
float Error = 0;
int32_t targetFDist = 30;  // THESE ARE RANDOM GUESSES
int32_t targetRDist = 14;  // THESE ARE RANDOM GUESSES
int32_t RightTurnSpeed = 0;
int32_t LeftTurnSpeed = 0;
float Correction = 0;
float Integral = 0;
float Derivative = 0;
float LastError = 0;

void main(void)
{
    InitCmdTable();             //Initialize cmd table
    InitHardware();             //Initialize hardware
    BIOS_start();
    followWall();
}

void InitHardware(void){
    //Set Clock to 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                                    //Enable GPIOF Peripheral
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)));                           //Wait until GPIOF is ready
    GPIOPinConfigure(GPIO_PF1_M1PWM5);                                              //Configure GPIOF_pin 1 and pin 2 for PWM
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1| GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_4);                //Configure Pin 3 and Pin 4 as output. To control motor phase
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_4, 0x18);                   //Output high for pin 3 and 4

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);                                     //Enable PWM1 Peripheral
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1));                              //Wait until PWM1 is ready

    PWMGenConfigure(PWM1_BASE,PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); //Configure PWM gen 1 and gen 2
    PWMGenConfigure(PWM1_BASE,PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 400);                                     //Set period: 400 clock ticks
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 400);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 200);                                    //50% duty cycle
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 200);
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);                 //Enable signal


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);                                    //Enable GPIOB Peripheral
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)));                           //Wait until GPIOB is ready
    GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);                       //Configure pin 4 and pin 5 as ADC

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);                                    //Enable GPIOB Peripheral
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)));                           //Wait until GPIOB is ready
    GPIOPinConfigure(GPIO_PB0_U1RX);                                                //Configure GPIOB0 as receiver
    GPIOPinConfigure(GPIO_PB1_U1TX);                                                //Configure GPIOB1 as transmitter
    GPIOPinTypeUART(GPIO_PORTB_BASE , GPIO_PIN_0 | GPIO_PIN_1);                     //Configure as UART pin

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);                                    //Enable UART Peripheral
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_UART1)));                           //Wait until UART1 is ready


    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,                       //Configure UART
                        UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
    UARTEnable(UART1_BASE);
    UARTStdioConfig(1, 115200, SysCtlClockGet());


    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                                     //Enable ADC0 Peripheral
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)));                            //Wait until ADC0 is ready
    ADCSequenceDisable(GPIO_PORTB_BASE, 0);                                         //Disable before configuring
    ADCSequenceConfigure(ADC0_BASE, 0,  ADC_TRIGGER_PROCESSOR, 0);                  //Configure ADC0. Trigger by processor. Sequence Num = 0.
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH10);                        //Ain10 = PB4 Ain11 = PB5
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH11|ADC_CTL_END|ADC_CTL_IE);
    ADCSequenceEnable(ADC0_BASE, 0);                                                //Enable ADC0
}

//---------------------------------------------------------------------------------------------

//GPIOF pin 4 = Right Wheel Phase
//GPIOF pin 2 = Right Wheel PWM (M1PWM6)
void rWheelForward(void){
    printf("Right wheel forward\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x10);
}
void rWheelReverse(void){
    printf("Right wheel reverse\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x0);
}

//---------------------------------------------------------------------------------------------

//GPIOF pin 3 = Left Wheel Phase
//GPIOF pin 1 = Left Wheel PWM (M1PWM5)
void lWheelForward(void){
    printf("Left wheel forward\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x8);
}
void lWheelReverse(void){
    printf("Left wheel reverse\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0);
}

//---------------------------------------------------------------------------------------------
//Controling duty cycle of motors
uint32_t currentWidthL =  200;
uint32_t currentWidthR =  200;

void setRDutyCycle(int32_t dutyCycle){
    char dutyCycleArr[4];
    int8_t i;
    int32_t dutyCycleInt;
    
    dutyCycleInt = dutyCycle;                                      //Convert to int
    if(dutyCycleInt > 100){
        printf("Invalid duty cycle\n");
    }
    currentWidthR = (dutyCycleInt * (PWMGenPeriodGet(PWM1_BASE,PWM_GEN_3)))/100;
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, currentWidthR);                  //Set duty cycle for right motor
    printf("Right Motor Duty Cycle: %d", currentWidthR);


}
void setLDutyCycle(int32_t dutyCycle){
    char dutyCycleArr[4];
    int8_t i;
    int32_t dutyCycleInt;

    dutyCycleInt = dutyCycle;                                      //Convert to int
    if(dutyCycleInt > 100){
        printf("Invalid duty cycle\n");
    }
    currentWidthL = (dutyCycleInt * (PWMGenPeriodGet(PWM1_BASE,PWM_GEN_2)))/100;
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, currentWidthL);                  //Set duty cycle for left motor
    printf("Left Motor Duty Cycle: %d", currentWidthL);
}

//---------------------------------------------------------------------------------------------
//Disable PWM
void motorStop(void){
    System_printf("Motor Stop\n");
    System_flush();
    //PWMGenDisable(PWM1_BASE, PWM_GEN_3);  //Right Wheel
    //PWMGenDisable(PWM1_BASE, PWM_GEN_2);  //Left Wheel
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, false);
}
//---------------------------------------------------------------------------------------------
//Start the motor. Enable PWM and enable signal output
void driveStart(void){
    System_printf("Drive Start\n");
    System_flush();
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);     //Right Wheel
    PWMGenEnable(PWM1_BASE, PWM_GEN_2); //Left Wheel
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);
}
//---------------------------------------------------------------------------------------------
//GPIOB pin 4 (Ain 10) = Front Distance Sensor
double readFDistSensor(void){
    uint32_t DistSensorADCData[8];
    uint8_t i;
    double distInCm;

    //Will take data 100 times. Can modify
    
    ADCProcessorTrigger(ADC0_BASE, 0);                                  //Trigger ADC
    while(!ADCIntStatus(ADC0_BASE, 0, false)){}
    ADCIntClear(ADC0_BASE, 0);                                          //Clear ADC interrupt
    ADCSequenceDataGet(ADC0_BASE, 0, &DistSensorADCData[0]);            //Get data
    distInCm = pow((9011.8/DistSensorADCData[0]),(1/.743));             //Voltage[mV] = 9011.8(distance)^(-0.743)
    printf("Front Distance Sensor: %d[mV]  %.2f[cm]\n", DistSensorADCData[0], distInCm);    //Print front distance sensor data
    return distInCm;
}

void followWall(void){
    while((readRDistSensor() < MAXDISTANCE) && (readFDistSensor() > MINDISTANCE)){
        Front_Status = readFDistSensor();
        if(Front_Status >= targetFDist){
            continueWall();
        }
        else
            break;
    }
    motorStop();
}

void continueWall(void){
    WF_Right = readRDistSensor();
    Error = (WF_Right - targetRDist);
    Integral = (Error + Integral);
    Derivative = (Error - LastError);

    Correction = Kp * Error + Kd * Derivative + Ki * Integral;

    LeftTurnSpeed = 90 - (int32_t)Correction;      //MORE RANDOM GUESSES
    RightTurnSpeed = 90 + (int32_t)Correction;     //MORE RANDOM GUESSES

    setRDutyCycle(RightTurnSpeed);
    setLDutyCycle(LeftTurnSpeed);
    lWheelForward();
    rWheelForward();

    LastError = Error;
}

//GPIOB pin 5 (Ain 11) = Right Distance Sensor
double readRDistSensor(void){
    uint32_t DistSensorADCData[8];
    uint8_t i;
    double distInCm;

    //Will take data 100 times. Can modify
    
    ADCProcessorTrigger(ADC0_BASE, 0);                                  //Trigger ADC
    while(!ADCIntStatus(ADC0_BASE, 0, false)){}
    ADCIntClear(ADC0_BASE, 0);                                          //Clear ADC interrupt
    ADCSequenceDataGet(ADC0_BASE, 0, &DistSensorADCData[0]);            //Get data
    distInCm = pow((17427/DistSensorADCData[1]),(1/1.065));             //Voltage[mV] = 17427(distance)^(-1.065)
    printf("Right Distance Sensor: %d[mV]  %.2f[cm]\n", DistSensorADCData[1], distInCm);    //Print right distance sensor data
    return distInCm;
}

//---------------------------------------------------------------------------------------------
//readFromConsole is idle
void readFromConsole(void){
    printTableToConsole();
    UARTprintf("\nEnter Command: ");
    char cmdInput[2];
    int8_t i;
    for(i=0;i<2;i++){
        while(!UARTCharsAvail(UART1_BASE));                                 //Wait until user input char
        cmdInput[i] = UARTCharGetNonBlocking(UART1_BASE);                   //Store char into cmdInput
        UARTCharPut(UART1_BASE, cmdInput[i]);                               //Echo back to console
    }
    cmdLookUp(cmdInput);                                                    //After two char input, look up cmd
}


void printTableToConsole(void){
    UARTprintf("Command Table\n");
    UARTprintf("P0: Motor Brake\n");
    UARTprintf("RF: Right Wheel Forward\n");
    UARTprintf("RR: Right Wheel Reverse\n");
    UARTprintf("LF: Left Wheel Forward\n");
    UARTprintf("LR: Left Wheel Reverse\n");
    UARTprintf("R0: Read Front Distance Sensor\n");
    UARTprintf("R1: Read Right Distance Sensor\n");
    UARTprintf("DS: Drive Start\n");
    UARTprintf("DR: Set Duty Cycle for Right Motor\n");
    UARTprintf("DL: Set Duty Cycle for Left Motor\n");

}
