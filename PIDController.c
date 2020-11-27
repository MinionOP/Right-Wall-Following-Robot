#include "PIDController.h"


void InitPID(PIDController *pid, double _PWMPeriod, double _baseWidth){
	pid->Kp = 10.0;
	pid->Ki = 0.1;
	pid->Kd = 25.0;	//8
	pid->iMax = 3;
	pid->iMin = -3;
	pid->prevError = 0;
	pid->prevMeasurement = 0;
	pid->Correction = 0;
	pid->targetRDist = 8.5;	//10cm
    pid->baseWidth = _baseWidth;
    pid->PWMPeriod = _PWMPeriod;
}

double PIDUpdate(PIDController *pid, double distMeasure){
	double error = distMeasure - pid->targetRDist;
	if(abs(error) > 30){
		error = 15;
	}
	//if(abs(error) <15){
	pid-> Port = pid->Kp * error;
	pid->Integral = pid->Ki *(error + pid->prevError);
	if(pid->Integral > pid ->iMax){
		pid->Integral = pid->iMax;
	}
	else if(pid->Integral < pid->iMin){
		pid->Integral = pid->iMin;
	}
	pid->Derivative = pid->Kd * (error - pid->prevError);
	pid->prevError = error;
	pid->Correction = pid->Port + pid->Integral + pid->Derivative;

	/*if(pid->Correction > pid->baseWidth){
		//Min 10 duty cycle
		pid->Correction = pid->baseWidth-40;
	}*/
	//}
/*
	if(pid->Correction > pid->PWMPeriod){
		pid->Correction = pid->PWMPeriod;
	}
	else if(pid->Correction < 0){
		pid->Correction = 0;
	}*/

	double newDutyCycle = ((pid->baseWidth - pid->Correction)/ (pid->PWMPeriod))*100;
	if(newDutyCycle > 100){
		newDutyCycle = 90;
	}
	else if(newDutyCycle < 10){
		newDutyCycle = 10;
	}

	return newDutyCycle;
}
void setPIDRight(PIDController* pid){
	pid->Kp = 25.0;
	pid->Ki = 0.1;
	pid->Kd = 60.0;	//8
	pid->targetRDist = 5.0;	//10cm
	pid->prevError = 0;
}

void setPIDForward(PIDController* pid){
	pid->Kp = 10.0;
	pid->Ki = 0.1;
	pid->Kd = 25.0;	//8
	pid->targetRDist = 8.5;	//10cm
	pid->prevError = 0;

}



double getPrevError(PIDController *pid){
	return pid->prevError;
}


