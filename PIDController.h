#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

typedef struct{
	double Kp, Ki, Kd;
	double targetRDist;
	double Port;
	double Integral;
	double iMax;
	double iMin;
	double Derivative;
	double prevError;
	double prevMeasurement;
	double Correction;
}PIDController;

//Duty cycle 60: kp = 2.0 ki = 0.0 kd = 0.3
//Duty cycle 70: kp = 2.0 ki = 0.0 kd = 0.5
//Duty cycle 85: kp = .5., ki = .1 kd = 5 Works


void InitPIDController(PIDController *pid){
	pid->Kp = 0.5;
	pid->Ki = 0.1;
	pid->Kd = 5;
	pid->iMax = 3;
	pid->iMin = -3;
	pid->prevError = 0;
	pid->prevMeasurement = 0;
	pid->Correction = 0;
	pid->targetRDist = 9.0;	//10cm
}

double PIDControllerUpdate(PIDController *pid, double distMeasure, double PWMPeriod, double BASE_WIDTH){
	double error = distMeasure - pid->targetRDist;
	if(abs(error) <15){
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

		if(pid->Correction > (PWMPeriod - BASE_WIDTH)){
			pid->Correction = PWMPeriod - BASE_WIDTH;
		}
	}
	return BASE_WIDTH - (pid->Correction * PWMPeriod)/100;

}

#endif /* PIDCONTROLLER_H_ */
