#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_


//Duty cycle 60: kp = 2.0 ki = 0.0 kd = 0.3
//Duty cycle 70: kp = 2.0 ki = 0.0 kd = 0.5
//Duty cycle 85: kp = .5., ki = .1 kd = 5 Works
//Duty cycle 75: kp = 1.5 ki = 0.1 kd = 6.0
//Duty cycle 65: kp = .5 ki = .1 kd = 5

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
	int baseWidth;
	int PWMPeriod;
}PIDController;



void InitPIDController(PIDController *pid, double _PWMPeriod, double _baseWidth){
	pid->Kp = 1.0;
	pid->Ki = 0.1;
	pid->Kd = 5.0;
	pid->iMax = 3;
	pid->iMin = -3;
	pid->prevError = 0;
	pid->prevMeasurement = 0;
	pid->Correction = 0;
	pid->targetRDist = 8.5;	//10cm
    pid->baseWidth = _baseWidth;
    pid->PWMPeriod = _PWMPeriod;
}

double PIDControllerUpdate(PIDController *pid, double distMeasure){
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

		if(pid->Correction > (pid->PWMPeriod - pid->baseWidth)){
			pid->Correction = pid->PWMPeriod - pid->baseWidth;
		}
	}
	int newWidth = (pid->baseWidth - (pid->Correction * pid->PWMPeriod)/100);
	return (newWidth/pid->PWMPeriod)*100;

}


#endif /* PIDCONTROLLER_H_ */
