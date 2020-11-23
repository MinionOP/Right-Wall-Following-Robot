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

void InitPID(PIDController *pid, double _PWMPeriod, double _baseWidth);
void setPIDRight(PIDController* pid);
void setPIDForward(PIDController* pid);

double PIDUpdate(PIDController *pid, double distMeasure);
double getPrevError(PIDController *pid);


#endif /* PIDCONTROLLER_H_ */
