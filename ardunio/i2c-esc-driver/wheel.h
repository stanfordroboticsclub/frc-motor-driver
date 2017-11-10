#ifndef WHEEL_H
#define WHEEL_H

#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>

double mapd(double x, double in_min, double in_max, double out_min, double out_max);
double squeeze(double x, double s_min, double s_max);

enum WheelMode {STOP, VOLTAGE, TELEOP, POSITION, VELOCITY};

struct Wheel{
	Wheel(int encPin1, int encPin2, int servoPin, int dir);
	const int UPDATE_RATE = 10; // Hz
	const int MS_PER_UPDATE = 1000 / UPDATE_RATE; // ms
	const double WHEEL_DIAMATER = 0.19; // m
	const double WHEEL_CIRCUMFERENCE = 3.14159 * WHEEL_DIAMATER; // m
	const double TICKS_PER_REV = 1000;
	int dir;

	Encoder *encoder;
	Servo *motor;
	long last_encoder;

	PID *pid;
	double kP;
	double kD;
	double kI;
	double kF;

	WheelMode mode;
	double targetVelocity;
	double targetPosition;
	double targetVoltage;
	double measuredVelocity;
	double measuredPosition;

	double pidOutput;
	double pidInput;
	double pidSetpoint;
	double motorOutput;
	unsigned long last_time;
	
	void setMode(WheelMode mode);
	void setGains(double kP, double kI, double kD, double kF);
	void reverseWheel();
	void setTargetVelocity(double targetSpeed);
	void setTargetPosition(double targetPosition);
	void setTargetVoltage(double targetVoltage);
	void resetPosition();
	void update();
};


#endif WHEEL_H
