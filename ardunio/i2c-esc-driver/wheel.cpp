#include "wheel.h"

Wheel::Wheel(int encPin1, int encPin2, int servoPin, int dir) {
    pinMode(encPin1, INPUT);
    pinMode(encPin2, INPUT);
    pinMode(servoPin, OUTPUT);
    this->dir = dir;
  	this->encoder = new Encoder(encPin1, encPin2);
    this->motor  = new Servo();
    this->motor->attach(servoPin);
    
    this->last_time = millis();
    this->last_encoder = this->encoder->read();
    this->measuredPosition = this->last_encoder / this->TICKS_PER_REV * this->WHEEL_CIRCUMFERENCE;

    this->pidSetpoint = 0;
    this->pidInput = 0;
    this->pidOutput = 0;

    this->pid = new PID(&(this->pidInput), &(this->pidOutput), &(this->pidSetpoint), this->kP, this->kI, this->kD, DIRECT);
    this->pid->SetOutputLimits(-255, 255);
	  
	  this->mode = STOP;
}

void Wheel::setMode(WheelMode mode) {
	this->mode = mode;
}

void Wheel::setGains(double kP, double kI, double kD, double kF) {
	this->kP = kP;
	this->kI = kI;
	this->kD = kD;
	this->kF = kF;
  this->pid->SetTunings(kP,kI,kD);
}

void Wheel::reverseWheel() {
	this->dir = (this->dir == 1) ? -1 : 1;
}

void Wheel::setTargetVelocity(double targetSpeed){
  if ( abs(targetSpeed) < 0.02 ){
    this->stop();
  }else{
    this->pid->SetMode(AUTOMATIC);
    this->targetVelocity = this->dir * targetSpeed;
    this->pidSetpoint = this->targetVelocity;
  	this->setMode(VELOCITY);
  }
}

void Wheel::setTargetPosition(double targetPosition) {
  this->pid->SetMode(AUTOMATIC);
	this->targetPosition = this->dir * targetPosition;
	this->pidSetpoint = this->targetPosition;
	this->setMode(POSITION);
}

void Wheel::setTargetVoltage(double targetVoltage) {
  this->pid->SetMode(MANUAL);
	this->targetVoltage = this->dir * targetVoltage;
	this->setMode(VOLTAGE);
}

void Wheel::stop(){
  this->pid->SetMode(MANUAL);
  this->setMode(STOP);
}

void Wheel::resetPosition() {
  this->measuredPosition = 0;
}

void Wheel::update(){
	if(millis() - this->last_time > this->MS_PER_UPDATE) {
    double dT = (millis() - (double) this->last_time) / 1000;
    this->last_time = millis();

    // experiment for detecting broken wires
    // doesn't work as wheels are initialy not spining
    bool wire_contact = true;
    if(abs(last_encoder - this->encoder->read()) <= 1){
//      Serial.println("LOST ENCODER!");
      wire_contact = false;
    }

//    Serial.print(last_encoder);Serial.print(" "); Serial.println(this->encoder->read());
    
    this->last_encoder = this->encoder->read();

    double newMeasuredPosition = this->last_encoder / this->TICKS_PER_REV * this->WHEEL_CIRCUMFERENCE;
    this->measuredVelocity = (newMeasuredPosition - this->measuredPosition) / dT;
    this->measuredPosition = newMeasuredPosition;

		switch(mode) {
		case STOP:
			Serial.println("Mode: STOP");
			this->motorOutput = 0;
			break;
		case VOLTAGE:
			Serial.println("Mode: VOLTAGE");
			this->motorOutput = this->targetVoltage;
			break;
		case POSITION: //PID(measuredPosition, output)
			Serial.println("Mode: POSITION");
			this->pidInput = this->measuredPosition;
			this->pid->Compute();
      this->motorOutput = this->pidOutput;
			break;
		case VELOCITY: //PIDF(measuredVelocity, output)
//			Serial.println("Mode: VELOCITY");
      this->pidInput = this->measuredVelocity;
//      Serial.println(this->pidInput);
			this->pid->Compute();
//     Serial.println(this->pidOutput);
			this->motorOutput = this->pidOutput + this->kF * this->targetVelocity;
			break;
		default:
			Serial.println("Bad Mode");
		}
  	this->motorOutput = squeeze(this->motorOutput, -255, 255);
//  	Serial.println(this->motorOutput);
  	int servoOutput = (int) mapd(this->motorOutput, -255, 255, 1500 - 532, 1500 + 532);
 
    this->motor->writeMicroseconds(servoOutput);

	}
}



double mapd(double x, double in_min, double in_max, double out_min, double out_max){
 double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 return temp;
}

double squeeze(double x, double s_min, double s_max) {
	if(x < s_min) {
		return s_min;
	}
	if(x > s_max) {
		return s_max;
	}
	return x;
}
