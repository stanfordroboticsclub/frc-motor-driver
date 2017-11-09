#include <Encoder.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x04

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
	int last_encoder;

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

Wheel::Wheel(int encPin1, int encPin2, int servoPin, int dir) {
    pinMode(encPin1, INPUT);
    pinMode(encPin2, INPUT);
    pinMode(servoPin, OUTPUT);
    this->dir = dir;
  	this->encoder = new Encoder(encPin1, encPin2);
    this->motor  = new Servo();
    this->motor->attach(servoPin);
    
    this->last_time = millis();
    this->last_encoder = 0;

    this->pid = new PID(&(this->pidInput), &(this->pidOutput), &(this->pidSetpoint), this->kP, this->kI, this->kD, DIRECT);
    this->pid->SetOutputLimits(-255, 255);
    this->pid->SetMode(AUTOMATIC);
	  
	  this->measuredPosition = 0;
	  mode = STOP;
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
  this->targetVelocity = this->dir * targetSpeed;
  this->pidSetpoint = this->targetVelocity;
	this->setMode(VELOCITY);
}

void Wheel::setTargetPosition(double targetPosition) {
	this->targetPosition = this->dir * targetPosition;
	this->pidSetpoint = this->targetPosition;
	this->setMode(POSITION);
}

void Wheel::setTargetVoltage(double targetVoltage) {
	this->targetVoltage = this->dir * targetVoltage;
	this->setMode(VOLTAGE);
}

void Wheel::resetPosition() {
  this->measuredPosition = 0;
}

void Wheel::update(){
	if(millis() - this->last_time > this->MS_PER_UPDATE) {
    double dT = (millis() - (double) this->last_time) / 1000;
    this->last_time = millis();
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
			Serial.println("Mode: VELOCITY");
      this->pidInput = this->measuredVelocity;
      Serial.println(this->pidInput);
			this->pid->Compute();
     Serial.println(this->pidOutput);
			this->motorOutput = this->pidOutput + this->kF * this->targetVelocity;
			break;
		default:
			Serial.println("Bad Mode");
		}
    Serial.println(this->motorOutput);
  	this->motorOutput = squeeze(this->motorOutput, -255, 255);
  	Serial.println(this->motorOutput);
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



Wheel *left;
Wheel *right;

void receiveData(int byteCount){
  if(byteCount == 1){
    Wire.read();
    Wire.write(left->encoder->read());
    Wire.write(right->encoder->read());
    return;
  }

  if(byteCount != 2) return;

  if(Wire.available()) {
    int left_read = Wire.read();
    double left_target = mapd(left_read, 0, 255 , -2, 2);  
    left->setTargetVelocity(left_target);

  }
  Serial.print(", "); 
  if(Wire.available()) {
    int right_read = Wire.read();
    double right_target = mapd(right_read, 0, 255 , -2, 2);
    right->setTargetVelocity(right_target);
  }
}

void sendData(){
  Wire.write(0);
  Wire.write(0);
}

void setup() {
//  Wire.begin(SLAVE_ADDRESS);
//  
//  Wire.onReceive(receiveData);
//  Wire.onRequest(sendData);

  
  left = new Wheel(18, 19, 11, 1);
  left->setGains(40, 100, 0, 100);
	left->setTargetVelocity(0);

  right = new Wheel(2, 3, 12, -1);
  right->setGains(40, 100, 0, 100);
	right->setTargetVelocity(0.5);

	Serial.begin(9600);
}

void loop() {
    //left->update();
    right->update();
}
