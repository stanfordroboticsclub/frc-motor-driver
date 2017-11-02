#include <Encoder.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x04


struct Wheel{
	Encoder *encoder;
	PID *pid;
	Servo *motor;
	double speed;
	double output;
	double input;
  double Kf;
  int dir;

  unsigned long last_time;
  int last_encoder;
	const double wheel_diameter = 0.19;
  const int encoder_count_per_rotation = 1000;
	void calculate();
  Wheel(int,int,int);
  void SetTunings(double Kf, double Kp, double Ki, double Kd);
  void setSpeed(double);
};

void Wheel::setSpeed(double speed){
  this->speed = this->dir * speed;
}

Wheel::Wheel(int encPin1, int encPin2, int servoPin){
  
    pinMode(encPin1, INPUT);
    pinMode(encPin2, INPUT);
    pinMode(servoPin, OUTPUT);

    this->dir = 1;
    
    last_time = millis();
    last_encoder = 0;

    double Kp=40, Ki=100, Kd=0;
    this->Kf = 110;

	  this->encoder = new Encoder(encPin1, encPin2);
    this->motor  = new Servo();
    this->motor->attach(servoPin);

    this->pid = new PID(&(this->input), &(this->output), &(this->speed), Kp, Ki, Kd,DIRECT);

    this->pid->SetOutputLimits(-500, 500);
    
    this->pid->SetMode(AUTOMATIC);
}

void Wheel::calculate(){

    if( millis() - this->last_time > 100){
        unsigned long time_change = millis() - this->last_time;
        this->last_time = millis();

        int change = this->last_encoder - this->encoder->read();
        this->last_encoder = this->encoder->read();

        this->input = ((double)change / this->encoder_count_per_rotation ) * this->wheel_diameter * 3.14159 / ((double)time_change / 1000);

          Serial.print(this->speed); Serial.print(" ");
           Serial.print(this->input); Serial.print(" ");
           Serial.print((this->output + this->speed*this->Kf )); Serial.print(" ");
           Serial.println();
    }

    this->pid->Compute();
    this->motor->writeMicroseconds(1500 - (this->output + this->speed*this->Kf ));
}

void Wheel::SetTunings(double Kf, double Kp, double Ki, double Kd){
  this->pid->SetTunings(Kp,Ki,Kp);
  this->Kf = Kf;
}


double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
 double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 return temp;
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
    left->setSpeed(left_target);

  }
  Serial.print(", "); 
  if(Wire.available()) {
    int right_read = Wire.read();
    double right_target = mapd(right_read, 0, 255 , -2, 2);

    right->setSpeed(right_target);
  }
}

void sendData(){
  Wire.write(0);
  Wire.write(0);
}



void setup() {
  Wire.begin(SLAVE_ADDRESS);
  
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  
  left = new Wheel(18, 19, 11);
  right = new Wheel(2, 3, 12);
  left->dir = -1;

  Serial.begin(9600);
  left->setSpeed(0);
  right->setSpeed(0);
}

void loop() {
    left->calculate();
    right->calculate();
}





