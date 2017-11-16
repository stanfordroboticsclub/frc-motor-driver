#include "wheel.h"
#include <Wire.h>


#define SLAVE_ADDRESS 0x04

Wheel *left;
Wheel *right;

void receiveData(int byteCount){
  if(byteCount == 1){
    Wire.read();
    Wire.write(left->encoder->read());
    Wire.write(right->encoder->read());
    return;
  }
  else if(byteCount == 2){
    int left_read = Wire.read();
    double left_target = mapd(left_read, 0, 255 , -2, 2);  
    left->setTargetVelocity(left_target);


    int right_read = Wire.read();
    double right_target = mapd(right_read, 0, 255 , -2, 2);
    right->setTargetVelocity(right_target);

  }else{
    while (Wire.available()){ 
      Wire.read(); 
      }
  }
}

void sendData(){
  Wire.write(121);
  Wire.write(93);
}

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  
  left = new Wheel(2, 3, 11, 1);
  left->setGains(40, 100, 0, 100);
	left->setTargetVelocity(0);

  right = new Wheel(18, 19, 12, -1);
  right->setGains(40, 100, 0, 100);
	right->setTargetVelocity(0);

	Serial.begin(9600);
}

void loop() {
    left->update();
    right->update();
}
