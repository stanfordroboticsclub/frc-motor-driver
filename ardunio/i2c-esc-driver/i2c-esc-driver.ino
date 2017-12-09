#include "wheel.h"
#include <Wire.h>


#define SLAVE_ADDRESS 0x04

Wheel *left;
Wheel *right;

void receiveData(int byteCount) {
  if (byteCount == 2) {
    int left_read = Wire.read();
    double left_target = mapd(left_read, 0, 255 , -2, 2);
    left->setTargetVelocity(left_target);


    int right_read = Wire.read();
    double right_target = mapd(right_read, 0, 255 , -2, 2);
    right->setTargetVelocity(right_target);

  } else {
    while (Wire.available()) {
      Wire.read();
    }
  }
}

void sendData() {

  long left_val = (long)(10000*left->measuredPosition);
  long right_val = (long)(-10000*right->measuredPosition);
    
  char data[8];

  data[0] = (left_val >> 24) & 0xFF;
  data[1] = (left_val >> 16) & 0xFF;
  data[2] = (left_val >> 8) & 0xFF;
  data[3] = (left_val >> 0) & 0xFF;

  data[4] = (right_val >> 24) & 0xFF;
  data[5] = (right_val >> 16) & 0xFF;
  data[6] = (right_val >> 8) & 0xFF;
  data[7] = (right_val >> 0) & 0xFF;
    
  Wire.read();
  Wire.write(data,8);

}

void setup() {
  Wire.begin(SLAVE_ADDRESS);

  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);


  left = new Wheel(2, 3, 11, 1);
//  left->setGains(40, 100, 0, 100);
  left->setGains(50, 0, 0, 110);
  left->setTargetVelocity(0);

  right = new Wheel(18, 19, 12, -1);
//  right->setGains(40, 100, 0, 100);
  right->setGains(50, 0, 0, 110);
  right->setTargetVelocity(0);

  Serial.begin(9600);
}

void loop() {
  left->update();
  right->update();
}
