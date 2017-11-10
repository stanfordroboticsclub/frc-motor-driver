#include "wheel.h"

Wheel *left;
Wheel *right;

 

void setup() {
  left = new Wheel(2, 3, 11, 1);
  left->setGains(40, 100, 0, 100);
  left->setTargetVelocity(0.3);

  right = new Wheel(18, 19, 12, -1);
  right->setGains(40, 100, 0, 100);
  right->setTargetVelocity(0.1);

  Serial.begin(9600);
}

void loop() {
    left->update();
    right->update();
}
