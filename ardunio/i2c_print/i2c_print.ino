#include <Wire.h>


#define SLAVE_ADDRESS 0x04

void setup() {
  Wire.begin(SLAVE_ADDRESS);

  
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.begin(9600);           

}

void loop() {


}


void receiveData(int byteCount){

  
  if(byteCount == 1){
    Serial.println("Got single byte"); 
    Wire.read();
    Wire.write(0);
    Wire.write(0);
    return;
  }

  if(byteCount != 2){
    Serial.println("Got more then two byte"); 
    return;
  }

  if(Wire.available()) {
    int left_read = Wire.read();
    Serial.print(left_read); 
  }
  Serial.print(", "); 
  
  if(Wire.available()) {
    int right_read = Wire.read();
    Serial.println(right_read); 
  }

}

void sendData(){
  Serial.println("Got request for data"); 
  Wire.write(0);
  Wire.write(0);
}

