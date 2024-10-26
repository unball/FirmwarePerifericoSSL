/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/
#include <Arduino.h>
#include <SimpleFOC.h>

TwoWire I2Cone = TwoWire(0);

void setup() {
  I2Cone.begin(21, 22, 400000);
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
}
 
void loop() {
    delay(2000);
    I2Cone.beginTransmission(4);
    I2Cone.write(0);
    I2Cone.endTransmission();
    Serial.println("EScrevi 0");

    delay(2000);
    I2Cone.beginTransmission(4);
    I2Cone.write(1);
    I2Cone.endTransmission();
    Serial.println("EScrevi 1");

    delay(2000);
    I2Cone.beginTransmission(4);
    I2Cone.write(2);
    I2Cone.endTransmission();
    Serial.println("EScrevi 2");

    delay(2000);
    I2Cone.beginTransmission(4);
    I2Cone.write(3);
    I2Cone.endTransmission();
    Serial.println("EScrevi 3");
}
