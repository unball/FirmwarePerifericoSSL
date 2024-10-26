#include <SimpleFOC.h>

MagneticSensorPWM sensor = MagneticSensorPWM(GPIO_NUM_15, 2, 922);
void doPWM(){sensor.handlePWM();}
MagneticSensorPWM sensor1 = MagneticSensorPWM(GPIO_NUM_13, 2, 922);
void doPWM1(){sensor1.handlePWM();}

void setup() {
  // monitoring port
  Serial.begin(115200);
  
  sensor.init();
  sensor1.init();
  sensor.enableInterrupt(doPWM);
  sensor1.enableInterrupt(doPWM1);

  Serial.println("Encoder ready");
  _delay(1000);
}

void loop() {
  // IMPORTANT
  // read sensor and update the internal variables
  sensor.update();
  sensor1.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.print(sensor.getVelocity());
  Serial.print("\t\t");
  Serial.print(sensor1.getAngle());
  Serial.print("\t");
  Serial.println(sensor1.getVelocity());
}
