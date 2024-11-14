#include <SimpleFOC.h>
#include <Wire.h>
#include <Arduino.h>

BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26, 27, 14, 12);

//Target variable
float target_velocity1 = 0;
float target_velocity2 = 0;

void receiveEvent(int howMany)
{
  if (howMany == sizeof(float) * 2) // Verifica se recebeu exatamente 2 floats
  {
    while (Wire.available())
    {
      Wire.readBytes(reinterpret_cast<uint8_t *>(&target_velocity1), sizeof(float));
      Wire.readBytes(reinterpret_cast<uint8_t *>(&target_velocity2), sizeof(float));
    }

    // Exibe os valores recebidos para depuração
    Serial.print("u1 recebido: ");
    Serial.println(target_velocity1);
    Serial.print("u2 recebido: ");
    Serial.println(target_velocity2);
    
  }
  else
  {
    Serial.println("Dados recebidos incorretos.");
  }
}

//Serial command setting
Commander command = Commander(Serial);
void doTarget1(char* cmd) { command.scalar(&target_velocity1, cmd); }
void doTarget2(char* cmd) { command.scalar(&target_velocity2, cmd); }

void setup() {

  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = 5;   // [V]
  motor1.velocity_limit = 15; // [rad/s]
  
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.voltage_limit = 5;   // [V]
  motor2.velocity_limit = 15; // [rad/s]

 
  //Open loop control mode setting
  motor1.controller = MotionControlType::velocity_openloop;
  motor2.controller = MotionControlType::velocity_openloop;

  //Initialize the hardware
  motor1.init();
  motor2.init();

  Wire.begin(8, 19, 18, 400000); // Configura este ESP32 como escravo no endereço 8
  Wire.onReceive(receiveEvent);  // Registra o evento de recebimento
  Serial.begin(115200);
  Serial.println("Escravo 8 pronto para receber dados");


  //Add T command
  command.add('T', doTarget1, "target velocity");
  command.add('t', doTarget2, "target velocity");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  motor1.move(target_velocity1);
  motor2.move(target_velocity2);

  //User newsletter
  command.run();
}