#include <SimpleFOC.h>
#include <Wire.h>
#include <Arduino.h>

BLDCMotor motor3 = BLDCMotor(11);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor4 = BLDCMotor(11);
BLDCDriver3PWM driver4 = BLDCDriver3PWM(26, 27, 14, 12);

//Target variable
float target_velocity3 = 0;
float target_velocity4 = 0;

void receiveEvent(int howMany)
{
  if (howMany == sizeof(float) * 2) // Verifica se recebeu exatamente 2 floats
  {
    while (Wire.available())
    {
      Wire.readBytes(reinterpret_cast<uint8_t *>(&target_velocity3), sizeof(float));
      Wire.readBytes(reinterpret_cast<uint8_t *>(&target_velocity4), sizeof(float));
    }

    // Exibe os valores recebidos para depuração
    Serial.print("u1 recebido: ");
    Serial.println(target_velocity3);
    Serial.print("u2 recebido: ");
    Serial.println(target_velocity4);
    
  }
  else
  {
    Serial.println("Dados recebidos incorretos.");
  }
}

//Serial command setting
Commander command = Commander(Serial);
void doTarget3(char* cmd) { command.scalar(&target_velocity3, cmd); }
void doTarget4(char* cmd) { command.scalar(&target_velocity4, cmd); }

void setup() {

  driver3.voltage_power_supply = 12;
  driver3.init();
  motor3.linkDriver(&driver3);
  motor3.voltage_limit = 5;   // [V]
  motor3.velocity_limit = 15; // [rad/s]
  
  driver4.voltage_power_supply = 12;
  driver4.init();
  motor4.linkDriver(&driver4);
  motor4.voltage_limit = 5;   // [V]
  motor4.velocity_limit = 15; // [rad/s]

 
  //Open loop control mode setting
  motor3.controller = MotionControlType::velocity_openloop;
  motor4.controller = MotionControlType::velocity_openloop;

  //Initialize the hardware
  motor3.init();
  motor4.init();

  Wire.begin(9, 19, 18, 400000); // Configura este ESP32 como escravo no endereço 8
  Wire.onReceive(receiveEvent);  // Registra o evento de recebimento
  Serial.begin(115200);
  Serial.println("Escravo 8 pronto para receber dados");


  //Add T command
  command.add('T', doTarget3, "target velocity");
  command.add('t', doTarget4, "target velocity");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  motor3.move(target_velocity3);
  motor4.move(target_velocity4);

  //User newsletter
  command.run();
}