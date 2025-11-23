#include <Arduino.h>
#include <SimpleFOC.h>
#include "UART.hpp"

#define CENTRAL_BOARD_ON true

unsigned long previousTsMove = 0;
unsigned long t0MoveMotor0 = 0;
unsigned long t0MoveMotor1 = 0;
unsigned long t0 = 0;
unsigned long tsMoveMotor = 7;
float wheelTarget0 = 0;
float wheelTarget1 = 0; 

MagneticSensorI2C encoder0 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2CEncoder0 = TwoWire(0);
BLDCMotor motor0 = BLDCMotor(11);
BLDCDriver3PWM driver0 = BLDCDriver3PWM(32, 33, 25, 22);
InlineCurrentSense currentSense0 = InlineCurrentSense(0.01f, 50.0f, 39, 36);

MagneticSensorI2C encoder1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2CEncoder1 = TwoWire(1);
BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);
InlineCurrentSense currentSense1 = InlineCurrentSense(0.01f, 50.0f,  35, 34);


void setup(){

  Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

    motor0.useMonitoring(Serial);    
    
    I2CEncoder0.begin(19,18, 400000UL);
    encoder0.init(&I2CEncoder0);
    motor0.linkSensor(&encoder0);

    driver0.voltage_power_supply = 12;
    driver0.voltage_limit = 12;
    driver0.pwm_frequency = 45000;
    driver0.init();
    motor0.linkDriver(&driver0);
    currentSense0.linkDriver(&driver0);

    motor0.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor0.torque_controller = TorqueControlType::dc_current;
    motor0.controller = MotionControlType::velocity;
    motor0.current_limit = 1.1;
    motor0.voltage_limit = 12;
    motor0.velocity_limit = 55;
    
    motor0.linkCurrentSense(&currentSense0);
    currentSense0.gain_b *= -1;
    currentSense0.init();

    motor0.PID_current_q.P = 25;
    motor0.PID_current_q.I = 10000; 
    motor0.PID_current_q.D = -0.001;
    motor0.LPF_current_q.Tf = 0.01;
    motor0.PID_current_q.limit = motor0.voltage_limit;

    motor0.PID_velocity.P = 0.01;
    motor0.PID_velocity.I = 1;
    motor0.PID_velocity.D = -0.00001;
    motor0.LPF_velocity.Tf = 0.001;
    motor0.PID_velocity.output_ramp = 1000;

    motor0.init();
    motor0.initFOC();    

    _delay(1000);

    motor1.useMonitoring(Serial);    
    
    I2CEncoder1.begin(23, 5, 400000UL);
    encoder1.init(&I2CEncoder1);
    motor1.linkSensor(&encoder1);

    driver1.voltage_power_supply = 12;
    driver1.voltage_limit = 12;
    driver1.pwm_frequency = 45000;
    driver1.init();
    motor1.linkDriver(&driver1);
    currentSense1.linkDriver(&driver1);

    motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor1.torque_controller = TorqueControlType::dc_current;
    motor1.controller = MotionControlType::velocity;
    motor1.current_limit = 1.1;
    motor1.voltage_limit = 12;
    motor1.velocity_limit = 55;
    
    motor1.linkCurrentSense(&currentSense1);
    currentSense1.gain_b *= -1;
    currentSense1.init();

    motor1.PID_current_q.P = 25;
    motor1.PID_current_q.I = 10000; 
    motor1.PID_current_q.D = -0.001;
    motor1.LPF_current_q.Tf = 0.01;
    motor1.PID_current_q.limit = motor1.voltage_limit;

    motor1.PID_velocity.P = 0.01;
    motor1.PID_velocity.I = 1;
    motor1.PID_velocity.D = -0.00001;
    motor1.LPF_velocity.Tf = 0.001;
    motor1.PID_velocity.output_ramp = 1000;

    motor1.init();
    motor1.initFOC();    

    _delay(1000);
 
  #if CENTRAL_BOARD_ON
    UART::setup();
  #endif

}

void loop(){
  unsigned long actualTimestamp = millis();

  #if CENTRAL_BOARD_ON
    UART::receiveMessage(&wheelTarget0,&wheelTarget1);
  #endif


  motor0.loopFOC();
  if(actualTimestamp - t0MoveMotor0 >= tsMoveMotor){
      motor0.move(wheelTarget0);
      t0MoveMotor0 = actualTimestamp;
  }

  motor1.loopFOC();
  if(actualTimestamp - t0MoveMotor1 >= tsMoveMotor){
      motor1.move(wheelTarget1);
      t0MoveMotor1 = actualTimestamp;
  }

  // if(actualTimestamp - t0 >= 20){
  //   Serial.print(encoder0.getVelocity());
  //   Serial.print(" ");
  //   Serial.println(encoder1.getVelocity());
  // }

}