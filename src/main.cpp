#include <SimpleFOC.h>
#include <Wire.h>
#include <Arduino.h>
#include "constants/constants_Motor_0.h"
#include "constants/constants_Motor_1.h"
#include "constants/constants_torque_PID.h"
#include "constants/constants_velocity_PID.h"
#include "constants/constants_GM4108H120T.h"

#define UART_TX 13
#define UART_RX 15

MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);
TwoWire I2CEncoder = TwoWire(motor0::I2C_BUS_NUMBER);
BLDCMotor motor = BLDCMotor(gm4108h120T::NUMBER_POLES, gm4108h120T::PHASE_RESISTANCE);
BLDCDriver3PWM driver = BLDCDriver3PWM(motor0::DRIVER_PINOUT_PHASE_A, motor0::DRIVER_PINOUT_PHASE_B, motor0::DRIVER_PINOUT_PHASE_C, motor0::DRIVER_PINOUT_ENABLE);
InlineCurrentSense currentSense = InlineCurrentSense(motor0::CURRENT_SENSE_RESISTOR, motor0::CURRENT_SENSE_GAIN, motor0::CURRENT_SENSE_PINOUT_PHASE_A, motor0::CURRENT_SENSE_PINOUT_PHASE_B);

HardwareSerial mySerial(1);

void setup() {

    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

    motor.useMonitoring(Serial);    
    
    I2CEncoder.begin(motor0::I2C_PINOUT_SDA,motor0::I2C_PINOUT_SCL, motor0::I2C_PINOUT_FREQUENCY);
    encoder.init(&I2CEncoder);
    motor.linkSensor(&encoder);

    driver.voltage_power_supply = motor0::DRIVER_VOLTAGE_POWER_SUPPLY;
    driver.voltage_limit = motor0::DRIVER_VOLTAGE_LIMIT;
    driver.init();
    motor.linkDriver(&driver);
    currentSense.linkDriver(&driver);

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::dc_current;
    motor.controller = MotionControlType::velocity;
    motor.current_limit = gm4108h120T::MOTOR_CURRENT_LIMIT;
    motor.voltage_limit = gm4108h120T::MOTOR_VOLTAGE_LIMIT;
    motor.velocity_limit = gm4108h120T::MOTOR_VELOCITY_LIMIT;
    
    motor.linkCurrentSense(&currentSense);
    currentSense.gain_b *= motor0::CURRENT_SENSE_GAIN_B;
    currentSense.init();

    motor.PID_current_q.P = torquePID::P_iq;
    motor.PID_current_q.I = torquePID::I_iq; 
    motor.PID_current_q.D = torquePID::D_iq;
    motor.LPF_current_q.Tf = torquePID::Tf_iq;
    motor.PID_current_q.limit = motor.voltage_limit;

    motor.PID_velocity.P = velocityPID::P_vel;
    motor.PID_velocity.I = velocityPID::I_vel;
    motor.PID_velocity.D = velocityPID::D_vel;
    motor.LPF_velocity.Tf = velocityPID::Tf_vel;
    motor.PID_velocity.output_ramp = velocityPID::output_ramp_vel;

    motor.init();
    motor.initFOC(); 

    mySerial.begin(9600,SERIAL_8N1,UART_RX,UART_TX);
    while(!mySerial);
    
    _delay(1000);

}

float input = 0;

unsigned long t0Current = 0;
unsigned long tsCurrent = 20; 

unsigned long t0StepsSignal = 0;

unsigned long t0MoveMotor = 0;
unsigned long tsMoveMotor = 5;

float x = 0;
float y = 0;

unsigned long t0 = 0;

void loop() {
    unsigned long tfCurrent = millis();
    unsigned long tfMoveMotor = millis();
    unsigned long tfStepsSignal = millis();

    motor.loopFOC();

    if(tfMoveMotor - t0MoveMotor >= tsMoveMotor){
        Serial.println(input);
        motor.move(input);
        t0MoveMotor = tfMoveMotor;
    }

    if(millis() - t0 >= 50){

      if(mySerial.available()){
        String msg = mySerial.readStringUntil('\n');
  
        char buffer[msg.length() + 1] = {};
        strcpy(buffer, msg.c_str());
  
        int commaIndex = msg.indexOf(',');
        if (commaIndex > 0) {
          x = msg.substring(0, commaIndex).toFloat();
          y = msg.substring(commaIndex + 1).toFloat();
  
        }
  
      }

      input = x;

      Serial.print(x);
      Serial.print(" ");
      Serial.println(y);

      t0 = millis();
    }
}