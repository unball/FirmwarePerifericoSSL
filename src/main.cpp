#include <SimpleFOC.h>
#include <Wire.h>
#include <Arduino.h>
#include "constants/constants_Motor_0.h"
#include "constants/constants_Motor_1.h"
#include "constants/constants_torque_PID.h"
#include "constants/constants_GM4108H120T.h"

MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);
TwoWire I2CEncoder = TwoWire(motor1::I2C_BUS_NUMBER);
BLDCMotor motor = BLDCMotor(gm4108h120T::NUMBER_POLES, gm4108h120T::PHASE_RESISTANCE);
BLDCDriver3PWM driver = BLDCDriver3PWM(motor1::DRIVER_PINOUT_PHASE_A, motor1::DRIVER_PINOUT_PHASE_B, motor1::DRIVER_PINOUT_PHASE_C, motor1::DRIVER_PINOUT_ENABLE);
InlineCurrentSense currentSense = InlineCurrentSense(motor1::CURRENT_SENSE_RESISTOR, motor1::CURRENT_SENSE_GAIN, motor1::CURRENT_SENSE_PINOUT_PHASE_A, motor1::CURRENT_SENSE_PINOUT_PHASE_B);

void setup() {

    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

    motor.useMonitoring(Serial);    
    
    I2CEncoder.begin(motor1::I2C_PINOUT_SDA,motor1::I2C_PINOUT_SCL, motor1::I2C_PINOUT_FREQUENCY);
    encoder.init(&I2CEncoder);
    motor.linkSensor(&encoder);

    driver.voltage_power_supply = motor1::DRIVER_VOLTAGE_POWER_SUPPLY;
    driver.voltage_limit = motor1::DRIVER_VOLTAGE_LIMIT;
    driver.init();
    motor.linkDriver(&driver);
    currentSense.linkDriver(&driver);

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::torque;
    motor.current_limit = gm4108h120T::MOTOR_CURRENT_LIMIT;
    motor.voltage_limit = gm4108h120T::MOTOR_VOLTAGE_LIMIT;
    motor.velocity_limit = gm4108h120T::MOTOR_VELOCITY_LIMIT;
    
    motor.linkCurrentSense(&currentSense);
    currentSense.gain_b *= motor1::CURRENT_SENSE_GAIN_B;
    currentSense.init();

    motor.init();
    motor.initFOC();    

    _delay(1000);

}

float input = 0.5;

unsigned long t0Current = 0;
unsigned long tsCurrent = 20; 

void loop() {
    unsigned long tfCurrent = millis();

    motor.loopFOC();
    motor.move(input);

    if(tfCurrent - t0Current >= tsCurrent){

        PhaseCurrent_s phaseCurrents = currentSense.getPhaseCurrents();
        ABCurrent_s abCurrents = currentSense.getABCurrents(phaseCurrents);
        DQCurrent_s dqCurrents = currentSense.getDQCurrents(abCurrents, motor.electrical_angle);
        float currentMagnitude = currentSense.getDCCurrent();

        Serial.print(t0Current);
        Serial.print("\t");
        Serial.print(input, 3);
        Serial.print("\t");
        Serial.print(dqCurrents.q,3);
        Serial.print("\t");
        Serial.print(dqCurrents.d,3);
        Serial.print("\t");
        Serial.print(currentMagnitude, 3);
        Serial.print("\t");
        Serial.print(encoder.getSensorAngle(),3);
        Serial.print("\t");
        Serial.println(encoder.getVelocity(),3);

        t0Current = tfCurrent;
    }
}