#include <SimpleFOC.h>
#include <Wire.h>
#include <Arduino.h>

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

void setup() {

    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

    motor0.useMonitoring(Serial);    
    
    I2CEncoder0.begin(19,18, 400000UL);
    encoder0.init(&I2CEncoder0);
    motor0.linkSensor(&encoder0);

    driver0.voltage_power_supply = 12;
    driver0.voltage_limit = 12;
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

}

float input = 0;

unsigned long t0Current = 0;
unsigned long tsCurrent = 20; 

unsigned long t0StepsSignal = 0;

unsigned long t0MoveMotor = 0;
unsigned long tsMoveMotor = 5;

void loop() {
    unsigned long tfCurrent = millis();
    unsigned long tfMoveMotor = millis();
    unsigned long tfStepsSignal = millis();

    if(tfStepsSignal - t0StepsSignal >= 1000){
        t0StepsSignal = tfStepsSignal;
        
        if(input != 0){
            input = 0;
        }else{
            input = -50;
        }
    }

    motor0.loopFOC();
    motor1.loopFOC();

    if(tfMoveMotor - t0MoveMotor >= tsMoveMotor){
        motor0.move(input);
        motor1.move(input);
        t0MoveMotor = tfMoveMotor;
    }

    if(tfCurrent - t0Current >= tsCurrent){

        // PhaseCurrent_s phaseCurrents = currentSense0.getPhaseCurrents();
        // ABCurrent_s abCurrents = currentSense0.getABCurrents(phaseCurrents);
        // DQCurrent_s dqCurrents = currentSense0.getDQCurrents(abCurrents, motor0.electrical_angle);
        // float currentMagnitude = currentSense0.getDCCurrent();

        // Serial.print(t0Current);
        // Serial.print("\t");
        // Serial.print(input, 3);
        // Serial.print("\t");
        // Serial.print(dqCurrents.q,3);
        // Serial.print("\t");
        // Serial.print(dqCurrents.d,3);
        // Serial.print("\t");
        // Serial.print(currentMagnitude, 3);
        // Serial.print("\t");
        // Serial.print(encoder0.getSensorAngle(),3);
        // Serial.print("\t");
        // Serial.println(encoder0.getVelocity(),3);

        // phaseCurrents = currentSense1.getPhaseCurrents();
        // abCurrents = currentSense1.getABCurrents(phaseCurrents);
        // dqCurrents = currentSense1.getDQCurrents(abCurrents, motor0.electrical_angle);
        // currentMagnitude = currentSense1.getDCCurrent();

        // Serial.print(dqCurrents.q,3);
        // Serial.print("\t");
        // Serial.print(dqCurrents.d,3);
        // Serial.print("\t");
        // Serial.print(currentMagnitude, 3);
        // Serial.print("\t");
        // Serial.print(encoder1.getSensorAngle(),3);
        // Serial.print("\t");
        // Serial.println(encoder1.getVelocity(),3);

        t0Current = tfCurrent;
    }
}