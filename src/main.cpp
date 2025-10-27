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

}

float input = 0;

unsigned long t0Current = 0;
unsigned long tsCurrent = 20; 

unsigned long t0StepSignal = 0;
bool stepSignal = false;

unsigned long t0MoveMotor = 0;
unsigned long tsMoveMotor = 5;

float addInput = 0.001;

void loop() {
    unsigned long tfCurrent = millis();
    unsigned long tfMoveMotor = millis();
    unsigned long tfRampSignal = millis();

    if (!stepSignal) {
        stepSignal = true;
        t0StepSignal = millis();
    }

    if (stepSignal) {
        if (millis() - t0StepSignal >= 3000) {
            input = -50;
        }         
        else { 
            input = 0;
        }
    }

    motor0.loopFOC();

    if(tfMoveMotor - t0MoveMotor >= tsMoveMotor){
        motor0.move(input);
        t0MoveMotor = tfMoveMotor;
    }

    if(tfCurrent - t0Current >= tsCurrent){

        PhaseCurrent_s phaseCurrents = currentSense0.getPhaseCurrents();
        ABCurrent_s abCurrents = currentSense0.getABCurrents(phaseCurrents);
        DQCurrent_s dqCurrents = currentSense0.getDQCurrents(abCurrents, motor0.electrical_angle);
        float currentMagnitude = currentSense0.getDCCurrent();

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
        Serial.print(encoder0.getSensorAngle(),3);
        Serial.print("\t");
        Serial.println(encoder0.getVelocity(),3);

        t0Current = tfCurrent;
    }
}