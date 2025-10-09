#include <SimpleFOC.h>
#include <Wire.h>
#include <Arduino.h>

MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

BLDCMotor motor = BLDCMotor(11,11.1);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, 39, 36);


void setup() {

    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);
    motor.useMonitoring(Serial);    
    
    I2Cone.begin(19,18, 400000UL);
    encoder.init(&I2Cone);
    motor.linkSensor(&encoder);

    driver.voltage_power_supply = 12;
    driver.voltage_limit = 12;
    driver.init();
    motor.linkDriver(&driver);
    current_sense.linkDriver(&driver);

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::torque;
    motor.current_limit = 1;
    motor.voltage_limit = 12;
    motor.init();
    
    current_sense.gain_b *= -1;
    current_sense.init();

    motor.linkCurrentSense(&current_sense);

    motor.initFOC();   

}
unsigned long t0_current = 0;
unsigned long inter_current = 10; //us

float target = 0;
unsigned long tempoInicio = 0;
bool degrau = false;

float incremento = 0.001;

void loop() {
    unsigned long tf_current = millis();
    unsigned long tempoAtual = millis();

    // motor.loopFOC();

    // if (!degrau) {
    //     degrau = true;
    //     tempoInicio = millis();
    // }


    // if (degrau) {
    //     if (millis() - tempoInicio >= 5000) {
    //         target = 1;
    //         motor.move(target);
    //     } else { 
    //         target = 0;
    //         motor.move(target);
    //     }
    // }


    if (tempoAtual - tempoInicio >= 50) {
        if(target >= 1){
            target = 1;
        }

        target += incremento;
        
        tempoInicio = tempoAtual;
    }

    motor.loopFOC();
    motor.move(target);

    
    if(tf_current - t0_current >= inter_current){

        PhaseCurrent_s phase_currents = current_sense.getPhaseCurrents();
        ABCurrent_s ab_currents = current_sense.getABCurrents(phase_currents);
        DQCurrent_s dq_currents = current_sense.getDQCurrents(ab_currents, motor.electrical_angle);
        float current_magnitude = current_sense.getDCCurrent();

        Serial.print(t0_current);
        Serial.print("\t");
        Serial.print(target, 5);
        Serial.print("\t");
        Serial.print(phase_currents.a,5);
        Serial.print("\t");
        Serial.print(phase_currents.b,5);
        Serial.print("\t");
        Serial.print(dq_currents.q,5);
        Serial.print("\t");
        Serial.print(dq_currents.d,5);
        Serial.print("\t");
        Serial.print(ab_currents.alpha,5);
        Serial.print("\t");
        Serial.print(ab_currents.beta,5);
        Serial.print("\t");
        Serial.print(current_magnitude, 5);
        Serial.print("\t");
        Serial.print(encoder.getSensorAngle(),5);
        Serial.print("\t");
        Serial.println(encoder.getVelocity(),5);
        
        
        t0_current = tf_current;
    }
}