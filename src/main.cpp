#include <SimpleFOC.h>
#include <Wire.h>
#include <Arduino.h>

MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C encoder1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2CTwo = TwoWire(1);

BLDCMotor motor = BLDCMotor(11,6.75);
BLDCMotor motor1 = BLDCMotor(11,6.75);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);
// 26, 27, 14, 12

InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, 39, 36);
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01f, 50.0f,  35, 34);

//Command Settings
// float target = 0;                                //Enter "T+speed" in the serial monitor to make the two motors rotate in closed loop
// Commander command = Commander(Serial);                    //For example, to make both motors rotate at a speed of 10rad/s, input "T10"
// void doTarget(char* cmd) { command.scalar(&target, cmd); }
void setup() {

    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);
    motor.useMonitoring(Serial);    
    motor1.useMonitoring(Serial);    
    
    I2Cone.begin(19,18, 400000UL);
    encoder.init(&I2Cone);
    motor.linkSensor(&encoder);

    driver.voltage_power_supply = 12;
    driver.voltage_limit = 12;
    driver.init();
    motor.linkDriver(&driver);
    current_sense.linkDriver(&driver);

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::dc_current;
    motor.controller = MotionControlType::velocity;
    motor.current_limit = 1;
    motor.voltage_limit = 12;
    motor.velocity_limit = 50;
    
    motor.linkCurrentSense(&current_sense);
    current_sense.gain_b *= -1;
    current_sense.init();

    motor.PID_current_q.P = 20;
    motor.PID_current_q.I = 100; 
    motor.PID_current_q.D = 0.01;
    motor.LPF_current_q.Tf = 0.01;
    motor.PID_current_q.limit = motor.voltage_limit;

    motor.PID_velocity.P = 0.001;
    motor.PID_velocity.I = 0.1;
    motor.PID_velocity.D = 0.00001;
    motor.LPF_velocity.Tf = 0.0001;
    motor.PID_velocity.output_ramp = 1000;


    // motor 1
    I2CTwo.begin(23, 5, 400000UL);
    encoder1.init(&I2CTwo);
    motor1.linkSensor(&encoder1);

    driver1.voltage_power_supply = 12;
    driver1.voltage_limit = 12;
    driver1.init();
    motor1.linkDriver(&driver1);
    current_sense1.linkDriver(&driver1);

    motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor1.torque_controller = TorqueControlType::dc_current;
    motor1.controller = MotionControlType::velocity;
    motor1.current_limit = 1;
    motor1.voltage_limit = 12;
    motor1.velocity_limit = 50;
    
    motor1.linkCurrentSense(&current_sense1);
    current_sense1.gain_b *= -1;
    current_sense1.init();

    motor1.PID_current_q.P = 20;
    motor1.PID_current_q.I = 100; 
    motor1.PID_current_q.D = 0.01;
    motor1.LPF_current_q.Tf = 0.01;
    motor1.PID_current_q.limit = motor1.voltage_limit;

    motor1.PID_velocity.P = 0.001;
    motor1.PID_velocity.I = 0.1;
    motor1.PID_velocity.D = 0.00001;
    motor1.LPF_velocity.Tf = 0.0001;
    motor1.PID_velocity.output_ramp = 1000;


    motor.init();
    motor1.init();
    
    motor.initFOC();    
    motor1.initFOC(); 

    _delay(1000);

}
unsigned long t0_current = 0;
unsigned long inter_current = 20; 

float target = 0;
unsigned long tempoInicio = 0;
bool degrau = false;
float incremento = 0.001;

unsigned long t0_move = 0;

unsigned long t0_foc = 0;

void loop() {
    unsigned long tf_current = millis();
    unsigned long tf_move = millis();
    unsigned long tf_foc = _micros();

    if (!degrau) {
        degrau = true;
        tempoInicio = millis();
    }


    if (degrau) {
        if (millis() - tempoInicio >= 3000) {
            target = -10;
        }         
        else { 
            target = 0;
        }
    }

    // if(millis() - tempoInicio >= 50){
    //     target += incremento;
    //     if(target >= 1){
    //         target = 1;
    //     }
    // }

    motor.loopFOC();
    motor1.loopFOC();

    // controle velocidade a cada 10ms 
    if(tf_move - t0_move >= 5){
        motor.move(target);
        motor1.move(target);

        t0_move = tf_move;
    }

    // command.run();

    if(tf_current - t0_current >= inter_current){

        PhaseCurrent_s phase_currents = current_sense.getPhaseCurrents();
        ABCurrent_s ab_currents = current_sense.getABCurrents(phase_currents);
        DQCurrent_s dq_currents = current_sense.getDQCurrents(ab_currents, motor.electrical_angle);
        float current_magnitude = current_sense.getDCCurrent();

        Serial.print(t0_current);
        Serial.print("\t");
        Serial.print(target, 5);
        Serial.print("\t");
        // Serial.print(phase_currents.a,5);
        // Serial.print("\t");
        // Serial.print(phase_currents.b,5);
        Serial.print("\t");
        Serial.print(dq_currents.q,5);
        Serial.print("\t");
        Serial.print(dq_currents.d,5);
        // Serial.print("\t");
        // Serial.print(ab_currents.alpha,5);
        // Serial.print("\t");
        // Serial.print(ab_currents.beta,5);
        Serial.print("\t");
        Serial.print(current_magnitude, 5);
        // Serial.print("\t");
        // Serial.print(encoder.getSensorAngle(),5);
        Serial.print("\t");
        Serial.println(encoder.getVelocity(),5);
        
        
        t0_current = tf_current;
    }
}