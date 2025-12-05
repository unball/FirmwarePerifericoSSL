#include <Arduino.h>
#include <SimpleFOC.h>
#include "UART.hpp"

#define DEGRAU false
#define LIGADESLIGA false
#define RAMPA true
#define DEBUG true

unsigned long previousTsMove = 0;
unsigned long t0MoveMotor1 = 0;
unsigned long t0 = 0;
unsigned long tsMoveMotor = 7;
float wheelTarget1 = 0; 

MagneticSensorI2C encoder1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2CEncoder1 = TwoWire(1);
BLDCMotor motor1 = BLDCMotor(11,6.75);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);
InlineCurrentSense currentSense1 = InlineCurrentSense(0.01f, 50.0f,  35, 34);


void setup(){

  Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

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
 

}


#if DEGRAU
  unsigned long t0_degrau = 0;
  bool degrau = false;
#endif

#if LIGADESLIGA
  unsigned long t0_liga_desliga = 0;
  bool ligadesl = false;
#endif

#if RAMPA
  unsigned long t0_rampa = 0;
#endif

void loop(){
  unsigned long actualTimestamp = millis();
    
  #if RAMPA
    if(actualTimestamp - t0_rampa >= 50){

      // SE QUISER RAMPA POSITIVA, USAR O SEGUINTE CODIGO:
      wheelTarget1 += 0.1;
      if(wheelTarget1 >= 55){
        wheelTarget1 = 55;
      }


      // wheelTarget1 -= 0.1;
      // if(wheelTarget1 <= -55){
      //   wheelTarget1 = -55;
      // }
      t0_rampa = actualTimestamp;
    }
  #endif

  #if LIGADESLIGA
    if (!ligadesl) {
      ligadesl = true;
      t0_liga_desliga = millis();
    }

    if(ligadesl){
      if(millis() - t0_liga_desliga >= 1000){
        t0_liga_desliga = millis();
        
        if(wheelTarget1 != 0){
          wheelTarget1 = 0;
        }else{
          // SE QUISER POSITIVA, USAR O SEGUINTE CODIGO:
          // wheelTarget1 = 55;
      
          wheelTarget1 = -55;
        }
      }
    }   
  #endif

  #if DEGRAU
    if (!degrau) {
      degrau = true;
      t0_degrau = millis();
    }

    if(degrau){
      if(millis() - t0_degrau >= 3000){
        // SE QUISER POSITIVA, USAR O SEGUINTE CODIGO:
        // wheelTarget1 = 55;

        wheelTarget1 = -55;
      }else {
        wheelTarget1 = 0;
      }
    }
  #endif

  motor1.loopFOC();
  if(actualTimestamp - t0MoveMotor1 >= tsMoveMotor){
    motor1.move(wheelTarget1);
    t0MoveMotor1 = actualTimestamp;
  }
  
  #if DEBUG

    if(actualTimestamp - t0 >= 20){

      PhaseCurrent_s phaseCurrents = currentSense1.getPhaseCurrents();
      ABCurrent_s abCurrents = currentSense1.getABCurrents(phaseCurrents);
      DQCurrent_s dqCurrents = currentSense1.getDQCurrents(abCurrents, motor1.electrical_angle);
      float dcCurrent = currentSense1.getDCCurrent(motor1.electrical_angle);

      Serial.print(t0);
      Serial.print(" ");
      Serial.print(wheelTarget1);
      Serial.print(" ");
      Serial.print(encoder1.getSensorAngle());
      Serial.print(" ");
      Serial.print(encoder1.getVelocity());
      Serial.print(" ");
      Serial.print(phaseCurrents.a,4);
      Serial.print(" ");
      Serial.print(phaseCurrents.b,4);
      Serial.print(" ");
      Serial.print(dqCurrents.q,4);
      Serial.print(" ");
      Serial.print(dqCurrents.d,4);
      Serial.print(" ");
      Serial.println(dcCurrent,4);

      t0 = actualTimestamp;
    }
  #endif


}