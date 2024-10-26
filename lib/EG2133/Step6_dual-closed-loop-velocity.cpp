/**
 *
 * Position/angle motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target angle (in radians) from serial terminal
 *
 */
#include <SimpleFOC.h>

// magnetic sensor instance - PWM
MagneticSensorPWM sensor = MagneticSensorPWM(GPIO_NUM_15, 2, 922);

void doPWM(){sensor.handlePWM();}

MagneticSensorPWM sensor1 = MagneticSensorPWM(GPIO_NUM_13, 2, 922);
void doPWM1(){sensor1.handlePWM();}

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);

BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1  = BLDCDriver3PWM(26,27,14,12);



// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

// InlineCurrentSense current_sense0 = InlineCurrentSense(0.01, 50.0, 39, 36);

void setup() {
  // pinMode(GPIO_NUM_23, OUTPUT);
  // digitalWrite(GPIO_NUM_23, HIGH);
  SimpleFOCDebug::enable();


  // initialise magnetic sensor hardware
  sensor.init();
  sensor.enableInterrupt(doPWM);

  sensor1.init();
  sensor1.enableInterrupt(doPWM1);
  
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  motor1.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;

  
  driver1.voltage_power_supply = 12;
  driver1.voltage_limit = 6;

  driver.init();
  motor.linkDriver(&driver);

  driver1.init();
  motor1.linkDriver(&driver1);

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 6;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 20;
  // maximal velocity of the position control
  motor.velocity_limit = 10;




  // velocity PI controller parameters
  motor1.PID_velocity.P = 0.2f;
  motor1.PID_velocity.I = 20;
  motor1.PID_velocity.D = 0;
  // maximal voltage to be set to the motor
  motor1.voltage_limit = 6;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor1.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor1.P_angle.P = 20;
  // maximal velocity of the position control
  motor1.velocity_limit = 10;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  // motor.useMonitoring(Serial);


  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor1.init();
  motor.initFOC();

  // initialize motor
  // align sensor and start FOC
  motor1.initFOC();

  // add target command T
  command.add('T', doTarget, "target angle");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}


void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();
  motor1.loopFOC();
  // sensor.update();
  // sensor1.update();
  // Serial.print(sensor.getAngle());
  // Serial.print("  ");
  // Serial.println(sensor1.getAngle());

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_angle);
  motor1.move(target_angle/2);


  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}