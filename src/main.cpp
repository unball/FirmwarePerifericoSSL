// Documentação: https://docs.simplefoc.com/magnetic_sensor_spi

#include <SimpleFOC.h>

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
//  cs              - SPI chip select pin 
//  bit_resolution - magnetic sensor resolution -> on AS5048A is 14 bit 
//  angle_register  - (optional) angle read register - default 0x3FFF
MagneticSensorSPI sensor = MagneticSensorSPI(5, 14, 0x3FFF);

// TODO: calibrar motor -> https://docs.simplefoc.com/bldcmotor
BLDCMotor motor = BLDCMotor(11);
// TODO: verificar como usar o driver -> https://docs.simplefoc.com/drivers_config
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

//Target variable
float target_velocity = 0;

//Serial command setting
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup(){
    // driver
    driver.init();
    motor.linkDriver(&driver);

    // init magnetic sensor hardware
    sensor.init();
    motor.linkSensor(&sensor);

    // init motor hardware
    motor.init();
    motor.initFOC();

    Serial.println("Motor ready");
    _delay(1000);


    //Add T command
    command.add('T', doTarget, "target velocity");

    Serial.begin(115200);
    Serial.println("Motor ready!");
    Serial.println("Set target velocity [rad/s]");
    _delay(1000);
}


void loop(){
    motor.loopFOC();
    motor.move(25);

    Serial.println(sensor.getVelocity());

    //User newsletter
    command.run();
}