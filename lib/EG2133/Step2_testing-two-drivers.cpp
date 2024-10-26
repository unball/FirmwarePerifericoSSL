#include <SimpleFOC.h>
// BLDC driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_27);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27);

void setup()
{
    // Driver
    // pwm frequency to be used [Hz]
    driver.pwm_frequency = 20000;
    // power supply voltage [V]
    driver.voltage_power_supply = 12;
    // Max DC voltage allowed - default voltage_power_supply
    driver.voltage_limit = 12;

    // driver init
    driver.init();

    // enable driver
    driver.enable();

    // Driver 1

    // pwm frequency to be used [Hz]
    driver1.pwm_frequency = 20000;
    // power supply voltage [V]
    driver1.voltage_power_supply = 12;
    // Max DC voltage allowed - default voltage_power_supply
    driver1.voltage_limit = 12;

    // driver init
    driver1.init();

    // enable driver
    driver1.enable();

    _delay(1000);
}

void loop()
{
    // setting pwm (A: 3V, B: 1V, C: 5V)
    // setting pwm1 (A: 5V, B: 3V, C: 1V)
    // When measuring the voltage on each phase with a multimeter you should get these values
    driver.setPwm(3, 1, 5);
    driver1.setPwm(5, 3, 1);
}
