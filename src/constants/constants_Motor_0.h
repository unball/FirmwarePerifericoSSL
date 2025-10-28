/* Params for MKS ESP32 DUAL FOC V1.0
*/

#ifndef CONSTANTS_DRIVER_0_H
#define CONSTANTS_DRIVER_0_H

namespace motor0 {

    // BLDC Motor
    constexpr int DRIVER_PINOUT_PHASE_A = 32;
    constexpr int DRIVER_PINOUT_PHASE_B = 33;
    constexpr int DRIVER_PINOUT_PHASE_C = 25;
    constexpr int DRIVER_PINOUT_ENABLE = 22;

    // Params for current sensor INA1818A
    constexpr float CURRENT_SENSE_RESISTOR = 0.01f;
    constexpr float CURRENT_SENSE_GAIN = 50.0f;
    constexpr float CURRENT_SENSE_PINOUT_PHASE_A = 39;
    constexpr float CURRENT_SENSE_PINOUT_PHASE_B = 36;
    constexpr int CURRENT_SENSE_GAIN_B = -1;

    // Limits for GM4108H120T
    constexpr float DRIVER_VOLTAGE_POWER_SUPPLY = 12;
    constexpr float DRIVER_VOLTAGE_LIMIT = 12;

    // Params for I2C - AS50600 (encoder)
    constexpr int I2C_BUS_NUMBER = 0;
    constexpr int I2C_PINOUT_SDA = 19;
    constexpr int I2C_PINOUT_SCL = 18;
    constexpr uint32_t I2C_PINOUT_FREQUENCY = 400000UL;
    
}

#endif // CONSTANTS_DRIVER_0_H