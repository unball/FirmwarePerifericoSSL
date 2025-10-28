/* Params for MKS ESP32 DUAL FOC V1.0
*/

#ifndef CONSTANTS_DRIVER_1_H
#define CONSTANTS_DRIVER_1_H

namespace motor1 {

    // BLDC Motor
    constexpr int DRIVER_PINOUT_PHASE_A = 26;
    constexpr int DRIVER_PINOUT_PHASE_B = 27;
    constexpr int DRIVER_PINOUT_PHASE_C = 14;
    constexpr int DRIVER_PINOUT_ENABLE = 12;

    // Params for current sensor INA1818A
    constexpr float CURRENT_SENSE_RESISTOR = 0.01f;
    constexpr float CURRENT_SENSE_GAIN = 50.0f;
    constexpr float CURRENT_SENSE__PINOUT_PHASE_A = 35;
    constexpr float CURRENT_SENSE__PINOUT_PHASE_B = 34;
    constexpr int CURRENT_SENSE_GAIN_B = -1;

    // Limits for GM4108H120T
    constexpr float DRIVER_VOLTAGE_POWER_SUPPLY = 12;
    constexpr float DRIVER_VOLTAGE_LIMIT = 12;

    // Params for I2C - AS50600 (encoder)
    constexpr int I2C_BUS_NUMBER = 1;
    constexpr int I2C_PINOUT_SDA = 23;
    constexpr int I2C_PINOUT_SCL = 5;
    constexpr uint32_t I2C_PINOUT_FREQUENCY = 400000UL;
    
}

#endif // CONSTANTS_DRIVER_1_H