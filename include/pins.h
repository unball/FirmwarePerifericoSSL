#ifndef PINS_H
#define PINS_H

namespace pins {

    // UART pinout to communicate with Central Board
    constexpr uint8_t UART_TX = 13;
    constexpr uint8_t UART_RX = 15; 

    // BLDC Motor 1
    constexpr uint8_t DRIVER_PINOUT_PHASE_A_1 = 26;
    constexpr uint8_t DRIVER_PINOUT_PHASE_B_1 = 27;
    constexpr uint8_t DRIVER_PINOUT_PHASE_C_1 = 14;
    constexpr uint8_t DRIVER_PINOUT_ENABLE_1 = 12;

    // I2C Motor 1 to encoder AS5600
    constexpr uint8_t I2C_PINOUT_SDA_1 = 23;
    constexpr uint8_t I2C_PINOUT_SCL_1 = 5;

    // Current Sensor Motor 1
    constexpr uint8_t CURRENT_SENSE_PINOUT_PHASE_A_1 = 35;
    constexpr uint8_t CURRENT_SENSE_PINOUT_PHASE_B_1 = 34;

    // BLDC Motor 0
    constexpr uint8_t DRIVER_PINOUT_PHASE_A_0 = 32;
    constexpr uint8_t DRIVER_PINOUT_PHASE_B_0 = 33;
    constexpr uint8_t DRIVER_PINOUT_PHASE_C_0 = 25;
    constexpr uint8_t DRIVER_PINOUT_ENABLE_0 = 22;

    // Current Sensor Motor 0
    constexpr uint8_t CURRENT_SENSE_PINOUT_PHASE_A_0 = 39;
    constexpr uint8_t CURRENT_SENSE_PINOUT_PHASE_B_0 = 36;

    // I2C Motor 0 to encoder AS5600
    constexpr uint8_t I2C_PINOUT_SDA_0 = 19;
    constexpr uint8_t I2C_PINOUT_SCL_0 = 18;
}

#endif // PINS_H