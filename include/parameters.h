#ifndef PARAMETERS_H
#define PARAMETERS_H

namespace parameters {
    
    // Control types
    constexpr uint8_t torqueVoltageMode = 0;
    constexpr uint8_t torqueDCCurrentMode = 1;
    constexpr uint8_t velocityMode = 2;

    // Sample time for control
    constexpr int sampletimeControl = 5; // ms

    // I2C frequency
    constexpr uint32_t I2C_PINOUT_FREQUENCY = 400000UL;

    // I2C BUS Motor 1
    constexpr int I2C_BUS_NUMBER_1 = 1;

    // I2C BUS Motor 0
    constexpr int I2C_BUS_NUMBER_0 = 0;

    // Current sensor
    constexpr float CURRENT_SENSE_RESISTOR = 0.01f;
    constexpr float CURRENT_SENSE_GAIN = 50.0f;
    constexpr int CURRENT_SENSE_GAIN_B = -1;

    // Limits for GM4108H120T
    constexpr float VOLTAGE_POWER_SUPPLY = 12;
    constexpr float VOLTAGE_LIMIT = 12;

    // DC Current mode -> Iq
    constexpr float P_torque = 25;
    constexpr float I_torque = 10000; 
    constexpr float D_torque = -0.001;
    constexpr float Tf_torque = 0.01;

    // Velocity PID
    constexpr float P_vel = 0.01;
    constexpr float I_vel = 1;
    constexpr float D_vel = -0.00001;
    constexpr float Tf_vel = 0.001;
    constexpr float output_ramp_vel = 1000;

    // Params for motor GM4108-120T
    constexpr int NUMBER_POLES = 11;
    constexpr float PHASE_RESISTANCE = 6.75;

    // Limits for FOC execution
    constexpr float MOTOR_CURRENT_LIMIT = 1.1;
    constexpr float MOTOR_VOLTAGE_LIMIT = 12;
    constexpr float MOTOR_VELOCITY_LIMIT = 50;
    
}

#endif // PARAMETERS_H