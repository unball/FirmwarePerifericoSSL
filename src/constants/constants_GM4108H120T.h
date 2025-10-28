#ifndef CONSTANTS_GM4108_120T_H
#define CONSTANTS_GM4108_120T_H

namespace gm4108h120T {
    
    // Params for motor GM4108-120T
    constexpr int NUMBER_POLES = 11;
    constexpr int PHASE_RESISTANCE = 6.75;

    // Limits for FOC execution
    constexpr float MOTOR_CURRENT_LIMIT = 1.1;
    constexpr float MOTOR_VOLTAGE_LIMIT = 12;
    constexpr float MOTOR_VELOCITY_LIMIT = 50;
}

#endif // CONSTANTS_GM4108_120T_H