#ifndef CONSTANTS_VELOCITY_PID_H
#define CONSTANTS_VELOCITY_PID_H

namespace velocityPID {

    constexpr float P_vel = 0.01;
    constexpr float I_vel = 1;
    constexpr float D_vel = -0.00001;
    constexpr float Tf_vel = 0.001;

    // Parametro para deixar a resposta mais rápida e suave
    constexpr float output_ramp_vel = 1000;
    
}

#endif // CONSTANTS_VELOCITY_PID_H