#ifndef CONSTANTS_TORQUE_PID_H
#define CONSTANTS_TORQUE_PID_H

namespace torquePID {

    // DC Current mode -> Iq
    constexpr float P_iq = 25;
    constexpr float I_iq = 10000; 
    constexpr float D_iq = -0.001;
    constexpr float Tf_iq = 0.01;
    
}

#endif // CONSTANTS_TORQUE_PID_H