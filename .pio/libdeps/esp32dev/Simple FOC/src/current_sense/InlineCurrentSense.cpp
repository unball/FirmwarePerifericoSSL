#include "InlineCurrentSense.h"
#include "communication/SimpleFOCDebug.h"
// InlineCurrentSensor constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin (optional)
InlineCurrentSense::InlineCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC){
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    shunt_resistor = _shunt_resistor;
    amp_gain  = _gain;
    volts_to_amps_ratio = 1.0f /_shunt_resistor / _gain; // volts to amps
    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
};


InlineCurrentSense::InlineCurrentSense(float _mVpA, int _pinA, int _pinB, int _pinC){
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    volts_to_amps_ratio = 1000.0f / _mVpA; // mV to amps
    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
};



// Inline sensor init function
int InlineCurrentSense::init(){
    // if no linked driver its fine in this case 
    // at least for init()
    void* drv_params = driver ? driver->params : nullptr;
    // configure ADC variables
    params = _configureADCInline(drv_params,pinA,pinB,pinC);
    // if init failed return fail
    if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0; 
    // set the center pwm (0 voltage vector)
    if(driver_type==DriverType::BLDC)
        static_cast<BLDCDriver*>(driver)->setPwm(driver->voltage_limit/2, driver->voltage_limit/2, driver->voltage_limit/2);
    // calibrate zero offsets
    calibrateOffsets();
    // set zero voltage to all phases
    if(driver_type==DriverType::BLDC)
        static_cast<BLDCDriver*>(driver)->setPwm(0,0,0);
    // set the initialized flag
    initialized = (params!=SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
    // return success
    return 1;
}
// Function finding zero offsets of the ADC
void InlineCurrentSense::calibrateOffsets(){
    const int calibration_rounds = 1000;
    
    // find adc offset = zero current voltage
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // read the adc voltage 1000 times ( arbitrary number )
    for (int i = 0; i < calibration_rounds; i++) {
        if(_isset(pinA)) offset_ia += _readADCVoltageInline(pinA, params);
        if(_isset(pinB)) offset_ib += _readADCVoltageInline(pinB, params);
        if(_isset(pinC)) offset_ic += _readADCVoltageInline(pinC, params);
        _delay(1);
    }
    // calculate the mean offsets
    if(_isset(pinA)) offset_ia = offset_ia / calibration_rounds;
    if(_isset(pinB)) offset_ib = offset_ib / calibration_rounds;
    if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

float filterCurrent(const int pinA,const void* params, float offset, float gain){
    int N = 10;

    float leitura_adc[N] = {0};
    float leitura_soma[N] = {0};
    float saida_soma[N] = {0};
    float saida_filtrada[N] = {0};
    // float coef_num[N] = {0.0000,0.0001,0.0003,0.0008,0.0014,0.0016,0.0014,0.0008,0.0003,0.0001,0.0000};
    // float coef_den[N] = {1.0000,-5.2999,13.3210,-20.6470,21.6982,-16.0753, 8.4719,-3.1273, 0.7721,-0.1149, 0.0078};

    // float coef_num[N] = {   0.1894,0.9472,1.8944,1.8944,0.9472,0.1894};
    // float coef_den[N] = {  1.0000,1.8688,1.8665,0.9985,0.2926,0.0358};

    float coef_num[N] = {0.0104,0.0934,0.3734,0.8714,1.3071,1.3071,0.8714,0.3734,0.0934,0.0104};
    float coef_den[N] = { 1.0000,0.9230,1.5538,0.8865,0.6377,0.2178,0.0768,0.0136,0.0019,0.0001};



    for(int i = N-1; i >= 0; i--){
        leitura_adc[i] = (_readADCVoltageInline(pinA, params) - offset)*gain;
    }

    for(int i = 0; i < N; i++){
        leitura_soma[i] += coef_num[i]*leitura_adc[i];

        if(i != 0){
            saida_soma[i] += coef_den[i]*saida_filtrada[i-1];
        }

        saida_filtrada[i] += leitura_soma[i] - saida_soma[i];
    }

    return saida_filtrada[N-1];
}

// read all three phase currents (if possible 2 or 3)
PhaseCurrent_s InlineCurrentSense::getPhaseCurrents(){
    PhaseCurrent_s current;

    if(isAlign){
        current.a = (!_isset(pinA)) ? 0 : filterCurrent(pinA, params, offset_ia, gain_a);// amps
        current.b = (!_isset(pinB)) ? 0 : filterCurrent(pinB, params, offset_ib, gain_b);// amps
        current.c = (!_isset(pinC)) ? 0 : filterCurrent(pinC, params, offset_ic, gain_c); // amps
    }else{
        current.a = (!_isset(pinA)) ? 0 : (_readADCVoltageInline(pinA, params) - offset_ia)*gain_a;// amps
        current.b = (!_isset(pinB)) ? 0 : (_readADCVoltageInline(pinB, params) - offset_ib)*gain_b;// amps
        current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageInline(pinC, params) - offset_ic)*gain_c; // amps
    }

    return current;
}
