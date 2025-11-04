#ifndef DRIVER0_H
#define DRIVER0_H

#include <SimpleFOC.h>
#include <Arduino.h>
#include "../../include/parameters.h"
#include "../../include/pins.h"

namespace Driver
{   
    void setup(uint8_t controlType, boolean useMotor0, boolean useMotor1);
    void executeMotor(float targetMotor0, float targetMotor1 );
    void setupTypeControl(uint8_t controlType, BLDCMotor* motor);

    static boolean executeMotor0 = false;
    static boolean executeMotor1 = false;    

}

#endif // DRIVER0_H