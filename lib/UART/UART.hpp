#ifndef UART_H
#define UART_H

#include <Arduino.h>
#include "../../include/parameters.h"
#include "../../include/pins.h"

namespace UART
{   
    void setup();
    void receiveMessage(float* wheelTarget0, float* wheelTarget1 );

    static HardwareSerial mySerial(1);
    static String uartBuffer = "";
}

#endif // UART_H