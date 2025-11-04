#include <Arduino.h>
#include "Driver.hpp"
#include "UART.hpp"

#define CENTRAL_BOARD_ON true

unsigned long previousTsMove = 0;
int sampleTimeControl = parameters::sampletimeControl;
float wheelTarget0 = 0;
float wheelTarget1 = 0; 

void setup(){

  Driver::setup(parameters::velocityMode, true, true);
 
  #if CENTRAL_BOARD_ON
    UART::setup();
  #endif

}

void loop(){
  unsigned long actualTimestamp = millis();

  #if CENTRAL_BOARD_ON
    UART::receiveMessage(&wheelTarget0,&wheelTarget1);
  #endif

  if(actualTimestamp - previousTsMove >= sampleTimeControl){
    
    Driver::executeMotor(wheelTarget0, wheelTarget1);

    actualTimestamp = previousTsMove;
  }

}