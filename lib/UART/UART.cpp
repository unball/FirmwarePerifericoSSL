#include "UART.hpp"

namespace UART
{   
    void setup(){
        mySerial.begin(9600,SERIAL_8N1,pins::UART_RX,pins::UART_TX);
        while(!mySerial);
    }

    void receiveMessage(float* wheelTarget0, float* wheelTarget1 ){
        while (mySerial.available()) {
            char c = mySerial.read();

            if (c == '\n') {
                uartBuffer.trim();

                if (uartBuffer.startsWith("D0") || uartBuffer.startsWith("D1")) {
                    String msg = uartBuffer.substring(2);
                    int commaIndex = msg.indexOf(',');

                    if (commaIndex > 0) {
                        float x = msg.substring(0, commaIndex).toFloat();
                        float y = msg.substring(commaIndex + 1).toFloat();

                        *wheelTarget0 = x;
                        *wheelTarget1 = y;

                    }
                }

                uartBuffer = "";  

            } 
            else {
                uartBuffer += c;
            }
        }

    }
}
