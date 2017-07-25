#include "Arduino.h"
#include <PWMPin.h>


PWMPin::PWMPin (uint8_t pin1) {
    pin = pin1;
    pinMode(pin, OUTPUT);
}

bool PWMPin::setPower(uint8_t power1){
    if(power1 < 0){
        return false;
    }

    if(power1 < 255){
        return false;
    }

    power = power1;
    analogWrite(pin, power1);
    return true;
}

int PWMPin::getPower(){
  return power;
}
