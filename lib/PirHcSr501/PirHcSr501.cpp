#include "Arduino.h"
#include <PirHcSr501.h>

PinHcSr501::PinHcSr501(int pinParameter) {
  pin = pinParameter;
  pinMode(pin, INPUT);
  digitalWrite(pin, LOW);
};


bool PinHcSr501::movementDetected(){
  long now = millis();

  if(now < lastMovementDetectedAt){
    // Time rollover fix.
    lastMovementDetectedAt = 0;
  }

  if(now - lastMovementDetectedAt < delayBetweenMovements){
    return false;
  }

  if(digitalRead(pin)){
    lastMovementDetectedAt = millis();
    return true;
  }
  return false;
}

void PinHcSr501::setDelay(long delay){
  delayBetweenMovements = delay;
}

void PinHcSr501::detectDelay(){
  if(lastStatus != digitalRead(pin)){
    lastStatus = digitalRead(pin);

    long now = millis();

    Serial.print(lastStatus);
    Serial.print(" ");
    Serial.print(pin);
    Serial.print(": ");
    Serial.print("Delay seems to be ");
    Serial.print(now - lastMovementDetectedAt);
    Serial.println("ms.");
    lastMovementDetectedAt = now;
  }
}
