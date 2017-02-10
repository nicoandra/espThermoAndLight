#include "Arduino.h"
#include "DHT.h"
#include <DHT_U.h>
#include <ThermoLogic.h>

ThermoLogic::ThermoLogic(uint8_t pinDht, uint8_t dhtType, uint8_t pinRelay) : dhtInstance(pinDht, dhtType) {
  pinMode(pinRelay, OUTPUT);
  digitalWrite(pinRelay, HIGH);
};

float ThermoLogic::getTemperature() {
  return actualTemperature;
};

float ThermoLogic::getHumidity() {
  return actualHumidity;
};

float ThermoLogic::getDesiredTemperature(){
  return desiredTemperature;
}

bool ThermoLogic::setDesiredTemperature(float parameter){
  desiredTemperature = parameter;
  ThermoLogic::calculatePower();
  return true;
}

int ThermoLogic::getPower(){
  return pwmPower;
}

boolean ThermoLogic::readSensorValues(){

  if(millis() < timeOfLastRead){
    // Reset the timer. It happens once every 4 or 5 days
    timeOfLastRead = 0;
  }

  if(timeOfLastRead + 10000 > millis()){
    // Not read. Read has been done already
    return false;
  }


  timeOfLastRead = millis();
  sensors_event_t event;
  dhtInstance.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading humidity");
    return false;
  }
  actualTemperature = event.temperature;

  dhtInstance.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity");
    return false;
  }

  actualHumidity = event.relative_humidity;
  // Serial.println("Sensor: values read.");
  return true;

}

void ThermoLogic::calculatePower(){

  float diff = (actualTemperature - 0.1) - desiredTemperature;

  if(diff < 0){
    // The actual temperature is below the desired one.
    // For a negative result, full power.
    pwmPower = 10;
    return ;
  }

  if(0 <= diff && diff <= .3){
    // We're in the .5 window
    pwmPower = 5;
    return;
  }

  pwmPower = 0;
  return ;
}


boolean ThermoLogic::writePwmValues(){
  if(pwmTimeOfLastChange > millis()){
    pwmTimeOfLastChange = 0;
  }

  if(pwmTimeOfLastChange + 500 > millis()){
    // Not a second passed yet, skip
    return false;
  }

  pwmTimeOfLastChange = millis();

  pwmCounter++;


  /*
  Serial.print(desiredTemperature);
  Serial.print("@ (des) vs ");
  Serial.print(actualTemperature);
  Serial.print("@ (cur) ||| Is ");
  Serial.print(pwmCounter);
  Serial.print(" (loop) > ");
  Serial.print(pwmPower);
  Serial.print("(pow) ? ");
  */

  int onOff;

  if((int) pwmCounter > (int) pwmPower){
    // Inversed because of the weird relays... sorry
    onOff = HIGH;
  } else {
    onOff = LOW;
  }

  digitalWrite(pinRelay, onOff);
  digitalWrite(LED_BUILTIN, onOff);
  // Serial.println(onOff);


  if(pwmCounter == 10){
    // Reset counter
    pwmCounter = 0;
  }

  return true;
}
