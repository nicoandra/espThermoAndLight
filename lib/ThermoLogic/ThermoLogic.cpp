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

void ThermoLogic::setDesiredTemperature(float parameter){
  desiredTemperature = parameter;
}

boolean ThermoLogic::readSensorValues(){

  if(millis() < timeOfLastRead){
    // Reset the timer. It happens once every 4 or 5 days
    timeOfLastRead = 0;
  }

  if(timeOfLastRead + 30000 > millis()){
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
  return true;

}

void ThermoLogic::calculatePower(){

  if(false){
    // Hardcoded to test the output pin
    pwmPower = 5;
    return;
  }

  if(actualTemperature - desiredTemperature < .5){
    // Desired temperature is 1 degree below the actual temperature. Full power
    // Serial.println("Setting pwmPower to 10");
    pwmPower = 10;
    return ;
  }

  if(actualTemperature - desiredTemperature > .5){
    // The current temperature is above the desired one. Set power to 0.
    // Serial.println("Setting pwmPower to 0");
    pwmPower = 0;
    return ;
  }

  // When between, set power to 50%
  // Serial.println("Setting pwmPower to 5");
  pwmPower = 5;
  return ;
}


boolean ThermoLogic::writePwmValues(){
  if(pwmTimeOfLastChange > millis()){
    pwmTimeOfLastChange = 0;
  }

  if(pwmTimeOfLastChange + 2000 > millis()){
    // Not a second passed yet, skip
    return false;
  }

  pwmTimeOfLastChange = millis();

  pwmCounter++;


  Serial.print(desiredTemperature);
  Serial.print("@ vs ");
  Serial.print(actualTemperature);
  Serial.print("@ ||| ");
  Serial.print(pwmCounter);
  Serial.print(" vs ");
  Serial.print(pwmPower);
  Serial.print(" = ");

  if(pwmCounter > pwmPower){
    digitalWrite(pinRelay, LOW);
    Serial.println(LOW);
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    Serial.println(HIGH);
    digitalWrite(pinRelay, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if(pwmCounter == 10){
    // Reset counter
    pwmCounter = 0;
  }

  return true;
}
