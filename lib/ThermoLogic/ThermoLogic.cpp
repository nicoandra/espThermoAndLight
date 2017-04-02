#include "Arduino.h"
#include "DHT.h"
#include <DHT_U.h>
#include <ThermoLogic.h>

ThermoLogic::ThermoLogic(uint8_t pinDhtParam, uint8_t dhtTypeParam, uint8_t pinRelayParam) : dhtInstance(pinDhtParam, dhtTypeParam) {
  pinDht = pinDhtParam;
  dhtType = dhtTypeParam;
  pinRelay = pinRelayParam;
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

bool ThermoLogic::readSensorValues(){

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
    actualTemperature = 900;
  } else {
    actualTemperature = event.temperature;
  }


  dhtInstance.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity");
    actualHumidity = 900
    ;
  } else {
    actualHumidity = event.relative_humidity;
  }


  // Serial.println("Sensor: values read.");
  return true;

}

void ThermoLogic::calculatePower(){

  float diff = (actualTemperature - 0.1) - desiredTemperature;

  if(diff < -0.1){
    // The actual temperature is below the desired one.
    // For a negative result, full power.
    pwmPower = 10;
    return ;
  }

  if(-0.1 <= diff && diff <= .2){
    // We're in the .5 window
    pwmPower = 5;
    return;
  }

  pwmPower = 0;
  return ;
}


bool ThermoLogic::writePwmValues(){
  if(pwmTimeOfLastChange > millis()){
    pwmTimeOfLastChange = 0;
  }

  if(pwmTimeOfLastChange + 1000 > millis()){
    // Not a second passed yet, skip
    return false;
  }

  pwmTimeOfLastChange = millis();

  pwmCounter++;

  int onOff;

  if((int) pwmCounter > (int) pwmPower){
    // Inversed because of the weird relays... sorry
    onOff = HIGH;
  } else {
    onOff = LOW;
  }

  digitalWrite(pinRelay, onOff);

  if(pwmCounter == 10){
    // Reset counter
    pwmCounter = 0;
  }

  return true;
}

void ThermoLogic::printValues(){
    Serial.print("PinRelay: ");
    Serial.print(pinRelay);
    Serial.print(" PinDHT: ");
    Serial.print(pinDht);
    Serial.print(" values: Temp: ");
    Serial.print(getTemperature());
    Serial.print("*C , Humid: ");
    Serial.print(getHumidity());
    Serial.print(" Power: ");
    Serial.println(getPower());
  }
