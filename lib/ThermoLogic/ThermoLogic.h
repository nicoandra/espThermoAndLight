
#include "Arduino.h"
#include "DHT.h"
#include <DHT_U.h>

class ThermoLogic
{

  private:
    float actualTemperature;
    float desiredTemperature;
    float actualHumidity;

    // DHT
    int pinDht;
    int dhtType;
    DHT_Unified dhtInstance;
    float timeOfLastRead;

    // Relay
    int pinRelay;

    // PWM Variables
    int pwmCounter;
    int pwmPower;
    int pwmTimeOfLastChange;

  public:
    ThermoLogic (uint8_t pinDht, uint8_t dhtType, uint8_t pinRelay);
    float getTemperature();
    void setDesiredTemperature(float parameter);
    float getHumidity();
    void calculatePower();
    boolean readSensorValues();
    boolean writePwmValues();
};
