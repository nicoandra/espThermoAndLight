
#include "Arduino.h"
#include "DHT.h"
#include <DHT_U.h>

class ThermoLogic
{

  private:
    float actualTemperature;
    float desiredTemperature = 19;
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
    bool setDesiredTemperature(float parameter);
    float getDesiredTemperature();
    float getHumidity();
    void calculatePower();
    int getPower();
    boolean readSensorValues();
    boolean writePwmValues();
};
