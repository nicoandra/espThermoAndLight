
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
    ThermoLogic (uint8_t pinDhtParam, uint8_t dhtTypeParam, uint8_t pinRelayParam);
    float getTemperature();
    bool setDesiredTemperature(float parameter);
    float getDesiredTemperature();
    float getHumidity();
    void calculatePower();
    int getPower();
    bool readSensorValues();
    bool writePwmValues();
    void printValues();
};
