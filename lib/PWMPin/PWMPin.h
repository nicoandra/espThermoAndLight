
#include "Arduino.h"

class PWMPin
{

  private:
    int power;
    int pin;

  public:
    PWMPin (uint8_t pin1);
    int getPower();
    bool setPower(uint8_t power1);
};
