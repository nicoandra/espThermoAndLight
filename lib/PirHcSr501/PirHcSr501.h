#include "Arduino.h"

class PinHcSr501
{

  private:
    long lastMovementDetectedAt;
    int pin;
    long delayBetweenMovements; // In milliseconds
    int lastStatus;

  public:
    PinHcSr501 (int pin);
    bool movementDetected();
    void detectDelay();
    void setDelay(long delay);
};
