#ifndef Failsafe_h
#define Failsafe_h
#include "data.h"
#include "motorLOGIC.h"

class failsafe {
  private:
    unsigned long lastSignalTime;
    const unsigned long timeoutmin; // in milliseconds
    const unsigned long timeoutmax ; // in milliseconds
   
  public:
    failsafe(unsigned long timeoutmin, unsigned long timeoutmax);
    void updateSignalTime();
    bool temporaryLoss();
    bool criticalLoss();
    void failsafeAction(mixMotor& mixer);
};

extern failsafe monFailsafe;

#endif 