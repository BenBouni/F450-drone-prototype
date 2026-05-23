#include "Failsafe.h"

// failsafe implementation 

    failsafe::failsafe(unsigned long timeoutmin, unsigned long timeoutmax) : timeoutmin(timeoutmin), timeoutmax(timeoutmax) {
      lastSignalTime = millis();
    }

    void failsafe::updateSignalTime() { // to be called when a valid signal is received
        lastSignalTime = millis();
    }

    bool failsafe::temporaryLoss() {
      if (millis() - lastSignalTime > timeoutmin)
      return true;
      return false;
    } 
    bool failsafe::criticalLoss() {
      if (millis() - lastSignalTime > timeoutmax)
      return true;
      return false;
    }
    void failsafe::failsafeAction(mixMotor& mixer) {
      if (criticalLoss() ) {
        mixer.stopTout();
      } else if (temporaryLoss() ) {
        if (xSemaphoreTake(xMutexControll, 0) == pdTRUE){
          order.thr = 1300.0f; // reduce throttle to a low value to allow for a safe landing if the signal comes back, but cut the PID corrections to avoid any sudden movements of the drone in case of abberant mesures or code blocking, and to allow for a safe landing if the signal comes back.
          order.pitch = 0.0f;
          order.roll = 0.0f;
          order.yaw = 0.0f;
          order.armed = false; // Disarm the drone
          xSemaphoreGive(xMutexControll);
        }; // stabilisation incase of temporary loss of signal, to avoid a crash while waiting for the signal to come back, by cutting the throttle and disarming the drone, but keeping the PID corrections to 0 to avoid any sudden movements of the drone in case of abberant mesures or code blocking, and to allow for a safe landing if the signal comes back.
    } else {
        // normal operation, do nothing
      }
    }