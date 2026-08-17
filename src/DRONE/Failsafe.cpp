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
      if (monIMU.IMU_flag) {
        mixer.stopTout();
        if (xSemaphoreTake(xMutexControll, pdMS_TO_TICKS(10)) == pdTRUE) {
          order.armed.store(false);
          order.thr.store(1000.0f);
          order.pitch.store(0.0f);
          order.roll.store(0.0f);
          order.yaw.store(0.0f);
          xSemaphoreGive(xMutexControll);
        }
        return;
      }

      if (criticalLoss() ) {
        mixer.stopTout();
        if (xSemaphoreTake(xMutexControll, pdMS_TO_TICKS(10)) == pdTRUE) {
          order.armed.store(false);
          order.thr.store(1000.0f);
          xSemaphoreGive(xMutexControll);
        }
      } else if (temporaryLoss() ) {
        mixer.stopTout();
        if (xSemaphoreTake(xMutexControll, pdMS_TO_TICKS(10)) == pdTRUE){
          order.thr.store(1000.0f);
          order.pitch.store(0.0f);
          order.roll.store(0.0f);
          order.yaw.store(0.0f);
          order.armed.store(false);
          xSemaphoreGive(xMutexControll);
        }
    } else {
        // normal operation, do nothing
      }
    }
