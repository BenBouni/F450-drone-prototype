#ifndef RADIO_H
#define RADIO_H
#include <RF24.h>
#include "data.h"
#include "motorLOGIC.h"


class Emitor_receptor { 
  private:
    RF24 radio;
    // five‑byte pipe address; RF24 expects a uint8_t pointer so use uint8_t explicitly
    const uint8_t address[6] = {'0','0','0','0','1','\0'};
    const unsigned long interval = 100; // sending interval in ms
    unsigned long dernierEnvoi;
    
  public:
    Emitor_receptor(uint8_t cePin, uint8_t csnPin);
    void begin();
    bool receivePacket(ControlData& Packet);
    bool sendPacket(const DroneData& packetDrone);
    void sendDroneData(const DroneData& packetDrone);
};

extern Emitor_receptor radio;

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

extern failsafe monFailsafe; // temporary delay / critical delay in ms, to be adjusted according to the expected signal loss duration and the desired failsafe behavior of the drone

class update_data {
  private :
  std::atomic <unsigned long> dernier_update {0};
  const unsigned long delai_update;
  std::atomic <bool> updated{false};
  public :
  update_data(unsigned long delaiupdate);
  void data(std::atomic <float> &THR,  std::atomic <float> &PITCH,  std::atomic <float> &ROLL,  std::atomic <float> &YAW,  std::atomic <bool> &ARMED);
  bool UPDATED() const;
};

extern update_data data;

#endif 

