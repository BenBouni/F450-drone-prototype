#ifndef RADIO_H
#define RADIO_H
#include <RF24.h>
#include "data.h"
#include "motorLOGIC.h"

typedef ControlData received;
typedef DroneData sent;


class Emitor_receptor {
  private:
    RF24 radio;
    uint8_t txAddress[6];               // pipe used when transmitting
    uint8_t rxAddress[6];               // pipe used when listening
    int CE_PIN ;
    int CSN_PIN ;
    unsigned long interval; // sending interval in ms
        unsigned long dernierEnvoi;
    
    public:
        Emitor_receptor(const int interval, const int CE_PIN, const int CSN_PIN, const uint8_t tx[6], const uint8_t rx[6]) :
          interval(interval), CE_PIN(CE_PIN), CSN_PIN(CSN_PIN) {
            memcpy(txAddress, tx, 6);
            memcpy(rxAddress, rx, 6);
          }
        
        void begin(); 
        bool receivePacket(received& Packet);
        void sendPacket(const sent& packetSent);       
        void alternateSend(const sent& packetSent, received& packetReceived);
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

