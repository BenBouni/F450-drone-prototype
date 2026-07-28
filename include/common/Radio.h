#ifndef RADIO_H
#define RADIO_H
#include <RF24.h>
#include "data.h"


template <typename sent, typename received>
class Emitor_receptor {
  private:

    RF24 radio;
    uint8_t txAddress[6];               // pipe used when transmitting
    uint8_t rxAddress[6];               // pipe used when listening
    int CE_PIN ;
    int CSN_PIN ;
    unsigned long interval; // sending interval in ms
    unsigned long dernierEnvoi;
    bool isListening = false;
    
    public:
        Emitor_receptor(const unsigned long interval, const int CE_PIN, const int CSN_PIN, const uint8_t tx[6], const uint8_t rx[6]) : 
                interval(interval), CE_PIN(CE_PIN), CSN_PIN(CSN_PIN), radio(CE_PIN, CSN_PIN) {
          memcpy(txAddress, tx, 6);
          memcpy(rxAddress, rx, 6);
      }

        void begin() {
            if (!radio.begin()) {
                Serial.println("Radio hardware not responding!");
                // Do not block forever here; allow system to continue in degraded mode.
                // The caller should handle missing radio (updated flag / failsafe will trigger).
                return;
            }
            radio.openReadingPipe(1, rxAddress);
            radio.openWritingPipe(txAddress);
            radio.setPALevel(RF24_PA_LOW);
            radio.startListening();
            isListening = true;
            dernierEnvoi = millis();
        }
        bool receivePacket(received& Packet) {
            if (radio.available()) {
                radio.read(&Packet, sizeof(Packet));
                return true;
            }
            return false;
        }
        void sendPacket(const sent& packetSent) {
            radio.stopListening();
            isListening = false;
            radio.write(&packetSent, sizeof(packetSent));
        }
        
        void alternateSend(const sent& packetSent, received& packetReceived) {
            unsigned long currentMillis = millis();
            if (currentMillis - dernierEnvoi >= interval) {
                sendPacket(packetSent);
                dernierEnvoi = currentMillis;
            } 
            if (isListening == false) {
                radio.txStandBy();
                radio.startListening();
                isListening = true;
            }
                receivePacket(packetReceived);
                
            
        }
        };

// On the drone we send DroneData (telemetry) and receive ControlData (commands from GCS)
extern Emitor_receptor<DroneData, ControlData> radio;



#endif 

