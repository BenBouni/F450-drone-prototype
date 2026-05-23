#include <Arduino.h>
#include<RF24.h>

struct received {
  int a;
  int b;
};

struct sent {
  int c;
  int d;
};
sent packetSent;
received packetReceived;

class Radio{
    private:
    RF24 radio;
    uint8_t txAddress[6] ;               // pipe used when
    uint8_t rxAddress[6]  ;               // pipe used when listening
    int CE_PIN ;
    int CSN_PIN ;
    unsigned long interval; // sending interval in ms
    unsigned long dernierEnvoi;
    
    public:
        Radio(const int interval, const int CE_PIN, const int CSN_PIN, const uint8_t tx[6], const uint8_t rx[6]) :
         interval(interval), CE_PIN(CE_PIN), CSN_PIN(CSN_PIN), radio(CE_PIN, CSN_PIN) {
            memcpy(txAddress, tx, 6);
            memcpy(rxAddress, rx, 6);
         }
        
        void begin() {
            if (!radio.begin()) {
                Serial.println("Radio hardware not responding!");
                while (1) {} // halt if radio is not working
            }
            radio.begin();
            radio.openReadingPipe(1, rxAddress);
            radio.openWritingPipe(txAddress);
            radio.setPALevel(RF24_PA_LOW);
            radio.startListening();
            dernierEnvoi = millis();
        }
        void receivePacket(received& Packet) {
            if (radio.available()) {
                radio.read(&Packet, sizeof(Packet));
            }
        }
        void sendPacket(const sent& packetSent) {
            radio.stopListening();
            radio.write(&packetSent, sizeof(packetSent));
            radio.startListening();
        }
        
        void altrnateSend(const sent& packetSent, received& packetReceived) {
            unsigned long currentMillis = millis();
            if (currentMillis - dernierEnvoi >= interval) {
                sendPacket(packetSent);
                dernierEnvoi = currentMillis;
            } 
                receivePacket(packetReceived);
            
        }
        };
