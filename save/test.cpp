#include <Arduino.h>
#include <BLEGamepadClient.h>
#include <RF24.h>
#include <SPI.h>



RF24 radio( 4, 5); // CE, CSN pins
struct dataPacket {
    float thr;
    float yaw;
    float pitch;
    float roll;
    float armed;
};

struct dataPacketDrone {
    float batteryVoltage;
    float ActualPitch;
    float ActualRoll;
    float ActualYaw;
    float ActualThrottle;
};


class RadioEmitor_Receptor {
    private:
    const uint8_t channel;
    const uint64_t PipelineAddressTX;
    const uint64_t PipelineAddressRX;
    RF24 radio; // CE, CSN pins
public:
    RadioEmitor_Receptor(const uint8_t channel, const uint64_t PipelineAddressTX, const uint64_t PipelineAddressRX) : channel(channel), PipelineAddressTX(PipelineAddressTX), PipelineAddressRX(PipelineAddressRX), radio(4, 5) {}
    void begin() {
        radio.begin();
        radio.setPALevel(RF24_PA_HIGH);
        radio.setChannel(channel);
    }
    bool sendPacket(const dataPacket& packet) {
        radio.openWritingPipe(PipelineAddressTX);
        radio.stopListening();
        return radio.write(&packet, sizeof(packet));
    }
    bool radioWrite(const dataPacket& packet) {
        return radio.write(&packet, sizeof(packet));
    }
    bool receivePacket( dataPacketDrone& packetDrone) {
        radio.openReadingPipe(1, PipelineAddressRX);
        radio.startListening();
        return true;
    }
    bool radioRead( dataPacketDrone& packetDrone) {
        if (radio.available()) {
            radio.read(&packetDrone, sizeof(packetDrone));
            return true;
        }
        return false;
    }
};
RadioEmitor_Receptor* RADIO;

class RadioAlternateEmitor_Receptor {
    private:
    unsigned long lastSendTime = 0;
    unsigned long lastReceiveTime = 0;
    const unsigned long sendInterval = 50; // Intervalle d'envoi en millisecond
    const unsigned long receiveInterval = 200; // Intervalle de réception en millisecond
    RadioEmitor_Receptor& radio;
public:
    RadioAlternateEmitor_Receptor(RadioEmitor_Receptor& radio) : radio(radio) {}
    bool update( dataPacket& packet, dataPacketDrone& packetDrone) {
        unsigned long currentTime = millis();

        if (currentTime - lastSendTime >= sendInterval) {
            radio.sendPacket(packet);
            lastSendTime = currentTime;
        }
        if (radio.receivePacket(packetDrone)) {
            if (currentTime - lastReceiveTime >= receiveInterval) {
                radio.receivePacket(packetDrone);
                lastReceiveTime = currentTime;
                return true;
            }
        }
        return radio.radioRead(packetDrone);
    };
};

XboxController controller;

RadioAlternateEmitor_Receptor* radioAlternate;
dataPacket packet;
dataPacketDrone packetDrone;

void setup(void) {
  Serial.begin(115200);
  controller.begin();
  RADIO = new RadioEmitor_Receptor(100, 0xE8E8F0F0E1LL, 0xE8E8F0F0E2LL);
  RADIO->begin();
  radioAlternate = new RadioAlternateEmitor_Receptor(*RADIO);
}






void loop() {
  if (controller.isConnected()) {
    XboxControlsState s;
    controller.read(&s);
    Serial.println("controller connected");

// Map controller inputs to drone commands + prepare the packet to send 
    packet.thr = constrain((-1.0f * s.leftTrigger + s.rightTrigger) / (1.023f * 2) + 1500.0f, 1000.0f, 2000.0f);
    packet.yaw = constrain((s.rightStickX / 32768.0f) * 30, -30.0f, 30.0f);
    packet.pitch = constrain((s.rightStickY / 32768.0f) * 30, -30.0f, 30.0f);
    packet.roll = constrain((s.leftStickX / 32768.0f) * 30, -30.0f, 30.0f);
    packet.armed = s.xboxButton ? 1.0f : 0.0f;
    Serial.printf("roll: %.2f, pitch: %.2f, yaw: %.2f, thr: %.2f, armed: %.2f\n",
    s.leftStickX, s.leftStickY, s.rightStickX, s.rightStickY, s.leftTrigger, s.rightTrigger);
    // Send and receive packets using the alternate radio handler
   

    unsigned long currentMillis = millis();
    unsigned long batteryupadateInterval = 100000; // Intervalle de mise à jour de la batterie en millisecondes
    static unsigned long lastBatteryUpdate = 0;

        if (radioAlternate->update(packet, packetDrone)){Serial.printf("ActualRoll: %.2f, ActualPitch: %.2f, ActualYaw: %.2f\n",
        packetDrone.ActualRoll,
        packetDrone.ActualPitch,
        packetDrone.ActualYaw);

        if (currentMillis - lastBatteryUpdate >= batteryupadateInterval) {
            Serial.printf("Battery Voltage: %.2f V\n", packetDrone.batteryVoltage);
            lastBatteryUpdate = currentMillis;
        }} else {
            Serial.println("No data received from drone");
        } 
        
    
  } else {
    Serial.println("controller not connected");
  }
}