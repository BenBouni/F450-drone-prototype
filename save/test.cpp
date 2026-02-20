#include <Arduino.h>
#include <BLEGamepadClient.h>
#include <RF24.h>
#include <SPI.h>
#include <atomic>

SemaphoreHandle_t xMutex;
void Radiocore(void * Pvparameter);

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
};

struct telemerie {
    std::atomic <float> batteryVoltage;
    std::atomic <float> ActualPitch;
    std::atomic <float> ActualRoll;
    std::atomic <float> ActualYaw;

};

struct Order {
  std::atomic <float> thr;
  std::atomic <float> pitch;
  std::atomic <float> roll;
  std::atomic <float> yaw;
  std::atomic <bool> armed;
};

  dataPacket Packet; // receive from GS
  dataPacketDrone packetDrone; // send to GS
  Order order; // from gs shared data
  telemerie tele; // from drone shared data

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
    bool sendPacket(const dataPacket& Packet) {
        radio.openWritingPipe(PipelineAddressTX);
        radio.stopListening();
        return radio.write(&Packet, sizeof(Packet));
    }
    bool radioWrite(const dataPacket& Packet) {
        return radio.write(&Packet, sizeof(Packet));
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
    bool update( dataPacket& Packet, dataPacketDrone& packetDrone) {
        unsigned long currentTime = millis();

        if (currentTime - lastSendTime >= sendInterval) {
            radio.sendPacket(Packet);
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

class update_data {
  private :
  std::atomic <unsigned long> dernier_update {0};
  const unsigned long delai_update;
  std::atomic <bool> updated{false};
  public :
  update_data(unsigned long delaiupdate) : delai_update(delaiupdate) {}

  void data(std::atomic <float> &THR,  std::atomic <float> &PITCH,  std::atomic <float> &ROLL,  std::atomic <float> &YAW,  std::atomic <float> &BATTERY) {
   if (RADIO->receivePacket(packetDrone)) {
     PITCH = packetDrone.ActualPitch;
     ROLL = packetDrone.ActualRoll;
     YAW = packetDrone.ActualYaw;
     BATTERY = packetDrone.batteryVoltage;
    dernier_update = millis();
   } 
   if (millis()-dernier_update > delai_update) {
    updated = false;
   } else updated = true; 
  }
  bool UPDATED() const { return updated.load(); }
};

XboxController controller;

RadioAlternateEmitor_Receptor* radioAlternate;


void setup(void) {
  Serial.begin(115200);
  controller.begin();
  RADIO = new RadioEmitor_Receptor(100, 0xE8E8F0F0E1LL, 0xE8E8F0F0E2LL);
  RADIO->begin();
  radioAlternate = new RadioAlternateEmitor_Receptor(*RADIO);
    xMutex = xSemaphoreCreateMutex();
      xTaskCreatePinnedToCore(
        Radiocore,
        "radiocore",
        10000,
        NULL,
        1,
        NULL,
        0
    );
}

     // Intervalle de mise à jour de la batterie en millisecondes
void loop() {
    static unsigned long lastBatteryUpdate = 0;
    unsigned long currentMillis = millis();
    const unsigned long batteryupadateInterval = 100000;
    static bool last_armed_state = false;

  if (controller.isConnected()) {
    XboxControlsState s;
    controller.read(&s);
    Serial.println("controller connected");

// Map controller inputs to drone commands + prepare the packet to send 
  if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
    // stocking orders in shared variable
    order.thr = constrain((-0.35f * s.leftTrigger + 0.65f * s.rightTrigger) / (1.023f ) + 1300.0f, 1000.0f, 2000.0f);
    order.yaw = constrain((s.rightStickX / 32768.0f) * 30, -30.0f, 30.0f);
    order.pitch = constrain((s.rightStickY / 32768.0f) * 30, -30.0f, 30.0f);
    order.roll = constrain((s.leftStickX / 32768.0f) * 30, -30.0f, 30.0f);
    if (s.xboxButton && !last_armed_state) { //arming on xbox button press
      order.armed = true;
    } else if (!s.xboxButton && last_armed_state) {
      order.armed = false;
    }
    last_armed_state = s.xboxButton;
   
        if (currentMillis - lastBatteryUpdate >= batteryupadateInterval) {
            Serial.printf("GS roll: %.2f, pitch: %.2f, yaw: %.2f", tele.ActualRoll, tele.ActualPitch, tele.ActualYaw);
            lastBatteryUpdate = currentMillis;
        } else {
            Serial.println("No data received from drone");
        } 
      } else {
    Serial.println("controller not connected");
  }
}
}

void Radiocore(void * Pvparameter) {
  for(;;) {
        if (xSemaphoreTake(xMutex, 0)== pdTRUE) {
         // save received data in shared variable   
        tele.ActualRoll= packetDrone.ActualRoll;
        tele.ActualPitch= packetDrone.ActualPitch;
        tele.ActualYaw= packetDrone.ActualYaw;
        tele.batteryVoltage= packetDrone.batteryVoltage;
        xSemaphoreGive(xMutex);
    }
    radioAlternate->update(Packet, packetDrone);
        //  packet to send
    if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
        Packet.thr = order.thr;
        Packet.yaw = order.yaw;
        Packet.pitch = order.pitch;
        Packet.roll = order.roll;
        Packet.armed = order.armed;
        xSemaphoreGive(xMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // little delay to avoid overwhelming the CPU
  
}
}

   
