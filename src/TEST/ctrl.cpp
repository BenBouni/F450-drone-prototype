// this file is a rework of the gcs part (file :"test.ccp/brouillon" was the 1st attempt with bluetooth), it is the intermidiate between the screen of telemtry/controller and the drone, 
// all communication is handedled in this file , the control part is getting reworked to switch to serial communication instead of bluetooth for the controller, 
// the radio communication with the drone is handled in the radio task and the data received from the drone is printed on the serial monitor 
// in the printer task, the control task is responsible for parsing the data received from the controller and sending it to the radio task to be transmitted to the 
// semaphores are used to protect shared resources such as the data received from the controller and the data received from the drone to avoid race conditions

#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
 // mettre a jour les pins CE et CSN selon votre configuration matérielle

#define CE 9
#define CSN 10

#define a 30.0f // processing factor for the controller inputs , can be tuned later to adjust the sensitivity of the controls

void RadioTask(void *pvParameters);
void ControlTask(void *pvParameters);
void PrinterTask(void *pvParameters);

SemaphoreHandle_t xMutexRadio; // handle received data
SemaphoreHandle_t xMutexPrinter; // handle telemetry snapshot
SemaphoreHandle_t xMutexserial; // handle serial communication with the controller

TickType_t frequency_radio = pdMS_TO_TICKS(10);
TickType_t frequency_printer = pdMS_TO_TICKS(1000);




struct TxData { // Tx stands for transmit
   float roll;
   float pitch;
   float yaw;
   float throttle;
   float armed;
};
TxData packetSent; // global variable to store the data to be sent to the drone

struct rawData { // directly comes from the controller without any processing
   float leftStickX; // roll 
   float leftStickY; // pitch
   float rightStickX;  // yaw
   float rightStickY; // throttle
   float xboxButton; // arming button (1 or 0)
}; 
rawData r; // global variable to store the processed data from the controller to be sent to the drone

struct RxData { // Rx stands for receive
   float roll;
   float pitch;
   float yaw;
   float throttle;
   float batteryVoltage;
};
RxData packetReceived; // global variable to store the data received from the drone and temporarely stocked

struct printData { // data received from the drone to be printed on the serial monitor
   float ActualRoll;
   float ActualPitch;
   float ActualYaw;
   float batteryVoltage;
};
printData print; // global variable to store the data received from the drone to be printed on the serial monitor

class Praser {
   private:
       rawData controlData;
       char buffer[128];
       size_t index = 0;

       static bool parseFloatToken(const char* token, float& outValue) {
           if (token == nullptr) {
               return false;
           }
           char* endPtr = nullptr;
           const float value = strtof(token, &endPtr);
           if (endPtr == token || !isfinite(value)) {
               return false;
           }
           outValue = value;
           return true;
       }

       void parseLine(char* line) {
           char* savePtr = nullptr;
           char* token = strtok_r(line, ",", &savePtr);
           if (token == nullptr || strcmp(token, "GS") != 0) {
               return;
           }

           rawData temp = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
           int fieldIndex = 0;
           while ((token = strtok_r(nullptr, ",", &savePtr)) != nullptr && fieldIndex < 5) {
               float value = 0.0f;
               if (!parseFloatToken(token, value)) {
                   return;
               }

               switch (fieldIndex) {
                   case 0: temp.leftStickX = constrain((value / 32768.0f) * a, -a, a); break;
                   case 1: temp.leftStickY = constrain((value / 32768.0f) * a, -a, a); break;
                   case 2: temp.rightStickX = constrain((value / 32768.0f) * a, -a, a); break;
                   case 3: temp.rightStickY = constrain((value / 32768.0f) * a, -a, a); break;
                   case 4: temp.xboxButton = constrain(value, 0.0f, 1.0f); break;
                   default: break;
               }
               ++fieldIndex;
           }

           if (fieldIndex != 5) {
               return;
           }
           controlData = temp;
       }

   public:
       void praserData() {
           while (Serial.available() > 0) {
               const char c = static_cast<char>(Serial.read());
               if (c == '\n') {
                   buffer[index] = '\0';
                   parseLine(buffer);
                   index = 0;
                   continue;
               }

               if (index < (sizeof(buffer) - 1)) {
                   buffer[index++] = c;
               } else {
                   index = 0;
               }
           }
       }

       const rawData& getData() const { return controlData; }
};
Praser praser; // create an instance of the praser class to be used in the control task

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
    bool isListeningState;
    
    public:
        Emitor_receptor(const unsigned long interval, const int CE_PIN, const int CSN_PIN, const uint8_t tx[6], const uint8_t rx[6]) : 
                interval(interval), CE_PIN(CE_PIN), CSN_PIN(CSN_PIN), radio(CE_PIN, CSN_PIN) {
          memcpy(txAddress, tx, 6);
          memcpy(rxAddress, rx, 6);
      }

        void begin() {
            if (!radio.begin()) {
                Serial.println("Radio hardware not responding!");
                isListeningState = false;
                return;
            }
            radio.openReadingPipe(1, rxAddress);
            radio.openWritingPipe(txAddress);
            radio.setPALevel(RF24_PA_LOW);
            radio.startListening();
            isListeningState = true;
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
            isListeningState = false;
            radio.write(&packetSent, sizeof(packetSent));
        }
        
        void alternateSend(const sent& packetSent, received& packetReceived) {
            unsigned long currentMillis = millis();
            if (currentMillis - dernierEnvoi >= interval) {
                sendPacket(packetSent);
                dernierEnvoi = currentMillis;
            } 
            if (isListeningState != true) {
                radio.txStandBy();
                radio.startListening();
                isListeningState = true;
            }
                receivePacket(packetReceived);  
        }
        };
Emitor_receptor<TxData, RxData> radio(10, CE, CSN, (uint8_t*)"00001", (uint8_t*)"00002");

void setup() {
    Serial.begin(115200);
    radio.begin();
    xMutexRadio = xSemaphoreCreateMutex();
    xMutexPrinter = xSemaphoreCreateMutex();
    xMutexserial = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(
        RadioTask,
        "RadioTask",
        10000,
        NULL,
        1,
        NULL,
        0 // run the radio task on core 0
    );
    xTaskCreatePinnedToCore(
        ControlTask,
        "ControlTask",
        10000,
        NULL,
        1,
        NULL,
        1 // run the control task on core 1
    );
    xTaskCreatePinnedToCore(
        PrinterTask,
        "PrinterTask",
        10000,
        NULL,
        0,
        NULL,
        1 // run the printer task on core 1
    );
}


void loop() {
    vTaskDelete(NULL); // the loop is not used because all the code is executed in seperate tasks on the two cores of the ESP32, to optimize the performance and avoid blocking the code with long operations such as radio communication or PID calculations, and to allow for better real-time performance of the control loop and the failsafe.
}

void RadioTask(void *pvParameters) {
    RxData tempReceived = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    for (;;) {
        if (xSemaphoreTake(xMutexRadio, pdMS_TO_TICKS(10)) == pdTRUE) {
            packetSent.roll = r.leftStickX;
            packetSent.pitch = r.leftStickY;
            packetSent.yaw = r.rightStickX;
            packetSent.throttle = r.rightStickY;
            packetSent.armed = r.xboxButton;
            xSemaphoreGive(xMutexRadio);
        }

        radio.alternateSend(packetSent, tempReceived);

        if (xSemaphoreTake(xMutexPrinter, pdMS_TO_TICKS(10)) == pdTRUE) {
            print.ActualRoll = tempReceived.roll;
            print.ActualPitch = tempReceived.pitch;
            print.ActualYaw = tempReceived.yaw;
            print.batteryVoltage = tempReceived.batteryVoltage;
            xSemaphoreGive(xMutexPrinter);
        }

        vTaskDelay(frequency_radio);
    }
}

void ControlTask(void *pvParameters) {
    for (;;) {
        if (xSemaphoreTake(xMutexserial, pdMS_TO_TICKS(10)) == pdTRUE) {
            praser.praserData();
            xSemaphoreGive(xMutexserial);
        }

        if (xSemaphoreTake(xMutexRadio, pdMS_TO_TICKS(10)) == pdTRUE) {
            r = praser.getData();
            xSemaphoreGive(xMutexRadio);
        }

        vTaskDelay(frequency_radio);
    }
}

void PrinterTask(void *pvParameters) {
    for (;;) {
        printData snapshot = {0.0f, 0.0f, 0.0f, 0.0f};
        if (xSemaphoreTake(xMutexPrinter, pdMS_TO_TICKS(10)) == pdTRUE) {
            snapshot = print;
            xSemaphoreGive(xMutexPrinter);
        }

        if (xSemaphoreTake(xMutexserial, pdMS_TO_TICKS(10)) == pdTRUE) {
            Serial.print("GS,");
            Serial.print(snapshot.ActualRoll);
            Serial.print(",");
            Serial.print(snapshot.ActualPitch);
            Serial.print(",");
            Serial.print(snapshot.ActualYaw);
            Serial.print(",");
            Serial.println(snapshot.batteryVoltage);
            xSemaphoreGive(xMutexserial);
        }

        vTaskDelay(frequency_printer);
    }
}
