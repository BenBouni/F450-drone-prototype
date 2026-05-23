// this file is a rework of the gcs part (file :"test.ccp/brouillon" was the 1st attempt with bluetooth), it is the intermidiate between the screen of telemtry/controller and the drone, 
// all communication
// is handedled in this file , the control part is getting reworked to switch to serial communication instead of bluetooth for the controller, 
// the radio communication with the drone is handled in the radio task and the data received from the drone is printed on the serial monitor 
// in the printer task, the control task is responsible for parsing the data received from the controller and sending it to the radio task to be transmitted to the 
// semaphores are used to protect shared resources such as the data received from the controller and the data received from the drone to avoid race conditions

#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
 // mettre a jour les pins CE et CSN selon votre configuration matérielle

#define CE 9
#define CSN 10

#define a 30.0f // processing factor for the controller inputs , can be tuned later to adjust the sensitivity of the controls

void RadioTask(void *pvParameters);
void ControlTask(void *pvParameters);
void PrinterTask(void *pvParameters);

SemaphoreHandle_t xMutexRadio; // handle received data
SemaphoreHandle_t xMutexPrinter; // handle serial printing
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
RxData packetReceived; // global variable to store the data received from the drone to be printed on the serial monitor

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
        char buffer[100];
        int i = 0;
    public:
        void praserData(){
            if (Serial.available()) {
                buffer[0] = '\0'; // Clear the buffer
                int len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1); // Read until newline
                buffer[len] = '\0'; // Ensure null end
            
                char* token = strtok(buffer, ",");
                if (token != NULL && strcmp(token, "GS") == 0) {
                    i = 0; // Reset i after processing 
                    token = strtok(NULL, ","); // Get the next token
                    while (token != NULL) {
                      float value = atof(token); // Convert the token to a float 
                        
                        if (i == 0) controlData.leftStickX = constrain((value / 32768.0f) * a, -a, a); // roll
                        else if (i == 1) controlData.leftStickY = constrain((value / 32768.0f) * a, -a, a); // pitch
                        else if (i == 2) controlData.rightStickX = constrain((value / 32768.0f) * a, -a, a); // yaw
                        else if (i == 3) controlData.rightStickY = constrain((value / 32768.0f) * a, -a, a); // throttle
                        else if (i == 4) controlData.xboxButton = value; // arming button
                        token = strtok(NULL, ",");  // not the best logic but it works for now , the controller is sending the data in the format "GS,roll,pitch,yaw,throttle" so we can just use i to determine which value we are parsing and assign it to the correct variable in the controlData struct
                        i = (i + 1) % 5; // Increment i 
                    }           
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
    
    public:
        Emitor_receptor(const unsigned long interval, const int CE_PIN, const int CSN_PIN, const uint8_t tx[6], const uint8_t rx[6]) : 
                interval(interval), CE_PIN(CE_PIN), CSN_PIN(CSN_PIN), radio(CE_PIN, CSN_PIN) {
          memcpy(txAddress, tx, 6);
          memcpy(rxAddress, rx, 6);
      }

        void begin() {
            if (!radio.begin()) {
                Serial.println("Radio hardware not responding!");
                while (1) {} // halt if radio is not working
            }
            radio.openReadingPipe(1, rxAddress);
            radio.openWritingPipe(txAddress);
            radio.setPALevel(RF24_PA_LOW);
            radio.startListening();
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
            radio.startWrite(&packetSent, sizeof(packetSent));
        }
        
        void alternateSend(const sent& packetSent, received& packetReceived) {
            unsigned long currentMillis = millis();
            if (currentMillis - dernierEnvoi >= interval) {
                sendPacket(packetSent);
                dernierEnvoi = currentMillis;
            } 
            if (!radio.isListening()) {
                radio.TXStandBy();
                radio.startListening();
            }
                receivePacket(packetReceived);
            
        }
        };
Emitor_receptor<TxData, RxData> radio(10, CE, CSN, (uint8_t*)"00001", (uint8_t*)"00002");

void setup() {
    Serial.begin(9600);
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
        2,
        NULL,
        1 // run the printer task on core 1
    );
}


void loop() {
    vTaskDelete(NULL); // the loop is not used because all the code is executed in seperate tasks on the two cores of the ESP32, to optimize the performance and avoid blocking the code with long operations such as radio communication or PID calculations, and to allow for better real-time performance of the control loop and the failsafe.
}

void RadioTask(void *pvParameters) {
    RxData tempReceived; // temporary variable to store the received data before copying it to the global variable packetReceived, to avoid

    for(;;) {
        
        if (xSemaphoreTake(xMutexRadio, 1) == pdTRUE) {
            packetSent.roll = r.leftStickX;
            packetSent.pitch = r.leftStickY;
            packetSent.yaw = r.rightStickX;
            packetSent.throttle = r.rightStickY;
            packetSent.armed = r.xboxButton;
            xSemaphoreGive(xMutexRadio);
        }
        radio.alternateSend(packetSent, tempReceived);
        if (xSemaphoreTake(xMutexPrinter, 1) == pdTRUE) {
            print.ActualRoll = tempReceived.roll;
            print.ActualPitch = tempReceived.pitch;
            print.ActualYaw = tempReceived.yaw;
            print.batteryVoltage = tempReceived.batteryVoltage;
            xSemaphoreGive(xMutexPrinter);
        }
        vTaskDelay(frequency_radio); // send every 10 ms
    }
}

void ControlTask(void *pvParameters) {



    for(;;) {
        if (xSemaphoreTake(xMutexserial, 1) == pdTRUE) {
            
            praser.praserData();
            xSemaphoreGive(xMutexserial);
        }
        if (xSemaphoreTake(xMutexRadio, 1) == pdTRUE ) {
            r = praser.getData();
            xSemaphoreGive(xMutexRadio);
        }
        
        vTaskDelay(frequency_radio); // every 10 ms
    }
    
}

void PrinterTask(void *pvParameters) {

    for(;;) {
        if (xSemaphoreTake(xMutexPrinter, 1) == pdTRUE && xSemaphoreTake(xMutexserial, 1) == pdTRUE) {
            Serial.print("Roll: ");
            Serial.print(print.ActualRoll);
            Serial.print(" | Pitch: ");
            Serial.print(print.ActualPitch);
            Serial.print(" | Yaw: ");
            Serial.print(print.ActualYaw);
            Serial.print(" | Battery Voltage: ");
            Serial.println(print.batteryVoltage);
            xSemaphoreGive(xMutexPrinter);
            xSemaphoreGive(xMutexserial);
        }
        vTaskDelay(frequency_printer); // print every second
    }
}

 
// there might be an issue in the printer task 