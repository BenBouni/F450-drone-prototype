#include "Pins.h"
#include "config.h"
#include "motorLOGIC.h"
#include "Radio.h"
#include "IMU.h"
#include "data.h"

void Captor(void * pvParameters);
void Controll(void * pvParameters);
void Radiocore(void * pvParameters);
void failsafeAction(void * pvParameters);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  monIMU.wire_begin(SDA_PIN, SCL_PIN);
  digitalWrite(LED_PIN, HIGH); // calibration en cours
  delay(5000);  
  digitalWrite(LED_PIN, LOW); // pret pour le vol
  monMix.start();
  monMix.arming();
  //mutex gestion :
  xMutexRadio = xSemaphoreCreateMutex();
  xMutexControll = xSemaphoreCreateMutex();
  xMutexTele = xSemaphoreCreateMutex();
  xMutexFailsafe = xSemaphoreCreateMutex();
  // tasks creation :
      xTaskCreatePinnedToCore(
        Radiocore,
        "radioCORE",
        10000,
        NULL,
        1,
        NULL,
        0
    );
      xTaskCreatePinnedToCore(
        Captor,
        "CAPTORCORE",
        10000,
        NULL,
        2,
        NULL,
        1
    );
      xTaskCreatePinnedToCore(
        Controll,
        "CONTROLLCORE",
        10000,
        NULL,
        3,
        NULL,
        1
    );
        xTaskCreatePinnedToCore(
          failsafeAction,
          "FAILSAFECORE",
          10000,
          NULL,
          4,
          NULL,
          1
      );
}

void loop() { vTaskDelete(NULL); } // the loop is not used because all the code is executed in seperate tasks on the two cores of the ESP32, to optimize the performance and avoid blocking the code with long operations such as radio communication or PID calculations, and to allow for better real-time performance of the control loop and the failsafe.

void Radiocore(void * Pvparameter) {
  TickType_t xLastWaketime = xTaskGetTickCount(); // variable to store the last wake time for vTaskDelayUntil
   radio.begin();
   for(;;){ 
    if (xSemaphoreTake(xMutexRadio, 0) == pdTRUE) {            
       data.data(order.thr, order.pitch, order.roll, order.yaw, order.armed);
      xSemaphoreGive(xMutexRadio);
      }
    if (xSemaphoreTake(xMutexTele, 0) == pdTRUE) {
        dronepacket.batteryVoltage = 11.1; // to be replaced by actual battery voltage measurement
        dronepacket.ActualPitch = tele.ActualPitch;
        dronepacket.ActualRoll = tele.ActualRoll;
        dronepacket.ActualYaw = tele.ActualYaw;
        xSemaphoreGive(xMutexTele);
    }
      radio.sendDroneData(dronepacket);
    vTaskDelayUntil(&xLastWaketime, frequency_radio);
   }
}
 
void Captor(void * Pvparameter) {
  TickType_t xLastTimeWake = xTaskGetTickCount(); 
  for(;;){
    float dt = monIMU.MettreAjourmesures();
    float mesureRoll = monIMU.getAngleRoll();
    float mesurePitch = monIMU.getAnglePitch();
    float mesureYaw = monIMU.getAngleYaw();

     if (xSemaphoreTake(xMutexTele, 1) == pdTRUE) {
      tele.ActualPitch = mesurePitch;
      tele.ActualRoll = mesureRoll;
      tele.ActualYaw = mesureYaw;
      xSemaphoreGive(xMutexTele);
    }
   
    vTaskDelayUntil(&xLastTimeWake, frequency_captor);
  }
}

void Controll(void * Pvparameter) {
  bool lastArmed = false;
  bool armed = false;
  unsigned long lastarm = 0;
  float thr = 1000.0f; // secure throttle value in case of arming failure, to avoid accidental takeoff in case of arming failure
  // use the shared constant defined in config.cpp to guarantee a non‑zero tick count
  const TickType_t frequency = frequency_controll; // 4 ms for the control loop, adjusted via pdMS_TO_TICKS in config.cpp
  TickType_t xLastWaketime = xTaskGetTickCount(); // variable to store the last wake time for vTaskDelayUntil
  for(;;){
    if (data.UPDATED()) {
      float erreurRoll = PIDroll.calcErreur(order.roll, tele.ActualRoll);
      float erreurPitch = PIDpitch.calcErreur(order.pitch, tele.ActualPitch);
      float erreurYaw = PIDyaw.calcErreur(order.yaw, tele.ActualYaw );
      if (xSemaphoreTake(xMutexControll, 1) == pdTRUE) {
        armed = order.armed.load();
        thr = order.thr.load();
        xSemaphoreGive(xMutexControll); 
    }
        if (armed == true && lastArmed == false) {
          monMix.arming();
          
          lastArmed = true;
        } else if (armed == false && lastArmed == true) {
          monMix.stopTout();
          lastArmed = false;
        }
            if (armed == true && lastArmed == true) {
            monMix.appliquer(thr, erreurPitch, erreurRoll, erreurYaw);
            } else {
            monMix.stopTout();
            }
      }
  vTaskDelayUntil(&xLastWaketime, frequency);
}
}
 
void failsafeAction(void * Pvparameter) {
  // 4 ms for the failsafe loop, to be adjusted according to the needs of the failsafe and the performance of the ESP32, and to avoid overloading the CPU with too many tasks running at high frequency
  TickType_t xLastWaketime = xTaskGetTickCount(); // variable to store the last wake time for vTaskDelayUntil
  for(;;){
    if (xSemaphoreTake(xMutexFailsafe, 1) == pdTRUE) {
      monFailsafe.failsafeAction(monMix);
      xSemaphoreGive(xMutexFailsafe);
    }
    vTaskDelayUntil(&xLastWaketime, frequency_failsafe);
  }
}