
#include "config.h"
#include "IMU.h"

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
  monMix.stopTout();
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
    if (xSemaphoreTake(xMutexRadio, 1) == pdTRUE) {            
       data.data(order.thr, order.pitch, order.roll, order.yaw, order.armed);
       if (xSemaphoreTake(xMutexFailsafe, (TickType_t)0) == pdTRUE) {
       monFailsafe.updateSignalTime();
       xSemaphoreGive(xMutexFailsafe);
}
      xSemaphoreGive(xMutexRadio);
      }
    if (xSemaphoreTake(xMutexTele, 1) == pdTRUE) {
        dronepacket.batteryVoltage = 11.1; // to be replaced by actual battery voltage measurement
        dronepacket.ActualPitch = tele.ActualPitch.load();
        dronepacket.ActualRoll = tele.ActualRoll.load();
        dronepacket.ActualYaw = tele.ActualYaw.load();
        xSemaphoreGive(xMutexTele);
    }
    // send telemetry (dronepacket) and receive incoming control packet (Packet)
    radio.alternateSend(dronepacket, Packet);
    vTaskDelayUntil(&xLastWaketime, frequency_radio);  
   }
}
 
void Captor(void * Pvparameter) {
  TickType_t xLastTimeWake = xTaskGetTickCount(); 
  for(;;){
   float dt = monIMU.MettreAjourmesures();
   if (dt < 0.0f || monIMU.IMU_flag) {
     monMix.stopTout();
     if (xSemaphoreTake(xMutexControll, pdMS_TO_TICKS(10)) == pdTRUE) {
       order.armed.store(false);
       xSemaphoreGive(xMutexControll);
     }
     vTaskDelayUntil(&xLastTimeWake, frequency_captor);
     continue;
   }

   float mesureRoll = monIMU.getAngleRoll();
   float mesurePitch = monIMU.getAnglePitch();
   float mesureYaw = monIMU.getAngleYaw();
   if (!isfinite(mesureRoll) || !isfinite(mesurePitch) || !isfinite(mesureYaw)) {
     monMix.stopTout();
     if (xSemaphoreTake(xMutexControll, pdMS_TO_TICKS(10)) == pdTRUE) {
       order.armed.store(false);
       xSemaphoreGive(xMutexControll);
     }
     vTaskDelayUntil(&xLastTimeWake, frequency_captor);
     continue;
   }

    if (xSemaphoreTake(xMutexTele, pdMS_TO_TICKS(10)) == pdTRUE) {
     tele.ActualPitch.store(mesurePitch);
     tele.ActualRoll.store(mesureRoll);
     tele.ActualYaw.store(mesureYaw);
     xSemaphoreGive(xMutexTele);
   }
   
   vTaskDelayUntil(&xLastTimeWake, frequency_captor);
 }
}

void Controll(void * Pvparameter) {
 bool lastArmed = false;
 bool armed = false;
 float thr = 1000.0f; // secure throttle value in case of arming failure, to avoid accidental takeoff in case of arming failure
 const TickType_t frequency = frequency_controll; // 4 ms for the control loop, adjusted via pdMS_TO_TICKS in config.cpp
 TickType_t xLastWaketime = xTaskGetTickCount();
  
 for(;;){
   if (monIMU.IMU_flag) {
     monMix.stopTout();
     if (xSemaphoreTake(xMutexControll, pdMS_TO_TICKS(10)) == pdTRUE) {
       order.armed.store(false);
       xSemaphoreGive(xMutexControll);
     }
     vTaskDelayUntil(&xLastWaketime, frequency);
     continue;
   }

   if (!data.UPDATED()) {
     monMix.stopTout();
     if (xSemaphoreTake(xMutexControll, pdMS_TO_TICKS(10)) == pdTRUE) {
       order.armed.store(false);
       xSemaphoreGive(xMutexControll);
     }
     vTaskDelayUntil(&xLastWaketime, frequency);
     continue;
   }

   if (data.UPDATED()) {
     // load atomics into temporaries to pass plain floats to PID
     float order_roll = order.roll.load();
     float order_pitch = order.pitch.load();
     float order_yaw = order.yaw.load();
     float tele_roll = tele.ActualRoll.load();
     float tele_pitch = tele.ActualPitch.load();
     float tele_yaw = tele.ActualYaw.load();

     float erreurRoll = PIDroll.calcErreur(order_roll, tele_roll);
     float erreurPitch = PIDpitch.calcErreur(order_pitch, tele_pitch);
     float erreurYaw = PIDyaw.calcErreur(order_yaw, tele_yaw);
     if (xSemaphoreTake(xMutexControll, pdMS_TO_TICKS(10)) == pdTRUE) {
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
    if (xSemaphoreTake(xMutexFailsafe, 0) == pdTRUE) {
      monFailsafe.failsafeAction(monMix);
      xSemaphoreGive(xMutexFailsafe);
    }
    vTaskDelayUntil(&xLastWaketime, frequency_failsafe);
  }
}