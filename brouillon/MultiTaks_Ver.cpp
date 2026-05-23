#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <RF24.h>
#include <SPI.h>
#include <atomic>

//----------------------
// Pin definitions
//----------------------
constexpr int SDA_PIN = 4;
constexpr int SCL_PIN = 5;
constexpr int LED_PIN = 2;
constexpr int RADIO_CE_PIN = 21;
constexpr int RADIO_CSN_PIN = 22;

//----------------------
// Timing
//----------------------
const TickType_t frequency_controll = pdMS_TO_TICKS(4);
const TickType_t frequency_captor = pdMS_TO_TICKS(2);
const TickType_t frequency_radio = pdMS_TO_TICKS(10);
const TickType_t frequency_failsafe = pdMS_TO_TICKS(100);

//----------------------
// Shared data structures
//----------------------
struct ControlData {
  float thr;
  float yaw;
  float pitch;
  float roll;
  bool armed;
};

struct DroneData {
  float batteryVoltage;
  float ActualPitch;
  float ActualRoll;
  float ActualYaw;
};

struct Order {
  std::atomic<float> thr{1000.0f};
  std::atomic<float> pitch{0.0f};
  std::atomic<float> roll{0.0f};
  std::atomic<float> yaw{0.0f};
  std::atomic<bool> armed{false};
};

struct Telemetry {
  std::atomic<float> batteryVoltage{0.0f};
  std::atomic<float> ActualPitch{0.0f};
  std::atomic<float> ActualRoll{0.0f};
  std::atomic<float> ActualYaw{0.0f};
};

//----------------------
// Synchronization
//----------------------
SemaphoreHandle_t xMutexRadio = NULL;
SemaphoreHandle_t xMutexControll = NULL;
SemaphoreHandle_t xMutexTele = NULL;
SemaphoreHandle_t xMutexFailsafe = NULL;

Order order;
Telemetry tele;
DroneData dronepacket;
ControlData Packet;

//----------------------
// Forward declarations
//----------------------
void Radiocore(void* pvParameter);
void Captor(void* pvParameter);
void Controll(void* pvParameter);
void failsafeAction(void* pvParameter);

//----------------------
// IMU class
//----------------------
class IMU {
  private:
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    float offsetAccelX = 0.0f;
    float offsetAccelY = 0.0f;
    float offsetAccelZ = 0.0f;
    float erreurGyroX = 0.0f;
    float erreurGyroY = 0.0f;
    float erreurGyroZ = 0.0f;
    const float GyroFC = 131.0f;
    float AccelFC = 16384.0f;
    float BETA = 0.1f;
    float angleRoll = 0.0f;
    float anglePitch = 0.0f;
    float angleYaw = 0.0f;
    unsigned long tempsPrecedent = 0;

    void lireDonneesBrutes() {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 14, true);

      accelX = (int16_t)(Wire.read() << 8 | Wire.read());
      accelY = (int16_t)(Wire.read() << 8 | Wire.read());
      accelZ = (int16_t)(Wire.read() << 8 | Wire.read());
      Wire.read(); Wire.read();
      gyroX = (int16_t)(Wire.read() << 8 | Wire.read());
      gyroY = (int16_t)(Wire.read() << 8 | Wire.read());
      gyroZ = (int16_t)(Wire.read() << 8 | Wire.read());
    }

  public:
    void calibrerIMU() {
      float sommeGx = 0.0f;
      float sommeGy = 0.0f;
      float sommeGz = 0.0f;
      float sommeAx = 0.0f;
      float sommeAy = 0.0f;
      float sommeAz = 0.0f;
      const int nbEchantillons = 500;

      for (int i = 0; i < nbEchantillons; i++) {
        lireDonneesBrutes();
        sommeGx += gyroX;
        sommeGy += gyroY;
        sommeGz += gyroZ;
        sommeAx += accelX;
        sommeAy += accelY;
        sommeAz += accelZ;
        delay(2);
      }

      erreurGyroX = sommeGx / nbEchantillons;
      erreurGyroY = sommeGy / nbEchantillons;
      erreurGyroZ = sommeGz / nbEchantillons;

      float accelX_moyen = sommeAx / nbEchantillons;
      float accelY_moyen = sommeAy / nbEchantillons;
      float accelZ_moyen = sommeAz / nbEchantillons;
      float accelMagnitude = sqrt(accelX_moyen * accelX_moyen + accelY_moyen * accelY_moyen + accelZ_moyen * accelZ_moyen);
      AccelFC = accelMagnitude / 16384.0f;

      offsetAccelX = accelX_moyen;
      offsetAccelY = accelY_moyen;
      offsetAccelZ = accelZ_moyen - AccelFC * 16384.0f;
    }

    void wire_begin(int sda, int scl) {
      Wire.begin(sda, scl);
      Wire.beginTransmission(0x68);
      Wire.write(0x6B);
      Wire.write(0x00);
      Wire.endTransmission(true);
      calibrerIMU();
      tempsPrecedent = micros();
    }

    void Beta_Modif() {
      if (Serial.available() > 0) {
        float newBeta = Serial.parseFloat();
        if (newBeta > 0.0f) {
          BETA = newBeta;
          Serial.print("Nouveau BETA : ");
          Serial.println(BETA, 4);
        }
      }
    }

    void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float DT) {
      float recipNorm;
      float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
      float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
      float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
      float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

      if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
        return;
      }

      recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
      if (recipNorm < 0.8f || recipNorm > 1.2f) {
        return;
      }

      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      float fx = 2.0f * (q1 * q3 - q0 * q2) - ax;
      float fy = 2.0f * (q0 * q1 + q2 * q3) - ay;
      float fz = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

      float s0 = -2.0f * q2 * fx + 2.0f * q1 * fy;
      float s1 =  2.0f * q3 * fx + 2.0f * q0 * fy - 4.0f * q1 * fz;
      float s2 = -2.0f * q0 * fx + 2.0f * q3 * fy - 4.0f * q2 * fz;
      float s3 =  2.0f * q1 * fx + 2.0f * q2 * fy;

      recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      qDot0 -= BETA * s0;
      qDot1 -= BETA * s1;
      qDot2 -= BETA * s2;
      qDot3 -= BETA * s3;

      q0 += qDot0 * DT;
      q1 += qDot1 * DT;
      q2 += qDot2 * DT;
      q3 += qDot3 * DT;

      recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
      q0 *= recipNorm;
      q1 *= recipNorm;
      q2 *= recipNorm;
      q3 *= recipNorm;

      angleRoll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
      anglePitch = asinf(constrain(2.0f * (q0 * q2 - q1 * q3), -1.0f, 1.0f)) * 180.0f / M_PI;
      angleYaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
    }

    float MettreAjourmesures() {
      lireDonneesBrutes();
      gyroX -= erreurGyroX;
      gyroY -= erreurGyroY;
      gyroZ -= erreurGyroZ;

      gyroX = (gyroX / GyroFC) * M_PI / 180.0f;
      gyroY = (gyroY / GyroFC) * M_PI / 180.0f;
      gyroZ = (gyroZ / GyroFC) * M_PI / 180.0f;

      accelX = accelX / AccelFC;
      accelY = accelY / AccelFC;
      accelZ = accelZ / AccelFC;

      accelX -= offsetAccelX;
      accelY -= offsetAccelY;
      accelZ -= offsetAccelZ;

      unsigned long tempsActuel = micros();
      float dt = (tempsActuel - tempsPrecedent) / 1000000.0f;
      tempsPrecedent = tempsActuel;
      dt = max(0.0001f, dt);
      madgwickUpdate(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, dt);
      return dt;
    }

    float getAngleRoll() const { return angleRoll; }
    float getAnglePitch() const { return anglePitch; }
    float getAngleYaw() const { return angleYaw; }
};

IMU monIMU;

//----------------------
// Radio link
//----------------------
class Emitor_receptor {
  private:
    RF24 radio;
    uint8_t address[6] = "00001";
    const unsigned long interval = 100;
    unsigned long dernierEnvoi = 0;

  public:
    Emitor_receptor() : radio(RADIO_CE_PIN, RADIO_CSN_PIN) {}

    void begin() {
      if (!radio.begin()) {
        Serial.println("RF24 init failed");
        while (true) {
          delay(1000);
        }
      }
      radio.openReadingPipe(1, address);
      radio.setPALevel(RF24_PA_HIGH);
      radio.startListening();
      dernierEnvoi = millis();
    }

    bool receivePacket(ControlData& packet) {
      if (radio.available()) {
        radio.read(&packet, sizeof(packet));
        return true;
      }
      return false;
    }

    bool sendPacket(const DroneData& packetDrone) {
      radio.stopListening();
      bool success = radio.write(&packetDrone, sizeof(packetDrone));
      radio.startListening();
      return success;
    }

    void sendDroneData(const DroneData& packetDrone) {
      unsigned long currentMillis = millis();
      if (currentMillis - dernierEnvoi >= interval) {
        sendPacket(packetDrone);
        dernierEnvoi = currentMillis;
      }
    }
};

Emitor_receptor radio;

//----------------------
// Motor control
//----------------------
class moteur {
  private:
    const int Moteur_pin;
    const int canal;
    int vitesseMin = 1000;
    int vitesseMax = 2000;
    bool estArmer = false;

  public:
    moteur(const int& pin, const int& channel)
      : Moteur_pin(pin), canal(channel) {}

    void start() {
      pinMode(Moteur_pin, OUTPUT);
      ledcSetup(canal, 50, 14);
      ledcAttachPin(Moteur_pin, canal);
      ledcWrite(canal, 0);
    }

    void armed() {
      estArmer = true;
    }

    void unarmed() {
      estArmer = false;
    }

    void vitCtrl(float vitesseRecue) {
      float target = constrain(vitesseRecue, vitesseMin, vitesseMax);
      if (!estArmer) {
        target = vitesseMin;
      }
      int PWMvalue = map((int)target, vitesseMin, vitesseMax, 0, 16383);
      ledcWrite(canal, PWMvalue);
    }

    void stop() {
      unarmed();
      vitCtrl(vitesseMin);
    }
};

class mixMotor {
  private:
    moteur m_ad;
    moteur m_ag;
    moteur m_dd;
    moteur m_dg;

  public:
    mixMotor() : m_ad(12, 1), m_ag(13, 2), m_dd(14, 3), m_dg(27, 4) {}

    void start() {
      m_ad.start();
      m_ag.start();
      m_dd.start();
      m_dg.start();
    }

    void stopTout() {
      m_ad.stop();
      m_ag.stop();
      m_dd.stop();
      m_dg.stop();
    }

    void arming() {
      m_ad.armed();
      m_ag.armed();
      m_dd.armed();
      m_dg.armed();
    }

    void appliquer(float thr, float p, float r, float y) {
      thr = constrain(thr, 1000.0f, 1600.0f);
      p = constrain(p, 0.0f, 100.0f);
      r = constrain(r, 0.0f, 100.0f);
      y = constrain(y, 0.0f, 100.0f);

      m_ag.vitCtrl(constrain(thr + p + r + y, 1000.0f, 2000.0f));
      m_ad.vitCtrl(constrain(thr + p - r - y, 1000.0f, 2000.0f));
      m_dg.vitCtrl(constrain(thr - p + r - y, 1000.0f, 2000.0f));
      m_dd.vitCtrl(constrain(thr - p - r + y, 1000.0f, 2000.0f));
    }
};

mixMotor monMix;

//----------------------
// PID controller
//----------------------
class PID {
  private:
    float kp, ki, kd;
    float sumErreur = 0.0f;
    float erreurPre = 0.0f;
    unsigned long lastMicros = 0;
    const float DT_MAX = 0.001f;

  public:
    PID(float p, float i, float d) : kp(p), ki(i), kd(d) {}

    float dt_init() {
      unsigned long currentMicros = micros();
      float dt = (currentMicros - lastMicros) / 1000000.0f;
      lastMicros = currentMicros;
      if (dt > DT_MAX) {
        dt = DT_MAX;
      }
      return dt;
    }

    float calcErreur(float demand, float mesure) {
      if (isnan(mesure) || isinf(mesure)) {
        return 0.0f;
      }
      float DT = dt_init();
      float erreur = demand - mesure;
      float P = erreur * kp;
      sumErreur += erreur * DT;
      sumErreur = constrain(sumErreur, -400.0f, 400.0f);
      float I = sumErreur * ki;
      float D = 0.0f;
      if (DT >= 0.0001f) {
        D = kd * (erreur - erreurPre) / DT;
      }
      erreurPre = erreur;
      return P + I + D;
    }
};

PID PIDroll(1.0f, 0.0f, 0.0f);
PID PIDpitch(1.0f, 0.0f, 0.0f);
PID PIDyaw(1.0f, 0.0f, 0.0f);

//----------------------
// Failsafe
//----------------------
class failsafe {
  private:
    unsigned long lastSignalTime;
    const unsigned long timeoutmin;
    const unsigned long timeoutmax;

  public:
    failsafe(unsigned long timeoutminValue, unsigned long timeoutmaxValue)
      : timeoutmin(timeoutminValue), timeoutmax(timeoutmaxValue), lastSignalTime(millis()) {}

    void updateSignalTime() {
      lastSignalTime = millis();
    }

    bool temporaryLoss() const {
      return (millis() - lastSignalTime) > timeoutmin;
    }

    bool criticalLoss() const {
      return (millis() - lastSignalTime) > timeoutmax;
    }

    void failsafeAction(mixMotor& mixer) {
      if (criticalLoss()) {
        mixer.stopTout();
      } else if (temporaryLoss()) {
        if (xSemaphoreTake(xMutexControll, 0) == pdTRUE) {
          order.thr = 1300.0f;
          order.pitch = 0.0f;
          order.roll = 0.0f;
          order.yaw = 0.0f;
          order.armed = false;
          xSemaphoreGive(xMutexControll);
        }
      }
    }
};

failsafe monFailsafe(1000, 10000);

//----------------------
// Data updater
//----------------------
class update_data {
  private:
    std::atomic<unsigned long> dernier_update{0};
    const unsigned long delai_update;
    std::atomic<bool> updated{false};

  public:
    update_data(unsigned long delaiupdate) : delai_update(delaiupdate) {}

    void data(std::atomic<float>& THR,
              std::atomic<float>& PITCH,
              std::atomic<float>& ROLL,
              std::atomic<float>& YAW,
              std::atomic<bool>& ARMED) {
      ControlData packet;
      if (radio.receivePacket(packet)) {
        THR = packet.thr;
        PITCH = packet.pitch;
        ROLL = packet.roll;
        YAW = packet.yaw;
        ARMED = packet.armed;

        if (xSemaphoreTake(xMutexFailsafe, 0) == pdTRUE) {
          monFailsafe.updateSignalTime();
          xSemaphoreGive(xMutexFailsafe);
        }
        dernier_update = millis();
      }
      updated = (millis() - dernier_update) <= delai_update;
    }

    bool UPDATED() const {
      return updated.load();
    }
};

update_data data(500);

//----------------------
// Setup and tasks
//----------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  monIMU.wire_begin(SDA_PIN, SCL_PIN);
  delay(5000);
  digitalWrite(LED_PIN, LOW);

  monMix.start();
  monMix.stopTout();

  xMutexRadio = xSemaphoreCreateMutex();
  xMutexControll = xSemaphoreCreateMutex();
  xMutexTele = xSemaphoreCreateMutex();
  xMutexFailsafe = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(Radiocore, "radioCORE", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(Captor, "CAPTORCORE", 10000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Controll, "CONTROLLCORE", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(failsafeAction, "FAILSAFECORE", 10000, NULL, 4, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}

void Radiocore(void* pvParameter) {
  TickType_t xLastWaketime = xTaskGetTickCount();
  radio.begin();

  for (;;) {
    if (xSemaphoreTake(xMutexRadio, 0) == pdTRUE) {
      data.data(order.thr, order.pitch, order.roll, order.yaw, order.armed);
      xSemaphoreGive(xMutexRadio);
    }

    if (xSemaphoreTake(xMutexTele, 0) == pdTRUE) {
      dronepacket.batteryVoltage = 11.1f;
      dronepacket.ActualPitch = tele.ActualPitch.load();
      dronepacket.ActualRoll = tele.ActualRoll.load();
      dronepacket.ActualYaw = tele.ActualYaw.load();
      xSemaphoreGive(xMutexTele);
    }

    radio.sendDroneData(dronepacket);
    vTaskDelayUntil(&xLastWaketime, frequency_radio);
  }
}

void Captor(void* pvParameter) {
  TickType_t xLastTimeWake = xTaskGetTickCount();

  for (;;) {
    monIMU.MettreAjourmesures();
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

void Controll(void* pvParameter) {
  bool lastArmed = false;
  bool armed = false;
  float thr = 1000.0f;
  TickType_t xLastWaketime = xTaskGetTickCount();

  for (;;) {
    if (data.UPDATED()) {
      float correctionRoll = PIDroll.calcErreur(order.roll, tele.ActualRoll.load());
      float correctionPitch = PIDpitch.calcErreur(order.pitch, tele.ActualPitch.load());
      float correctionYaw = PIDyaw.calcErreur(order.yaw, tele.ActualYaw.load());

      if (xSemaphoreTake(xMutexControll, 1) == pdTRUE) {
        armed = order.armed.load();
        thr = order.thr.load();
        xSemaphoreGive(xMutexControll);
      }

      if (armed && !lastArmed) {
        monMix.arming();
        lastArmed = true;
      } else if (!armed && lastArmed) {
        monMix.stopTout();
        lastArmed = false;
      }

      if (armed && lastArmed) {
        monMix.appliquer(thr, correctionPitch, correctionRoll, correctionYaw);
      } else {
        monMix.stopTout();
      }
    }

    vTaskDelayUntil(&xLastWaketime, frequency_controll);
  }
}

void failsafeAction(void* pvParameter) {
  TickType_t xLastWaketime = xTaskGetTickCount();

  for (;;) {
    if (xSemaphoreTake(xMutexFailsafe, 1) == pdTRUE) {
      monFailsafe.failsafeAction(monMix);
      xSemaphoreGive(xMutexFailsafe);
    }
    vTaskDelayUntil(&xLastWaketime, frequency_failsafe);
  }
}
