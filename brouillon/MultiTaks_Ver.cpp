#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <RF24.h>
#include <SPI.h>
#include <atomic>
#include <climits>
#include <Servo.h>

SemaphoreHandle_t xMutexRadio; // handle received data
SemaphoreHandle_t xMutexControll; // handell data in the PID loop
SemaphoreHandle_t xMutexTele; // handle telemetry data to send to GS
SemaphoreHandle_t xMutexFailsafe; // handle failsafe conditions

void Radiocore(void * Pvparameter); // task for receiving and sending data by radio
void Captor(void * Pvparameter); // task for reading the IMU and calculating the angles with the madgwick filter
void Controll(void * Pvparameter); // task for calculating the PID corrections and applying the motor mixing
void failsafeAction(void * Pvparameter); // task for checking the failsafe conditions and applying the failsafe actions

// data structures for to manage proprely control and telemetry  

struct ControlData {
  float thr; //1000 à 1600
  float yaw; // 0 à 100
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
  std::atomic <float> thr;
  std::atomic <float> pitch;
  std::atomic <float> roll;
  std::atomic <float> yaw;
  std::atomic <bool> armed;
};

struct telemerie {
    std::atomic <float> batteryVoltage;
    std::atomic <float> ActualPitch;
    std::atomic <float> ActualRoll;
    std::atomic <float> ActualYaw;
    };

  ControlData Packet; // receive from GS
  DroneData dronepacket; // send to GS
  Order order; // global order object to be shared between the radio task and the PID/controll task, to store the latest received control data from the GS
  telemerie tele; // global telemetry object to be shared between the PID/controll task and the radio task, to store the latest calculated telemetry data to send to the GS

// class creations :
// the classes are defined in the same file for simplicity, but they will be moved to separate files for better organization and readability

class IMU {
  private:
    // MPU-6050 
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ; 
    // coordonates quaternions for the madgwick filter
    float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
    //for calibration :
    float offsetAccelX = 0, offsetAccelY = 0, offsetAccelZ = 0; 
    float erreurGyroX = 0, erreurGyroY = 0, erreurGyroZ = 0;
    // conversion factors :
    const float GyroFC = 131.0;
    float AccelFC = 16384.0;
    const float AccMagFC = 600.0; // will implement the compass later, this is the conversion factor for the magnetometer, to adjust according to the measurements
     float BETA = 0.1; // beta parameter for the madgwick filter, to be tuned for better performance
    // magnetometer : (will implement it later, for now we will only use the accelerometer and the gyroscope for the attitude estimation and the yaw angle will be calculated without the magnetometer, which is not optimal but will work for a first version of the system)
    
    // filtered angles (what the PID will use):
    float angleRoll = 0.0;
    float anglePitch = 0.0;
    float angleYaw = 0.0;
    unsigned long tempsPrecedent;

    void lireDonneesBrutes() {
        Wire.beginTransmission(0x68);
        Wire.write(0x3B); // Adresse du registre de début des données
        Wire.endTransmission(false);
        Wire.requestFrom(0x68, 14, true); // Demander 14 octets

        accelX = (int16_t)(Wire.read() << 8 | Wire.read());
        accelY = (int16_t)(Wire.read() << 8 | Wire.read());
        accelZ = (int16_t)(Wire.read() << 8 | Wire.read());
        Wire.read(); Wire.read(); // Ignorer Temp
        gyroX = (int16_t)(Wire.read() << 8 | Wire.read()); 
        gyroY = (int16_t)(Wire.read() << 8 | Wire.read());
        gyroZ = (int16_t)(Wire.read() << 8 | Wire.read());

       
      
    }
  public :
  
  //imu calibration :
  void calibrerIMU() {
    float sommeGx=0, sommeGy=0, sommeGz=0;
    float sommeAx=0, sommeAy=0, sommeAz=0;
    const int nbEchantillons = 500;

    for(int i = 0; i < nbEchantillons; i++) {
        lireDonneesBrutes();
        sommeGx += gyroX;
        sommeGy += gyroY;
        sommeGz += gyroZ;
        sommeAx += accelX;
        sommeAy += accelY;
        sommeAz += accelZ;
        delay(2);  // little delay to space the measurements, to avoid any issues with the I2C bus or the sensor itself, and to get a more accurate average of the measurements for the calibration
    }

    // Biais gyro
    erreurGyroX = sommeGx / nbEchantillons;
    erreurGyroY = sommeGy / nbEchantillons;
    erreurGyroZ = sommeGz / nbEchantillons;

    // Biais accelero
    float accelX_moyen = sommeAx / nbEchantillons;
    float accelY_moyen = sommeAy / nbEchantillons;
    float accelZ_moyen = sommeAz / nbEchantillons;

    // accelero conversion factor (we calculate it based on the magnitude of the acceleration vector, which should be equal to 1g when the sensor is at rest, to get a more accurate conversion factor that takes into account the actual sensitivity of the sensor and any variations between different sensors of the same model)
    float accelMagnitude = sqrt(accelX_moyen * accelX_moyen + accelY_moyen * accelY_moyen + accelZ_moyen * accelZ_moyen);
    AccelFC = accelMagnitude / 16384.0f;

    // we store the accelero biases in LSB to subtract them before the conversion, to get more accurate measurements for the filter and the control loop, and to avoid any issues with the conversion factor that could arise from the biases in the measurements
    // we stock the accelero biases in LSB to avoid any issues with the conversion factor that could arise from the biases in the measurements
    offsetAccelX = accelX_moyen;
    offsetAccelY = accelY_moyen;
    offsetAccelZ = accelZ_moyen - AccelFC * 16384.0f; // we subtract 1g in LSB to get the actual bias of the Z axis, which should be equal to 1g when the sensor is at rest, to get more accurate measurements for the filter and the control loop, and to avoid any issues with the conversion factor that could arise from the biases in the measurements
}
    //communication
    void wire_begin(int sda, int scl) {
      Wire.begin(sda, scl);
      Wire.beginTransmission(0x68); //start
        Wire.write(0x6B); //register to wake up the MPU-6050 from sleep mode
        Wire.write(0x00);    // set the register to 0 to wake up the sensor
        Wire.endTransmission(true); // end
        calibrerIMU();
        tempsPrecedent = millis();
    }

    void Beta_Modif() { // to modify the BETA parameter of the madgwick filter in real time from the terminal (tuning)
    if (Serial.available() > 0) {
        float newBeta = Serial.parseFloat(); 
        if (newBeta > 0) {
            BETA = newBeta;
            Serial.print("Nouveau BETA : ");
            Serial.println(BETA, 4);
        }
    }
}

   void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float DT) {
    float recipNorm;
    float qDot0, qDot1, qDot2, qDot3;
    float s0, s1, s2, s3;

    // quaternionss deriviateds from gyroscope
    qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // acceleration mesures normalised 
    if (ax == 0.0f && ay == 0.0f && az == 0.0f) return; // no division over 0
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    if (recipNorm > 0.8f && recipNorm < 1.2f) { // no  abberation in the acceleration mesure
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    } else {
        return; 
    }

    // determining the error between the estimated direction of gravity and the measured direction of gravity, and calculating a corrective step based on this error and the BETA parameter
    float fx = 2.0f * (q1 * q3 - q0 * q2) - ax;
    float fy = 2.0f * (q0 * q1 + q2 * q3) - ay;
    float fz = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

    // calculating the Jacobian and the gradient (matrix multiplication)
    s0 = -2.0f * q2 * fx + 2.0f * q1 * fy;                 // -2*q2*fx + 2*q1*fy
    s1 =  2.0f * q3 * fx + 2.0f * q0 * fy - 4.0f * q1 * fz; // 2*q3*fx + 2*q0*fy - 4*q1*fz
    s2 = -2.0f * q0 * fx + 2.0f * q3 * fy - 4.0f * q2 * fz; // -2*q0*fx + 2*q3*fy - 4*q2*fz
    s3 =  2.0f * q1 * fx + 2.0f * q2 * fy;                 // 2*q1*fx + 2*q2*fy

    // Normalisation of the gradient
    recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // correction step is applied to the quaternion derivative
    qDot0 -= BETA * s0;
    qDot1 -= BETA * s1;
    qDot2 -= BETA * s2;
    qDot3 -= BETA * s3;

    // Integration
    q0 += qDot0 * DT;
    q1 += qDot1 * DT;
    q2 += qDot2 * DT;
    q3 += qDot3 * DT;

    // Normalisation des quaternions
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // conersion to euler angles in degrees
    angleRoll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    anglePitch = asinf(2.0f * constrain(q0 * q2 - q1 * q3, -1.0f, 1.0f)) * 180.0f / M_PI;
    angleYaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
}
    float MettreAjourmesures() {
        lireDonneesBrutes();
        // calibrated
        gyroX -= erreurGyroX;
        gyroY -= erreurGyroY;
        gyroZ -= erreurGyroZ;
        // Conversion to physical units        // Convert to radians/s
        gyroX = (gyroX/GyroFC)*M_PI/180.0f; 
        gyroY = (gyroY/GyroFC)*M_PI/180.0f;
        gyroZ = (gyroZ/GyroFC)*M_PI/180.0f;

        accelX = accelX/AccelFC;
        accelY = accelY/AccelFC;
        accelZ = accelZ/AccelFC;

        accelX -= offsetAccelX;
        accelY -= offsetAccelY;
        accelZ -= offsetAccelZ;

        unsigned long tempsActuel = micros();
        float diffTemps = tempsActuel - tempsPrecedent;
        tempsPrecedent = tempsActuel;
        float dt = diffTemps/1000000.0;
        dt = max(0.0001f, dt); // avoid divisions by zero in the madgwick filter
        madgwickUpdate(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, dt);
      // return the dt for the PID loop
       return dt;
      }   
    float getAngleRoll() { return angleRoll; }
    float getAnglePitch() { return anglePitch; }  
    float getAngleYaw() { return angleYaw; }
};

IMU monIMU;

class Emitor_receptor { 
  private:
    RF24 radio;
    const byte address[6] = "00001";
    const unsigned long interval = 100; // sending interval in ms
    unsigned long dernierEnvoi;
  public:
    Emitor_receptor() : radio(21, 22) {} // CE, CSN pins
    void begin() {
      radio.begin();
      radio.openReadingPipe(0, address);
      radio.setPALevel(RF24_PA_HIGH);
      radio.startListening();
      dernierEnvoi = millis();
    }

    bool receivePacket(ControlData& packet) { // receive from GS 
        if (radio.available()) {
            radio.read(&packet, sizeof(packet));
            return true;
        } else return false;        
    }
    bool sendPacket(const DroneData& packetDrone) { // send to GS
        dernierEnvoi = millis();
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


class moteur {
  private :
  const int Moteur_pin;
  const int canal;
  float vitesseMoteur;
  int PWMvalue;
  int vitesseMin = 1000;
  int vitesseMax = 2000;
  bool estArmer = false;
  
  public:
  moteur(const int& pin, const int& canal, float vitesse)
  : Moteur_pin(pin), canal(canal), vitesseMoteur(vitesse) {

  }
  void start() {
     pinMode(Moteur_pin, OUTPUT);
     ledcSetup(canal, 50, 14); // 50 Hz, 14-bit resolution
     ledcAttachPin(Moteur_pin, canal);
     ledcWrite(canal, 0); // Démarre à 0% de duty cycle (moteur éteint)
  }
  void armed() {
    estArmer=true;
  } 
  void unarmed() {
    estArmer=false;
  } 
  void vitCtrl(float vitesseRecue) {
 
    if (estArmer == false) {
      vitesseMoteur = vitesseMin;

    } else {
      vitesseMoteur = constrain(vitesseRecue, vitesseMin, vitesseMax);
    }
    // speed convertion to PWM value for the ESC 
    PWMvalue = 16384*(vitesseMoteur/20000.0); // 14-bit resolution
    ledcWrite(canal, PWMvalue);

  }
};

class mixMotor {
  private:
  moteur m_ad;
  moteur m_ag;
  moteur m_dd;
  moteur m_dg;
 
  public : 
   mixMotor() : m_ad(12,1,1000), m_ag(13,2,1000), m_dd(14,3,1000), m_dg(27,4,1000) {}  

   void start() {
    m_ad.start(); m_ag.start(); m_dd.start(); m_dg.start();
   }
   //protection sup +
   void stopTout() {
        m_ad.unarmed(); m_ag.unarmed(); 
        m_dd.unarmed(); m_dg.unarmed();
    }

  void arming() {
    m_ad.armed(); m_ag.armed();
    m_dd.armed(); m_dg.armed();
    
    }

  void appliquer(float thr, float p, float r,  float y) {
    thr = constrain(thr, 1000,1600);p = constrain(p, 0,100);r = constrain(r, 0,100);y = constrain(y, 0,100);
    m_ag.vitCtrl(constrain(thr + p + r + y, 1000, 2000));
    m_ad.vitCtrl(constrain(thr + p - r - y, 1000, 2000)); 
    m_dg.vitCtrl(constrain(thr - p + r - y, 1000, 2000)); 
    m_dd.vitCtrl(constrain(thr - p - r + y, 1000, 2000));
  }
};

mixMotor monMix;

class PID {
  private :
  float kp, ki, kd;
  float sumErreur = 0;
  float erreurPre = 0;
  unsigned long lastMicros = 0;
  float DT = 0.0001;
  const float DT_MAX = 0.001f; // 0.001 second, to avoid big drifts of the PID in case of code blocking or abberant mesures
  public: 
  PID(float p, float i,float d) : kp(p), ki(i), kd(d)  {}

  float dt_init() {
    unsigned long currentMicros = micros();
    
    DT = (currentMicros - lastMicros) / 1000000.0f; // conversion to seconds
    lastMicros = currentMicros;
    if (DT > DT_MAX) DT = DT_MAX ; // avoid big drifts of the PID in case of code blocking or abberant mesures
    return DT;
  }

  float calcErreur(float demand, float mesure) {
     
    if (isnan(mesure) || isinf(mesure)) {
    return 0.0f; // return 0 if mesure is invalid
    }
    DT = dt_init();    
    float erreur = demand - mesure;

    float P = erreur * kp;
   
    sumErreur += erreur * DT;
    // anti windup
    if (sumErreur < -400) sumErreur = -400;
    if (sumErreur > 400) sumErreur = 400;
    float I = sumErreur *ki ;

    float D =0;
    if (DT >= 0.0001f) D = kd * (erreur-erreurPre)/DT;
    erreurPre = erreur;

    return P + I + D; 
  }

};

class failsafe {
  private:
    unsigned long lastSignalTime;
    const unsigned long timeoutmin; // in milliseconds
    const unsigned long timeoutmax ; // in milliseconds
   
  public:
    failsafe(unsigned long timeoutmin, unsigned long timeoutmaxs) : timeoutmin(timeoutmin), timeoutmax(timeoutmaxs) {
      lastSignalTime = millis();
    }

    void updateSignalTime() { // to be called when a valid signal is received
        lastSignalTime = millis();
    }

    bool temporaryLoss() {
      if (millis() - lastSignalTime > timeoutmin)
      return true;
      return false;
    } 
    bool criticalLoss() {
      if (millis() - lastSignalTime > timeoutmax)
      return true;
      return false;
    }
    void failsafeAction(mixMotor& mixer) {
      if (criticalLoss() ) {
        mixer.stopTout();
      } else if (temporaryLoss() ) {
        if (xSemaphoreTake(xMutexControll, 0) == pdTRUE){
          order.thr = 1300.0f; // reduce throttle to a low value to allow for a safe landing if the signal comes back, but cut the PID corrections to avoid any sudden movements of the drone in case of abberant mesures or code blocking, and to allow for a safe landing if the signal comes back.
          order.pitch = 0.0f;
          order.roll = 0.0f;
          order.yaw = 0.0f;
          order.armed = false; // Disarm the drone
          xSemaphoreGive(xMutexControll);
        }; // stabilisation incase of temporary loss of signal, to avoid a crash while waiting for the signal to come back, by cutting the throttle and disarming the drone, but keeping the PID corrections to 0 to avoid any sudden movements of the drone in case of abberant mesures or code blocking, and to allow for a safe landing if the signal comes back.
    } else {
        // normal operation, do nothing
      }
    }
};


failsafe monFailsafe(1000, 10000); // temporary delay / critical delay in ms, to be adjusted according to the expected signal loss duration and the desired failsafe behavior of the drone 

class update_data {
  private :
  std::atomic <unsigned long> dernier_update {0};
  const unsigned long delai_update;
  std::atomic <bool> updated{false};
  public :
  update_data(unsigned long delaiupdate) : delai_update(delaiupdate)  {} // delaiupdate is the maximum allowed time between two valid data reception from the GS

  void data(std::atomic <float> &THR,  std::atomic <float> &PITCH,  std::atomic <float> &ROLL,  std::atomic <float> &YAW,  std::atomic <bool> &ARMED) {
   if (radio.receivePacket(Packet)) {
     THR = Packet.thr;
     PITCH = Packet.pitch;
     ROLL = Packet.roll;
     YAW = Packet.yaw;
     ARMED = Packet.armed;
    if (xSemaphoreTake(xMutexFailsafe, 0) == pdTRUE) {
      monFailsafe.updateSignalTime();
      xSemaphoreGive(xMutexFailsafe);
    }
   } 
   if (millis()-dernier_update > delai_update) {
    updated = false;
   } else updated = true; 
  }
  bool UPDATED() const { return updated.load(); }
};


// Global objects

PID PIDroll(1.0, 0.0, 0.0);
PID PIDpitch(1.0, 0.0, 0.0);
PID PIDyaw(1.0, 0.0, 0.0);

update_data data(500);

#define SDA_PIN 4
#define SCL_PIN 5
#define LED_PIN 2

const TickType_t frequency_controll = 4 / portTICK_PERIOD_MS; // 4 ms for the control loop,  because it is a good compromise between the performance of the PID loop and the performance of the ESP32 
const TickType_t frequency_captor = 2 / portTICK_PERIOD_MS; // 2 ms for the captor task, because it is enough for the PID/controll loop 
const TickType_t frequency_radio = 10 / portTICK_PERIOD_MS; // 100 Hz for the radio task , because it is enough for the radio communication and to avoid overloading the CPU with too many tasks running at high frequency

// initialisation of the tasks and the mutexes in the setup function

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  monIMU.wire_begin(SDA_PIN, SCL_PIN);
  pinMode(LED_PIN, OUTPUT);
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
  const TickType_t frequency = 4 / portTICK_PERIOD_MS; // 4 ms for the control loop, to be adjusted according to the needs of the PID loop and the performance of the ESP32, and to avoid overloading the CPU with too many tasks running at high frequency
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
    vTaskDelayUntil(&xLastWaketime, frequency_controll);
  }
}


 

// all the code is executed in a loop with a period of 4ms on core 1 in parallel with the reception and sending of data by radio on core 0
//    TODO LIST :
// still need to tune the PID and the BETA parameter of the madgwick filter, and to test the failsafe in case of loss of signal from the GS, and to test the whole system in flight.

// need to study and add magnetometer to stabilize the yaw 
// still need more documentation and comments in the code to explain the different parts of the code and the logic behind it, and to make it more readable and maintainable. 


//    FEATURES TO ADD LATER:
// will also add some safety features like a kill switch, and a buzzer to alert the user in case of critical failure or low battery.
// will also add some features like a GPS module for position hold and return to home, and a barometer for altitude hold.