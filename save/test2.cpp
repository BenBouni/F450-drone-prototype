#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <RF24.h>
#include <SPI.h>
#include <atomic>
#include <climits>
#include <Servo.h>

SemaphoreHandle_t xMutex;
void Radiocore(void * Pvparameter);

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
  Order order;
  telemerie tele;

class IMU {
  private:
    // MPU-6050 
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ; 
    //pour les calibrations imu :
    float erreurGyroX = 0, erreurGyroY = 0, erreurGyroZ = 0;
    // facteurs de conversions :
    const float GyroFC = 131.0;
    const float AccelFC = 16384.0;
    // Angles filtrés (ce que le PID utilisera)
    float angleRoll = 0.0;
    float anglePitch = 0.0;
    float angleYaw = 0.0; // Le Yaw est uniquement géré par le Gyro

    // Gestion du temps pour le filtre (Calcul de Delta t)
    unsigned long tempsPrecedent = 0;
    const float ALPHA = 0.98; // Coefficient de pondération du filtre complémentaire
  void lireDonneesBrutes() {
        // Demande au MPU de commencer à nous envoyer les 6 valeurs brutes
        Wire.beginTransmission(0x68);
        Wire.write(0x3B); // Le premier registre où se trouve AccelX
        Wire.endTransmission(false); // Garde la connexion ouverte

        // Demande 14 octets de donnees brutes (Accel X,Y,Z, Temp, Gyro X,Y,Z)
        Wire.requestFrom(0x68, 14); // disambiguate overload

        // Note : MPU-6050 stocke les données en 'High Byte' puis 'Low Byte'
        accelX = (int16_t)(Wire.read() << 8 | Wire.read()); 
        accelY = (int16_t)(Wire.read() << 8 | Wire.read());
        accelZ = (int16_t)(Wire.read() << 8 | Wire.read());
        
        // pour ignorer Lecture de la température 
        int16_t temp = (Wire.read() << 8) | Wire.read();
        
        gyroX = (int16_t)(Wire.read() << 8 | Wire.read()); 
        gyroY = (int16_t)(Wire.read() << 8 | Wire.read());
        gyroZ = (int16_t)(Wire.read() << 8 | Wire.read());
    }
  public :
  
  //calibration de l'imu
  void calibrerIMU() {
        float sommeX = 0, sommeY = 0, sommeZ = 0;
        for(int i = 0; i < 500; i++) {
            lireDonneesBrutes();
            sommeX += gyroX;
            sommeY += gyroY;
            sommeZ += gyroZ;
        }
        erreurGyroX = sommeX / 500;
        erreurGyroY = sommeY / 500;
        erreurGyroZ = sommeZ / 500;
        delay(10); //
    }  
    //communication
    void wire_begin(int sda, int scl) {
      Wire.begin(sda, scl);
      Wire.setClock(400000);
      Wire.beginTransmission(0x68); //commmencer
      Wire.write(0x6B); //reveil
      Wire.write(0x00); //lecture
      
      Wire.endTransmission(true); // fin
      calibrerIMU();
      tempsPrecedent = micros();   
  }
  

  IMU() { 
      angleRoll = 0.0;
      anglePitch = 0.0;
      tempsPrecedent = micros();
    }
    
    float MettreAjourmesures() {
        lireDonneesBrutes();
        //données calibrées
        gyroX -= erreurGyroX;
        gyroY -= erreurGyroY;
        gyroZ -= erreurGyroZ;
        
        gyroX = gyroX/GyroFC;
        gyroY = gyroY/GyroFC;
        gyroZ = gyroZ/GyroFC;

        accelX = accelX/AccelFC;
        accelY = accelY/AccelFC;
        accelZ = accelZ/AccelFC;

        unsigned long tempsActuel = micros();
        float diffTemps = tempsActuel - tempsPrecedent;
        if (tempsActuel < tempsPrecedent) {
           diffTemps = (ULONG_MAX - tempsPrecedent) + tempsActuel;
        }
        tempsPrecedent = tempsActuel;
        float dt = diffTemps/1000000.0;
          if (isnan(dt) || isinf(dt) || dt > 0.1f) {
              dt = 0.004f; // Valeur sécuritaire
          }
        //projection de l'accel : 
        float angleAccelRoll = atan2(accelY,accelZ) * 180/M_PI;
        float angleAccelPitch = atan2(-accelX,sqrt(accelY*accelY+accelZ*accelZ)) * 180/M_PI;

        //nos angles finals : 
        angleRoll = ALPHA*(gyroX*dt + angleRoll) +(1-ALPHA)*angleAccelRoll;
        anglePitch = ALPHA*(gyroY*dt + anglePitch) +(1-ALPHA)*angleAccelPitch;
        angleYaw += (gyroZ*dt ); // accel Z n'a pas de projection 
        return dt;
      } 
  float getRoll() { return angleRoll; }
  float getPitch() { return anglePitch; }
  float getYaw() { return angleYaw; }
          
};
IMU monIMU;

class Emitor_receptor { // Récepteur RF24
  private:
    RF24 radio;
    const byte address[6] = "00001";
    const unsigned long interval = 100; // Intervalle d'envoi en ms
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
    // Convertir la vitesse en microsecondes pour le signal PWM
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

  public: 
  PID(float p, float i,float d) : kp(p), ki(i), kd(d) {}

  float calcErreur(float demand, float mesure, float dt) {

    if (isnan(mesure) || isinf(mesure)) {
    return 0.0f; // Retourne 0 si mesure invalide
    }
    float erreur = demand - mesure;

    float P = erreur * kp;
   
    sumErreur += erreur * dt ;
    if (sumErreur < -400) sumErreur = -400;
    if (sumErreur > 400) sumErreur = 400;
    float I = sumErreur *ki ;

    float D =0;
    if (dt > 0.0001f) D = kd * (erreur-erreurPre)/dt;
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
        if (xSemaphoreTake(xMutex, 0) == pdTRUE){
          order.thr = 1300.0f; // Cut throttle
          order.pitch = 0.0f;
          order.roll = 0.0f;
          order.yaw = 0.0f;
          order.armed = false; // Disarm the drone
          xSemaphoreGive(xMutex);
        }; // stationnaire
    } else {
        // normal operation, do nothing
      }
    }
};


failsafe monFailsafe(1000, 10000);

class update_data {
  private :
  std::atomic <unsigned long> dernier_update {0};
  const unsigned long delai_update;
  std::atomic <bool> updated{false};
  public :
  update_data(unsigned long delaiupdate) : delai_update(delaiupdate) {}

  void data(std::atomic <float> &THR,  std::atomic <float> &PITCH,  std::atomic <float> &ROLL,  std::atomic <float> &YAW,  std::atomic <bool> &ARMED) {
   if (radio.receivePacket(Packet)) {
     THR = Packet.thr;
     PITCH = Packet.pitch;
     ROLL = Packet.roll;
     YAW = Packet.yaw;
     ARMED = Packet.armed;
    dernier_update = millis();
    monFailsafe.updateSignalTime();
   } 
   if (millis()-dernier_update > delai_update) {
    updated = false;
   } else updated = true; 
  }
  bool UPDATED() const { return updated.load(); }
};


PID PIDroll(1.0, 0.0, 0.0);
PID PIDpitch(1.0, 0.0, 0.0);
PID PIDyaw(1.0, 0.0, 0.0);
PID PIDthr(1.0, 0.0, 0.0);

float dt;

update_data data(500);

#define SDA_PIN 4
#define SCL_PIN 5
#define LED_PIN 2
unsigned long tempsCyclePre = 0;
const unsigned long PERIOD_CYCLE = 4000; // 250 Hz



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
  xMutex = xSemaphoreCreateMutex();
      xTaskCreatePinnedToCore(
        Radiocore,
        "radioCORE",
        10000,
        NULL,
        1,
        NULL,
        0
    );
}

void loop() {
  static unsigned long last_arm;
  static bool isArmed = false;
  static float r_order, p_order, y_order, thr_order;
   unsigned long currentTime = millis();

if (micros() - tempsCyclePre > PERIOD_CYCLE ){
  //mise a jour des mesures

//mise a jour commande
if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
    r_order = order.roll;
    p_order = order.pitch;
    y_order = order.yaw;
    thr_order = order.thr;

    if(order.armed && !isArmed) {
      if (millis() - last_arm > 500 ) {isArmed = true; last_arm = millis();}
    } else if (!order.armed && isArmed) {if (millis() - last_arm > 500 ) {isArmed = false; last_arm = millis();}}
    xSemaphoreGive(xMutex);
  } else {
    //stabilistion en cas de perte
    r_order = 0;
    p_order = 0;
    y_order = 0;
    monFailsafe.failsafeAction(monMix);
  }  
  dt = monIMU.MettreAjourmesures(); 
  float mesureRoll = monIMU.getRoll();
  float mesurePitch = monIMU.getPitch();
  float mesureYaw = monIMU.getYaw();
      // calculate correction
    float Rollcorr = PIDroll.calcErreur(r_order, mesureRoll, dt );
    float Yawcorr = PIDyaw.calcErreur(y_order, mesureYaw, dt);
    float Pitchcorr = PIDpitch.calcErreur(p_order, mesurePitch, dt);

    if (order.armed.load() == true) {
     // apply motor mixing
     monMix.appliquer(order.thr, Pitchcorr, Rollcorr, Yawcorr );
  } else {
      monMix.stopTout();
  }
  // prepare data to send to GS
  tele.ActualPitch = monIMU.getPitch();
  tele.ActualRoll = monIMU.getRoll();
  tele.ActualYaw = monIMU.getYaw();
  tele.batteryVoltage = 11.1; // Example voltage

  tempsCyclePre = micros();
  if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
  dronepacket.batteryVoltage = tele.batteryVoltage; // Example voltage
  dronepacket.ActualPitch = tele.ActualPitch;
  dronepacket.ActualRoll = tele.ActualRoll;
  dronepacket.ActualYaw = tele.ActualYaw;
  xSemaphoreGive(xMutex);
    }
  //debug  
  //Serial.print(monIMU.getRoll());
  //Serial.println(monIMU.getPitch());
  //Serial.println(monIMU.getYaw());    
  }
}
// note : la boucle d execution se fait en 4ms pour le core 1  en parallel de la reception et envoie des données par radio dans le core 0

void Radiocore(void * Pvparameter) {
   radio.begin();
   for(;;){
    radio.receivePacket(Packet); 
    if (xSemaphoreTake(xMutex, 0) == pdTRUE) {            
       data.data(order.thr, order.pitch, order.roll, order.yaw, order.armed);
      xSemaphoreGive(xMutex);
      }
      radio.sendDroneData(dronepacket);
      vTaskDelay(4 / portTICK_PERIOD_MS);
   }
}

