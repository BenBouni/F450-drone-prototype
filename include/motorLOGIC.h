#ifndef MOTORLOGIC_H
#define MOTORLOGIC_H

#include <Wire.h>   
#include <math.h>
#include <Servo.h>
#include "data.h"  // for shared-order/telemetry structs
#include "Pins.h"

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
  moteur(const int& pin, const int& canal, float vitesse);

  void start();

  void armed();

  void unarmed();

  void vitCtrl(float vitesseRecue);
};

class mixMotor {
  private:
  moteur m_ad;
  moteur m_ag;
  moteur m_dd;
  moteur m_dg;
 
  public : 
   mixMotor();

  void start();

   //protection sup +
  void stopTout();

  void arming();

  void appliquer(float thr, float p, float r,  float y);
};

class PID {
  private :
  float kp, ki, kd;
  float sumErreur = 0;
  float erreurPre = 0;
  unsigned long lastMicros = 0;
  float DT = 0.0001;
  const float DT_MAX = 0.001f; // 0.001 second, to avoid big drifts of the PID in case of code blocking or abberant mesures
  public: 
  PID(float p, float i,float d);

  float dt_init();

  float calcErreur(float demand, float mesure);
};


extern mixMotor monMix;
extern PID PIDroll;
extern PID PIDpitch;    
extern PID PIDyaw;

#endif