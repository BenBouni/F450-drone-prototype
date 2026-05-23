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

#endif