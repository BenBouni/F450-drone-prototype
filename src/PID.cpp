#include "PID.h"

 
 // PID class implementation
    PID::PID(float p, float i,float d) : kp(p), ki(i), kd(d)  {}

  float PID::dt_init() {
    unsigned long currentMicros = micros();
    
    DT = (currentMicros - lastMicros) / 1000000.0f; // conversion to seconds
    lastMicros = currentMicros;
    if (DT > DT_MAX) DT = DT_MAX ; // avoid big drifts of the PID in case of code blocking or abberant mesures
    return DT;
  }

  float PID::calcErreur(float demand, float mesure) {
     
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
 // the correction
    return P + I + D; 
  }