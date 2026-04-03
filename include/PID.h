#ifndef PID_H
#define PID_H

#include <math.h>

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

extern PID PIDroll;
extern PID PIDpitch;
extern PID PIDyaw;

#endif // PID_H 