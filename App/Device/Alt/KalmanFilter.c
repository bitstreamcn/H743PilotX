#include "KalmanFilter.h"

// Tracks the position z and velocity v of an object moving in a straight line,
// (here assumed to be vertical) that is perturbed by random accelerations.
// sensor measurement of z is assumed to have constant measurement noise 
// variance zVariance,
// This can be calculated offline for the specific sensor, and is supplied 
// as an initialization parameter.


  // State being tracked
  static float h;  // Estimated vertical height or altitude in Earth frame (positive in the zenith direction) [m].
  static float v;  // Estimated vertical velocity (positive in the zenith direction) [m/s].
  static float bias;  // Estimated acceleration bias [m/s^2]

  static float accCov;  // dynamic acceleration variance
  static float altCov; //  z measurement noise variance fixed
  static float biasCov; // assumed fixed.


  // 3x3 State Covariance matrix
  static float Pzz_;
  static float Pzv_;
  static float Pza_;
  static float Pvz_;
  static float Pvv_;
  static float Pva_;
  static float Paz_;
  static float Pav_;
  static float Paa_;


void KalmanFilter_setup2()
{
	KalmanFilter_setup1(1.0, 0.5);
}

void KalmanFilter_setup1(float altCov, float accCov)
{
	KalmanFilter_setup(altCov, accCov, 0);
}

void KalmanFilter_setup(float _altCov, float _accCov, float _biasCov)
{

  altCov = _altCov;
  accCov = _accCov;
  biasCov = _biasCov;

  h = 0;
  v = 0;
  bias = 0;

  Pzz_ = 1.0f;
  Pzv_ = 0.0f;
  Pza_ = 0.0f;

  Pvz_ = 0.0f;
  Pvv_ = 1.0f;
  Pva_ = 0.0f;

  Paz_ = 0.0f;
  Pav_ = 0.0;
  Paa_ = 100000.0f;
}

void KalmanFilter_propagate(float a, const float dt) {
  // Predict state
  float accel = a - bias;
  v += accel * dt;
  h += v * dt;

  //accCov = fabs(accel)/50.0f;
  //CLAMP(accCov, 0.01f, 0.50f);

  // Predict State Covariance matrix
  float t00,t01,t02;
  float t10,t11,t12;
  float t20,t21,t22;

  float dt2div2 = dt*dt/2.0f;
  float dt3div2 = dt2div2*dt;
  float dt4div4 = dt2div2*dt2div2;

  t00 = Pzz_ + dt*Pvz_ - dt2div2*Paz_;
  t01 = Pzv_ + dt*Pvv_ - dt2div2*Pav_;
  t02 = Pza_ + dt*Pva_ - dt2div2*Paa_;

  t10 = Pvz_ - dt*Paz_;
  t11 = Pvv_ - dt*Pav_;
  t12 = Pva_ - dt*Paa_;

  t20 = Paz_;
  t21 = Pav_;
  t22 = Paa_;

  Pzz_ = t00 + dt*t01 - dt2div2*t02;
  Pzv_ = t01 - dt*t02;
  Pza_ = t02;

  Pvz_ = t10 + dt*t11 - dt2div2*t12;
  Pvv_ = t11 - dt*t12;
  Pva_ = t12;

  Paz_ = t20 + dt*t21 - dt2div2*t22;
  Pav_ = t21 - dt*t22;
  Paa_ = t22;

  Pzz_ += dt4div4*accCov;
  Pzv_ += dt3div2*accCov;

  Pvz_ += dt3div2*accCov;
  Pvv_ += dt*dt*accCov;

  Paa_ += biasCov;
}


void KalmanFilter_update(float z) {
  // Error
  float innov = z - h; 
  float sInv = 1.0f / (Pzz_ + altCov);

  // Kalman gains
  float kz = Pzz_ * sInv;
  float kv = Pvz_ * sInv;
  float ka = Paz_ * sInv;

  // Update state 
  h += kz * innov;
  v += kv * innov;
  bias += ka * innov;

  // Update state covariance matrix
  Paz_ -= ka * Pzz_;
  Pav_ -= ka * Pzv_;
  Paa_ -= ka * Pza_;

  Pvz_ -= kv * Pzz_;
  Pvv_ -= kv * Pzv_;
  Pva_ -= kv * Pza_;

  Pzz_ -= kz * Pzz_;
  Pzv_ -= kz * Pzv_;
  Pza_ -= kz * Pza_;
}

void KalmanFilter_update1(float altitude, float _altCov) {
    altCov = _altCov;
    KalmanFilter_update(altitude);
  }

float KalmanFilter_getH(){return h;}
float KalmanFilter_getV(){return v;}
