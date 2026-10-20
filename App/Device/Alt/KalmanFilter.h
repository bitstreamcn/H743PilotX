#pragma once



  void KalmanFilter_setup(float altCov, float accCov, float biasCov);
  void KalmanFilter_setup1(float altCov, float accCov);
  void KalmanFilter_setup2();

  /**
  * Propagate the state.
  * @param acceleration vertical acceleration in Earth frame (positive in the zenith direction) [m/s^2].
  * @param dt update/sampling period [s].
  */
  void KalmanFilter_propagate(float acceleration, const float dt);

  /**
   * State correction update.
   * @param altitude measurement of altitude in Earth frame (positive in the zenith direction) [m].
   */
  void KalmanFilter_update(float altitude);

  /**
   * State correction update. Use this method if you use multiple sensors measuring the altitude.
   * @param altitude measurement of altitude in Earth frame (positive in the zenith direction) [m].
   * @param altCov covariance of the altitude measurement (σ^2).
   */
  void KalmanFilter_update1(float altitude, float altCov);


  float KalmanFilter_getH();
  float KalmanFilter_getV();
