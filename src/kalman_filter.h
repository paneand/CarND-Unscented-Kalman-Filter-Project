#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"


using namespace std;

/**
Interface for a generic Kalman Filter with the three operations: init, predict and update.
*/
class KalmanFilter {
public:
  /**
   
   * @param Q_in Process covariance matrix
   */
  virtual void Init(const MeasurementPackage &measurement_pack) = 0;

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  virtual void Predict(const MeasurementPackage &measurement_pack) = 0;

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  virtual void Update(const MeasurementPackage &measurement_pack) = 0;
  
};

#endif /* KALMAN_FILTER_H_ */
