#ifndef FusionEstimator_H_
#define FusionEstimator_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
Abstract class. It represents an estimator of the state of an object given the measurement from different sources
and provides the possibility to calculate the RMSE of a list of estimations. 
*/
class FusionEstimator {
public:

  FusionEstimator();
  ~FusionEstimator();

  /** Returns -1 in the first element of the VectorXd if the Estimator is not able to provide an estimation. **/
  virtual VectorXd GetEstimate(const MeasurementPackage &measurement_pack) = 0;

  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

private:
};

#endif /* FusionEstimator_H_ */