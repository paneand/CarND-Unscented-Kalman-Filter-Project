#ifndef FUSION_UKF_H
#define FUSION_UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include "fusion_estimator.h"
#include "kalman_filter.h"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
This class inherits from KalmanFilter and FusionEstimator to provide an interface to estimate the position of an object
leveraging on the EKF equations, combining Lidar and Radar measurements. 
*/
class FusionUKF : public KalmanFilter, public FusionEstimator {
public:
  /**
   * Constructor
   */
  FusionUKF();

  /**
   * Destructor
   */
  ~FusionUKF();

  void Init(const MeasurementPackage &measurement_pack);

  void Predict(const MeasurementPackage &measurement_pack);

  void Update(const MeasurementPackage &measurement_pack);


  VectorXd GetEstimate(const MeasurementPackage &measurement_pack);

private:


  void InitFromRadar(const MeasurementPackage &measurement_pack);
  void InitFromLidar(const MeasurementPackage &measurement_pack);
  MatrixXd GenerateSigmaPoints();
  void PredictSigmaPoints(MatrixXd Xsig_aug, float delta_t);
  void UpdateMeanStateAndCovarianceFromPredictedSigma();
  void UpdateLidar(MeasurementPackage meas_package);
  void UpdateRadar(MeasurementPackage meas_package);
  MatrixXd MapPredictedSigmaPointsIntoMeasurementSpace();
  VectorXd GetMeanPredictedMeasurement(MatrixXd Zsig);
  MatrixXd GetInnovationCovariance(MatrixXd Zsig, VectorXd z_pred);
  MatrixXd GetCrossCorrelation(MatrixXd Zsig, VectorXd z_pred);
  void UpdateNIS(VectorXd y, MatrixXd Si);
  bool is_initialized_;
  bool is_radar_measurement_;           // True means last input measure is radar, false means Lidar measurament
  bool skip_radar_;       
  bool skip_laser_;       
  VectorXd x_;                          ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  MatrixXd P_;                          ///* state covariance matrix
  MatrixXd Xsig_pred_;                  ///* predicted sigma points matrix
  long long time_us_;                   ///* time when the state is true, in us
  double std_a_;                        ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_yawdd_;                    ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_laspx_;                    ///* Laser measurement noise standard deviation position1 in m
  double std_laspy_;                    ///* Laser measurement noise standard deviation position2 in m
  double std_radr_;                     ///* Radar measurement noise standard deviation radius in m
  double std_radphi_;                   ///* Radar measurement noise standard deviation angle in rad
  double std_radrd_ ;                   ///* Radar measurement noise standard deviation radius change in m/s
  VectorXd weights_;                    ///* Weights of sigma points
  int n_x_;                             ///* State dimension
  int n_aug_;                           ///* Augmented state dimension
  double lambda_;                       ///* Sigma point spreading parameter
  int n_z_rad_;                         // Number of measurements of radr
  int n_z_las_;                         // Number of measurements of lasr
  float i_v_, i_yaw_, i_yawrate_;       // Initial values
  vector<double> nis_lidar_;            // Vector of lidar NIS
  vector<double> nis_radar_;            // Vector of radar NIS
};

#endif /* UKF_H */
