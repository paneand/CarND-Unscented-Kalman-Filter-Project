#include "fusion_ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <limits>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
FusionUKF::FusionUKF() {
  skip_radar_ = false;
  skip_laser_ = false;
  is_initialized_ = false;    // We are creating the intance of FusionEKF, but the filter has not yet been initialized
  time_us_ = 0;
  n_x_ = 5;                   // State dimensions
  n_aug_ = 7;                 // State+noise dimensions
  n_z_rad_ = 3;               // Number of scalar values provided by the radar measurement
  n_z_las_ = 2;               // Number of scalar values provided by the lader measurement
  lambda_ = 3 - n_aug_;       // Spreading parameter for Sigma points
  x_ = VectorXd(n_x_);        // State space: (px,py,v_magnitude,yaw,yawrate)
  P_ = MatrixXd(n_x_, n_x_);  // State covariance matrix. It represents the uncertainty of our current belief of the state.
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);    // Predicted Sigma points matrix
  weights_ = VectorXd(2*n_aug_+1);                // Weights for sigma points
  std_a_ = 0.4;               // Process noise standard deviation longitudinal acceleration in m/s^2. We expect 95% of the times the acceleration will be lower than 2*std_a_ 
  std_yawdd_ = 0.45;          // Process noise standard deviation yaw acceleration in rad/s^2. We expect 95% of the times the acceleration will be lower than this 2*std_yawdd_
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer. 
  std_laspx_ = 0.15;          // Laser measurement noise standard deviation position1 in m
  std_laspy_ = 0.15;          // Laser measurement noise standard deviation position2 in m
  std_radr_ = 0.3;            // Radar measurement noise standard deviation radius in m                     
  std_radphi_ = 0.03;         // Radar measurement noise standard deviation angle in rad
  std_radrd_ = 0.3;           // Radar measurement noise standard deviation radius change in m/s
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.


  // Given by the problem
  i_v_ = 5.199937;
  i_yaw_ = 0;
  i_yawrate_ = 0.006911322;
}

FusionUKF::~FusionUKF() {}





VectorXd FusionUKF::GetEstimate(const MeasurementPackage &measurement_pack){
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      is_radar_measurement_ = true;
      if(skip_radar_){
        VectorXd special(1);
        special(0) = std::numeric_limits<float>::max();   // escape value
        return special;
      }
    }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      is_radar_measurement_ = false;
      if(skip_laser_){
        VectorXd special(1);
        special(0) = std::numeric_limits<float>::max();   // escape value
        return special;
      }
    }
  if (!is_initialized_) {
    Init(measurement_pack);
    return x_;
  }
  Predict(measurement_pack);
  Update(measurement_pack);
  // print the output
  //cout << "x_ = " << x_ << endl;
  //cout << "P_ = " << P_ << endl;
  return x_;
  }

void FusionUKF::Update(const MeasurementPackage &measurement_pack){
  if(is_radar_measurement_){
      UpdateRadar(measurement_pack);
    }
  else{
      UpdateLidar(measurement_pack);
    }
}

void FusionUKF::Init(const MeasurementPackage &measurement_pack){
    if (is_radar_measurement_) {
      InitFromRadar(measurement_pack);
    }
    else {
      InitFromLidar(measurement_pack);
    }
    time_us_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
}
void FusionUKF::InitFromRadar(const MeasurementPackage &measurement_pack){
  VectorXd x = VectorXd(5);
  float rho,phi,px,py;
  cout << "First measurement = radar" << endl;
  rho = measurement_pack.raw_measurements_[0];
  phi = measurement_pack.raw_measurements_[1];
  px = rho*cos(phi);
  py = rho*sin(phi);
  x_ << px, py, i_v_, i_yaw_, i_yawrate_;
  // State covariance: it represents linear relation (covariance) between the state variables (therefore symmetrical)
  // It should converge to smaller values as there should not be correlation between them
  // We directly measured px and py to set the state, therefore we assume pxstd and pystd as the ones from the sensor 
  // For v,yaw,yawrate we set these value arbitrarly since we knew the GT values. 
  P_ << std_radphi_+std_radr_, 0, 0, 0, 0,    
        0, std_radphi_+std_radr_, 0, 0, 0, 
        0, 0, 0.1, 0, 0,
        0, 0, 0, 0.1, 0,
        0, 0, 0, 0,   0.1;
}
void FusionUKF::InitFromLidar(const MeasurementPackage &measurement_pack){
  VectorXd x = VectorXd(5);
  float px,py;
  cout << "First measurement = laser" << endl;
  px = measurement_pack.raw_measurements_[0];
  py = measurement_pack.raw_measurements_[1];
  x_ << px, py, i_v_, i_yaw_, i_yawrate_;
  P_ << std_laspx_, 0, 0, 0, 0,    
            0, std_laspy_, 0, 0, 0, 
            0, 0, 0.1, 0, 0,
            0, 0, 0, 0.1, 0,
            0, 0, 0, 0,   0.1;
}
void FusionUKF::Predict(const MeasurementPackage &measurement_pack){
  //compute the time elapsed between the current and previous measurements and updates timestamp
  float dt = (measurement_pack.timestamp_ - time_us_) / 1000000.0;     //dt - expressed in seconds
  time_us_ = measurement_pack.timestamp_;
  MatrixXd Xsig_aug = GenerateSigmaPoints();
  PredictSigmaPoints(Xsig_aug,dt);
  UpdateMeanStateAndCovarianceFromPredictedSigma();
}
MatrixXd FusionUKF::GenerateSigmaPoints(){
  /*
  ##############################                       ##############################
  ############################## GENERATE SIGMA POINTS ##############################
   */
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  //create augmented mean state
  x_aug.head(5) = x_;
  // Process noise --> it is a gaussian distribution with mean 0 and covariance equals to stda_ and stYadd
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
  return Xsig_aug;
}
void FusionUKF::PredictSigmaPoints(MatrixXd Xsig_aug, float delta_t){
  /*
  ##############################                       ##############################
  ############################## PREDICT SIGMA POINTS ##############################
   */
  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
}
void FusionUKF::UpdateMeanStateAndCovarianceFromPredictedSigma(){
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights_
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}
void FusionUKF::UpdateLidar(MeasurementPackage meas_package) {
  // mapping from state space to laser measurement space is linear, standard KF is applied
  MatrixXd H_laser = MatrixXd(n_z_las_,n_x_);
  MatrixXd R_laser = MatrixXd(n_z_las_,n_z_las_);
  R_laser << std_laspx_*std_laspx_, 0,      
        0,  std_laspy_*std_laspy_;
  H_laser << 1, 0, 0, 0, 0,
                 0, 1, 0, 0, 0; 
  VectorXd y,yt,z,z_pred;
  // Init values
  z = VectorXd(2);
  z_pred = VectorXd(2);
  z << meas_package.raw_measurements_[0],meas_package.raw_measurements_[1];
  z_pred = H_laser * x_;
  y = z - z_pred;
  // Update step 
  MatrixXd Ht = H_laser.transpose();
  MatrixXd S = H_laser * P_ * Ht + R_laser;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  P_ = (I - K * H_laser) * P_;
  UpdateNIS(y,Si);
}
void FusionUKF::UpdateRadar(MeasurementPackage meas_package) {
  VectorXd z = VectorXd(3);
  z << meas_package.raw_measurements_[0],meas_package.raw_measurements_[1],meas_package.raw_measurements_[2];
  MatrixXd Zsig = MapPredictedSigmaPointsIntoMeasurementSpace();
  VectorXd z_pred = GetMeanPredictedMeasurement(Zsig);
  MatrixXd S = GetInnovationCovariance(Zsig,z_pred);
  MatrixXd Tc = GetCrossCorrelation(Zsig,z_pred);
  MatrixXd K = Tc * S.inverse();    //Kalman gain K;

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  UpdateNIS(z_diff,S.inverse());
} 
void FusionUKF::UpdateNIS(VectorXd y, MatrixXd Si){
  int total_measurements = 0;
  int n_below_tresh = 0;
  float rate = 0;
  float lidar_tresh = 5.991;
  float radar_tresh = 7.815;


  MatrixXd nis = y.transpose() * Si * y;
  if(is_radar_measurement_){  // RADAR NIS
      nis_radar_.push_back(nis(0,0));
      cout << "Radar Nis " << nis(0,0) << ";\t";
      total_measurements = nis_radar_.size();
      vector<double>::iterator it;
      for (it = nis_radar_.begin(); it != nis_radar_.end(); ++it){
        if(*it<=radar_tresh){    
          n_below_tresh++;
        }
      }
  }
  else{
      nis_lidar_.push_back(nis(0,0));
      cout << "Lidar Nis " << nis(0,0) << ";\t";
      total_measurements = nis_lidar_.size();
      vector<double>::iterator it;
      for (it = nis_lidar_.begin(); it != nis_lidar_.end(); ++it){
        if(*it<=lidar_tresh){    
          n_below_tresh++;
        }
      }
  }
  rate = n_below_tresh/(float)total_measurements;
  cout << "Rate:  "<< rate << endl;
}
MatrixXd FusionUKF::MapPredictedSigmaPointsIntoMeasurementSpace() {
  MatrixXd Zsig = MatrixXd(n_z_rad_, 2 * n_aug_ + 1);
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //     2n+1 sigma points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }
  return Zsig;
}
VectorXd FusionUKF::GetMeanPredictedMeasurement(MatrixXd Zsig){
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_rad_);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  return z_pred;
}
MatrixXd FusionUKF::GetInnovationCovariance(MatrixXd Zsig, VectorXd z_pred){
  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z_rad_,n_z_rad_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_rad_,n_z_rad_);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;
  return S;
}
MatrixXd FusionUKF::GetCrossCorrelation(MatrixXd Zsig, VectorXd z_pred){
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_rad_);
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  return Tc;
}


