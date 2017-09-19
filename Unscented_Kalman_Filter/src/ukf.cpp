#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  is_initialized_ = false;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Projection Matrix for LIDAR measurement
  H_ = MatrixXd(2,5); // State vector is 5d in UKF
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.7;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5;
  n_aug_ = 7; // n_x_ + 2
  n_sig_ = 15; // 2*n_aug_ + 1
  lambda_ = 3 - n_x_;
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0, std_radrd_*std_radrd_;

  R_laser_ = MatrixXd(2,2);
  R_laser_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  weights_ = VectorXd(n_sig_);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=1; i < n_sig_; i++)
  {
    weights_(i) = 1/(2*(lambda_ + n_aug_));
  }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if(!is_initialized_)
  {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float rhoDot = meas_package.raw_measurements_[2];

      float x = rho * cos(phi);
      float y = rho * sin(phi);
      float vx = rhoDot * cos(phi);
      float vy = rhoDot * sin(phi);
      float v = sqrt(vx*vx + vy*vy);

      x_ << x, y, v, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      float x = meas_package.raw_measurements_[0];
      float y = meas_package.raw_measurements_[1];

      if (fabs(x) < 0.0001 and fabs(y) < 0.0001)
      {
        x = 0.0001;
        y = 0.0001;
      }
      x_ << x, y, 0, 0, 0;
    }

    old_time_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  // Predict -> Update
  double dt = (meas_package.timestamp_ - old_time_)/1000000.0; // in second
  old_time_ = meas_package.timestamp_;
  Prediction(dt);

  if (use_radar_ and meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
  else if (use_laser_ and meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // 

  // Generate sigma points -> Predict sigma points

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i=1; i<n_aug_+1; i++)
  {
      Xsig_aug.col(i) = x_aug + sqrt(lambda_ + n_aug_)*A.col(i-1);
      Xsig_aug.col(i+n_aug_) = x_aug - sqrt(lambda_ + n_aug_)*A.col(i-1);
  }

  // x_k+1 = x_k + a (motion) + b (noise)
  VectorXd a = VectorXd(5);
  VectorXd b = VectorXd(5);
          
  for (int i=0; i<n_sig_; i++)
  {
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double dYaw = Xsig_aug(4,i);
    double std_a = Xsig_aug(5,i);
    double std_yawdd = Xsig_aug(6,i);
    
      if(fabs(dYaw) < 0.001) // avoid 0-division
      {
          a << v*cos(yaw)*dt, v*sin(yaw)*dt, 0, 0, 0;
          b << 1/2*dt*dt*cos(yaw)*std_a, 1/2*dt*dt*sin(yaw)*std_a, dt*std_a, 1/2*dt*dt*std_yawdd, dt*std_yawdd;
      }
      else
      {
          a << v/dYaw*(sin(yaw+dYaw*dt)-sin(yaw)), v/dYaw*(-cos(yaw+dYaw*dt)+cos(yaw)), 0, dYaw*dt, 0;
          b << 1/2*std_a*dt*dt*cos(yaw), 1/2*std_a*dt*dt*sin(yaw), std_a*dt, 1/2*std_yawdd*dt*dt, std_yawdd*dt;
      }
      
    Xsig_pred_.col(i) = Xsig_aug.col(i).head(n_x_) + a + b;
  }

  x_.fill(0.0);
  // Predict new state mean
  for (int i=0; i<n_sig_; i++)
  {
      x_ += weights_(i)*Xsig_pred_.col(i);
  }
  P_.fill(0.0);
  for (int i=0; i<n_sig_; i++)
  {
    VectorXd xDiff = Xsig_pred_.col(i) - x_;
    while (xDiff(3)> M_PI) xDiff(3)-=2.*M_PI;
    while (xDiff(3)< -M_PI) xDiff(3)+=2.*M_PI;
    P_ += weights_(i)*xDiff*xDiff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  int n_z = 2;
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

  VectorXd z_pred = H_ * x_;
  VectorXd zDiff = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * zDiff);
  MatrixXd I = MatrixXd::Identity(5,5);
  P_ = (I - K * H_) * P_;
  
  NIS_laser_ = zDiff.transpose() * S.inverse() * zDiff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);
  
  //transform sigma points into measurement space
  for (int i=0; i<n_sig_; i++)
  {
      double px = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);
      double yawDot = Xsig_pred_(4,i);
      
      Zsig(0,i) = sqrt(px*px+py*py);
      Zsig(1,i) = atan2(py, px);
      Zsig(2,i) = (px*cos(yaw) + py*sin(yaw))*v/Zsig(0,i);
  }
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i<n_sig_; i++)
  {
      z_pred += weights_(i)*Zsig.col(i);
  }

  //calculate measurement covariance matrix S
  S.fill(0.0);
  VectorXd temp = VectorXd(n_z);
  for (int i=0; i<n_sig_; i++)
  {
      temp = Zsig.col(i) - z_pred;
      S += weights_(i)*temp*temp.transpose();
  }
  S += R_radar_; 

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i=0; i<n_sig_; i++)
  {
      VectorXd xDiff = Xsig_pred_.col(i) - x_;
      while (xDiff(3)> M_PI) xDiff(3)-=2.*M_PI;
      while (xDiff(3)< -M_PI) xDiff(3)+=2.*M_PI;

      VectorXd zDiff = Zsig.col(i) - z_pred;
      while (zDiff(1)> M_PI) zDiff(1)-=2.*M_PI;
      while (zDiff(1)< -M_PI) zDiff(1)+=2.*M_PI;

      Tc += weights_(i)*xDiff*zDiff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();

  // Vector for incoming measurement
  VectorXd z = VectorXd(n_z);
  VectorXd zDiff = VectorXd(n_z);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
  zDiff = z-z_pred;
  while (zDiff(1)> M_PI) zDiff(1)-=2.*M_PI;
  while (zDiff(1)< -M_PI) zDiff(1)+=2.*M_PI;
  
  // Update state mean and covariance
  x_ += K*zDiff;
  P_ -= K*S*K.transpose();

  // Calculate NIS
  NIS_radar_ = zDiff.transpose() * S.inverse() * zDiff;
}
