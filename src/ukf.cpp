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

	// state dimension
	n_x_ = 5;
	// augmented dimension
	n_aug_ = 7;
  // radar dimension
  n_z_ = 3;
	// spreading parameter
	lambda_ = 3 - n_aug_;

	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	// initial state vector
	x_ = VectorXd(n_x_);

	// initial covariance matrix
	P_ = MatrixXd(n_x_, n_x_);
	P_ << 1, 0, 0, 0, 0,
	    0, 1, 0, 0, 0,
	    0, 0, 10, 0, 0,
	    0, 0, 0, 1, 0,
	    0, 0, 0, 0, 1;

	// augmented mean vector and state covariance
	x_aug_ = VectorXd(n_aug_);
	P_aug_ = MatrixXd(n_aug_, n_aug_);

	// Sigma points
	Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
	Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

	Zsig_ = MatrixXd(3, 2 * n_aug_ + 1);

	// Weights for sigma points
	weights_ = VectorXd(2 * n_aug_ + 1);
	weights_(0) = lambda_ / (lambda_ + n_aug_);
	for (int i = 1; i < weights_.size(); i++) {
		weights_(i) = 1 / (2 * (lambda_ + n_aug_));
	}

	H_ = MatrixXd(2, 5);
	H_ << 1, 0, 0, 0, 0,
	    0, 1, 0, 0, 0;

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 2.55;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 1.5;

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

	is_initialized_ = false;
	previous_timestamp_ = 0;

	R_laser_ = MatrixXd(2, 2);
	R_laser_ << std_laspx_ * std_laspx_, 0,
	    0, std_laspy_ * std_laspy_;

	R_radar_ = MatrixXd(3, 3);
	R_radar_ << std_radr_*std_radr_, 0, 0,
	    0, std_radphi_*std_radphi_, 0,
	    0, 0,std_radrd_*std_radrd_;

  z_pred_ = VectorXd(n_z_);
  S_ = MatrixXd(n_z_, n_z_);
}

UKF::~UKF() {
}

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
	if (!is_initialized_) {
		// first measurement
		x_ << 0, 0, 0, 0, 0;

		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			   Convert radar from polar to cartesian coordinates and initialize state.
			 */
			float rho = meas_package.raw_measurements_(0);
			float theta = meas_package.raw_measurements_(1);

			x_(0) = rho * cos(theta);
			x_(1) = rho * sin(theta);

		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			/**
			   Initialize state.
			 */
			x_(0) = meas_package.raw_measurements_(0);
			x_(1) = meas_package.raw_measurements_(1);
		}

		cout << "Initial position: " << x_ << endl;

		// done initializing, no need to predict or update
		previous_timestamp_ = meas_package.timestamp_;
		is_initialized_ = true;
		return;
	}

  // Calculate delta t
	double dt = (meas_package.timestamp_ - previous_timestamp_);
	dt /= 1000000.0;     //dt - expressed in seconds
	previous_timestamp_ = meas_package.timestamp_;

	// Launching Kalman filter predict step
	Prediction(dt);

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		if (use_radar_) UpdateRadar(meas_package);
	} else {
		// Laser updates
		if (use_laser_) UpdateLidar(meas_package);
	}

	// print the output
	cout << "x_ = " << x_ << endl;
	cout << "P_ = " << P_ << endl;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  // 3 prediction steps: creating sigma points, predicting their value after dt
  // and calculating new mean/covariance based on those predicted values
	CreateSigmaPoints();
	PredictSigmaPoints(delta_t);
	PredictMeanAndCovariance();
}

void UKF::CreateSigmaPoints() {
	//create augmented mean state
	x_aug_.head(n_x_) << x_;
	x_aug_.tail(2) << 0, 0;

	//create augmented covariance matrix
	P_aug_.setZero();
	P_aug_.topLeftCorner(n_x_, n_x_) = P_;
	P_aug_.bottomRightCorner(2, 2) << std_a_ * std_a_, 0,
	    0, std_yawdd_ * std_yawdd_;

	//create square root matrix
	MatrixXd A = P_aug_.llt().matrixL();
	A *= sqrt(lambda_ + n_aug_);

	// create sigma points
	Xsig_aug_.col(0) = x_aug_;
	for (int i = 0; i < A.cols(); i++) {
		Xsig_aug_.col(i + 1) = x_aug_ + A.col(i);
		Xsig_aug_.col(i + n_aug_ + 1) = x_aug_ - A.col(i);
	}
}

void UKF::PredictSigmaPoints(double delta_t) {
	// predict sigma points
	for (int i = 0; i < Xsig_aug_.cols(); i++) {
		VectorXd x_k = Xsig_aug_.col(i).head(n_x_);
		float p_x = Xsig_aug_(0, i);
		float p_y = Xsig_aug_(1, i);
		float v = Xsig_aug_(2, i);
		float yaw = Xsig_aug_(3, i);
		float yaw_dot = Xsig_aug_(4, i);
		float v_a = Xsig_aug_(5, i);
		float v_yaw_dot = Xsig_aug_(6, i);

		VectorXd adj = VectorXd(n_x_);
		VectorXd noise = VectorXd(n_x_);

		if (fabs(yaw_dot) > 0.001) {
			adj << (sin(yaw + yaw_dot * delta_t) - sin(yaw)) * v / yaw_dot,
			(-cos(yaw + yaw_dot * delta_t) + cos(yaw)) * v / yaw_dot,
			    0,
			    yaw_dot * delta_t,
			    0;
		} else {
			adj << v * cos(yaw) * delta_t,
			    v * sin(yaw) * delta_t,
			    0,
			    yaw_dot * delta_t,
			    0;
		}
		float delta_squared = delta_t * delta_t;
		noise << delta_squared * cos(yaw) * v_a / 2,
		    delta_squared * sin(yaw) * v_a / 2,
		    delta_t * v_a,
		    delta_squared * v_yaw_dot / 2,
		    delta_t * v_yaw_dot;

		Xsig_pred_.col(i) = x_k + adj + noise;
	}
}

void UKF::PredictMeanAndCovariance() {
	//predict state mean
	x_.setZero();
	for (int i = 0; i < weights_.size(); i++) {
		x_ += weights_(i) * Xsig_pred_.col(i);
	}

	//predict state covariance matrix
	P_.setZero();
	for (int i = 0; i < weights_.size(); i++) {
		MatrixXd diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (diff(3) > M_PI) diff(3) -= 2. * M_PI;
		while (diff(3) < -M_PI) diff(3) += 2. * M_PI;
		diff *= diff.transpose();
		P_ += weights_(i) * diff;
	}
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  // Lidar measurements are transformed linearly,
  // so we use simple Kalman Filter here

	VectorXd z = meas_package.raw_measurements_.head(2);
	VectorXd y = z - H_ * x_;

	MatrixXd Ht = H_.transpose();

	MatrixXd S = H_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ += K * y;
	P_ -= K * H_ * P_;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  // Transform predictions into radar measurment space and calculate
  // measurment covariance matrix
  TransformPrediction();
  // Use difference between predicted and actual measurments and update
  // state and covariance
  UpdatePrediction(meas_package);
}

void UKF::TransformPrediction() {
	// Transforming into measurment space
	z_pred_.setZero();
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {     //2n+1 simga points

		double px = Xsig_pred_(0,i);
		double py = Xsig_pred_(1,i);
		double v  = Xsig_pred_(2,i);
		double yaw = Xsig_pred_(3,i);

		float rho = sqrt(px * px + py * py);
		float theta = (px == 0 || py == 0) ? 0 : atan2(py, px);
		float rho_dot = (rho == 0) ? 0 : ((px * cos(yaw) * v + py * sin(yaw) * v) / rho);
		Zsig_.col(i) << rho, theta, rho_dot;
		//mean predicted measurement
		z_pred_ += weights_(i) * Zsig_.col(i);
	}

	// measurement covariance matrix S
	S_.setZero();
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {     //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig_.col(i) - z_pred_;

		//angle normalization
		while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
		while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

		S_ += weights_(i) * z_diff * z_diff.transpose();
	}

	//add measurement noise covariance matrix
	S_ += R_radar_;
}

void UKF::UpdatePrediction(MeasurementPackage meas_package) {
	//calculate cross correlation matrix
	MatrixXd Tc = MatrixXd(n_x_, n_z_);
	Tc.setZero();
	for (int i = 0; i < weights_.size(); i++) {
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		VectorXd z_diff = Zsig_.col(i) - z_pred_;

		// Normalize angles #3 in X and #1 in Z
		while (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;
		while (x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;

		while (z_diff(1) > M_PI) z_diff(1) -= 2 * M_PI;
		while (z_diff(1) < -M_PI) z_diff(1) += 2 * M_PI;

		Tc += weights_(i) * x_diff * z_diff.transpose();
	}

	//calculate Kalman gain K;
	MatrixXd K = Tc * S_.inverse();

	//update state mean and covariance matrix
	x_ += K * (meas_package.raw_measurements_.head(3) - z_pred_);
	P_ -= K * S_ * K.transpose();

}
