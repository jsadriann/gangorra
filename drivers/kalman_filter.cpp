#include "kalman_filter.h"

void KalmanFilter::initializeKalmanFilter() {
	this->theta = 0;
	this->theta_dot = 0;
	this->P[0][0] = 1;	 this->P[0][1] = 0;
	this->P[1][0] = 0;	 this->P[1][1] = 1;
	this->Q[0][0] = 0.1; this->Q[0][1] = 0;
	this->Q[1][0] = 0;   this->Q[1][1] = 0.03;
	this->R[0] = 0.27;  // Covariância do acelerômetro
	this->R[1] = 0.2;  // Covariância do giroscópio
}

KalmanFilter::KalmanFilter() {
	initializeKalmanFilter();
}

void KalmanFilter::getAngleWithEKF(const accel accelData, const gyro gyroData, float &angle, const float dt) {
	// Conversão para unidades apropriadas
	const float accel_angle = atan2(
															static_cast<float>(accelData.accel_y) / ACCEL_SCALE_FACTOR,
															sqrt(pow(static_cast<float>(accelData.accel_x) / ACCEL_SCALE_FACTOR, 2) +
																	 pow(static_cast<float>(accelData.accel_z) / ACCEL_SCALE_FACTOR, 2))
														) * (180.0 / M_PI);

	const float gyro_rate = static_cast<float>(gyroData.gyro_x) / GYRO_SCALE_FACTOR;

	// --- EKF Prediction ---
	this->theta += dt * this->theta_dot;
	this->theta_dot = gyro_rate;

	// Atualizar covariância do estado
	const float P00_temp = this->P[0][0];
	const float P01_temp = this->P[0][1];

	this->P[0][0] += dt * (this->P[1][0] + P01_temp) + this->Q[0][0]; //como incerteza da velocidade angular contribui para a confiaça do modelo
	this->P[0][1] += dt * this->P[1][1]; //contribui para incerteza do ângulo
	this->P[1][0] += dt * this->P[1][1]; //contribui para incerteza do ângulo
	this->P[1][1] += this->Q[1][1]; //incerteza associada à velocidade angular estimada

	// --- EKF Update ---
	const float y[2] = {accel_angle - this->theta, gyro_rate - this->theta_dot};
	const float S[2] = {this->P[0][0] + this->R[0], this->P[1][1] + this->R[1]};

	const float K[2][2] = {
		{this->P[0][0] / S[0], this->P[0][1] / S[1]},
		{this->P[1][0] / S[0], this->P[1][1] / S[1]}
	};

	this->theta += K[0][0] * y[0] + K[0][1] * y[1];
	this->theta_dot += K[1][0] * y[0] + K[1][1] * y[1];

	// Normalizar o ângulo estimado
	this->theta = normalizeAngle(this->theta);

	// Atualizar matriz de covariância do estado
	this->P[0][0] -= K[0][0] * P00_temp;
	this->P[0][1] -= K[0][0] * P01_temp;
	this->P[1][0] -= K[1][0] * P00_temp;
	this->P[1][1] -= K[1][0] * P01_temp;

	// Retorna o ângulo fundido
	angle = this->theta;
}

float KalmanFilter::normalizeAngle(float angle) {
	while (angle > 180.0) angle -= 360.0;
	while (angle < -180.0) angle += 360.0;
	return angle;
}