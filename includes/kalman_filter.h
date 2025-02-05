#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include <mpu6050.hpp>
#include <cmath>

class KalmanFilter {
	float theta{};        // Ângulo estimado
	float theta_dot{};    // Velocidade angular estimada
	float P[2][2]{};      // Matriz de covariância
	float Q[2][2]{};      // Covariância do processo
	float R[2]{};         // Covariância das medições

	void initializeKalmanFilter();
	static float normalizeAngle(float angle);

public:
	KalmanFilter();

	void getAngleWithEKF(accel accelData, gyro gyroData, float &angle, float dt);
};

#endif //KALMAN_FILTER_H
