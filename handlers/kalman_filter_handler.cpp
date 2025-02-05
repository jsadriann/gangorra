//
// Created by guilherme on 04/02/25.
//

#include "kalman_filter_handler.h"


void vTaskKalmanMPU6050Angle(void *pvParameters) {
	const auto* mpu = static_cast<mpu_params *>(pvParameters);
	QueueHandle_t mailbox = mpu->mailbox;
	MpuMailbox_t mailboxData;

	KalmanFilter kalman_filter;

	float angle_filtered; // Ângulo filtrado
	float angle_raw;      // Ângulo bruto do acelerômetro
	const float dt = 0.05; // Intervalo de tempo em segundos (50 ms)

	while(true) {
		if(xQueueReceive(mailbox, &mailboxData, portMAX_DELAY) == pdPASS) {
			mpu6050::getAngle(angle_raw, mailboxData.mpu.accelData);

			kalman_filter.getAngleWithEKF(
					mailboxData.mpu.accelData,
					mailboxData.mpu.gyroData,
					angle_filtered,
					dt
				);
			printf("Raw Angle: %.2f | Filtered Angle: %.2f\n", angle_raw, angle_filtered);
			printf("TimeStamp: %d\n\n", mailboxData.xTimeStamp);
		} else {
			printf("Consumer: Falha ao receber da mailbox\n");
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}
