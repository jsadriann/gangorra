#include "motor_handler.h"

void vTaskMotorControl(void *pvParameters) {
	const auto* motor = static_cast<motor_params *>(pvParameters);
	QueueHandle_t mailbox = motor->mailbox;
	Motor* sensor = motor->motor;
	sensor->vInitMotors();

	int dutyCycle;
	while (true) {
		// Aguarda por valores na fila
		if (xQueueReceive(mailbox, &dutyCycle, portMAX_DELAY) == pdPASS) {
			// Ajusta a velocidade do motor usando a classe Motor
			printf("Velocidade ajustada: %d%%\n", dutyCycle);
			sensor->setSpeed(dutyCycle);
		}else {
			printf("Nao foi possível obter dados da queue, mantendo a velocidade");
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}
