#include "motor_handler.h"

void vTaskMotorControl(void *pvParameters) {
	const auto* motor = static_cast<motor_params *>(pvParameters);
	QueueHandle_t mailbox = motor->mailbox;
	Motor* sensor = motor->motor;

	int dutyCycle;
	while (true) {
		if (xQueueReceive(mailbox, &dutyCycle, portMAX_DELAY) == pdPASS) {
			printf("%s velocity adjusted in %d%%\n", pcQueueGetName(motor->mailbox), dutyCycle);
			sensor->setSpeed(dutyCycle);
		}else {
			printf("It was not possible to obtain data from Queue, maintaining the speed");
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}
