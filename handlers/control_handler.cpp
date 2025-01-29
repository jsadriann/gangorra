#include "control_handler.h"

void potentiometerTask(void *pvParameters) {
	adc_init();
	adc_gpio_init(26);
	adc_select_input(0);

	QueueHandle_t xMotor = (QueueHandle_t)pvParameters;
	while (true) {
		// Lê o valor do ADC (0-4095)
		uint16_t potValue = adc_read();

		// Normaliza para a faixa de 30%-70%
		int normalizedValue = (potValue * (MAX_DUTY - MIN_DUTY) / 4095) + MIN_DUTY;

		// Envia o valor para a fila
		if (xQueueSend(xMotor, &normalizedValue, portMAX_DELAY) != pdPASS) {
			printf("Erro ao enviar valor para a fila!\n");
		}
		printf("Pot value: %d \n", normalizedValue);
		// Pequeno atraso para suavizar leituras
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}