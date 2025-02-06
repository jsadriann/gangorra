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

void vPIDParametersTask(void *pvParameters) {
	PIDParams_t* params = static_cast<PIDParams_t *>(pvParameters);

	float time = 0, elapsedTime = 0, timePrev = 0;

	float previous_error = 0, error = 0;
	float pid_p = 0, pid_i = 0, pid_d = 0;

	// Constantes do PID
	MpuMailbox_t mailbox;

	while(true) {
		if(xQueueReceive(params->mpuMailbox, &mailbox, portMAX_DELAY) != pdPASS) {
			printf("Don't possible get data from mailbox mpu queue");
			continue;
		}

		timePrev = time;
		time = mailbox.xTimeStamp;
		elapsedTime = time - timePrev;

		error = mailbox.angle - desired_angle;

		pid_p = kp * error;
		if (-3 < error && error < 3) {
			pid_i += ki * error;
		}
		pid_d = kd * ((error - previous_error) / elapsedTime);

		float PID = pid_p + pid_i + pid_d;

		printf("P: %.2f I: %.2f D: %.2f \n", pid_p, pid_i, pid_d);
		// Limitar valores do PID
		if (PID < -1000) PID = -1000;
		if (PID > 1000) PID = 1000;

		printf("PID: %.2f\n", PID);

		float pwmLeft  = fminf(fmaxf(throttle + PID, 1300), 2000);
		float pwmRight = fminf(fmaxf(throttle - PID, 1300), 2000);

		//Normalize duty cycle to percentual form before send for queue
		const int dutyLeft = (pwmLeft - 1000) / 10.0;
		const int dutyRight = (pwmRight - 1000) / 10.0;

		if(xQueueSend(params->leftMotorQueue, &dutyLeft, portMAX_DELAY) == pdPASS &&
			xQueueSend(params->rightMotorQueue, &dutyRight, portMAX_DELAY) == pdPASS) {
			printf("Motors updated with %d left and %d right\n", dutyLeft, dutyRight);
		}else {
			printf("Fail on send data for motors queue\n");
		}
		previous_error = error;

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}