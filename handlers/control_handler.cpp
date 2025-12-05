#include "control_handler.h"

// float kp=3.55;
// float ki=0.005;
// float kd=2.05;

// void potentiometerTask(void *pvParameters) {
// 	adc_init();
// 	adc_gpio_init(26);
// 	adc_select_input(0);

// 	QueueHandle_t xMotor = (QueueHandle_t)pvParameters;
// 	while (true) {
// 		// Lê o valor do ADC (0-4095)
// 		uint16_t potValue = adc_read();

// 		// Normaliza para a faixa de 30%-70%
// 		int normalizedValue = (potValue * (MAX_DUTY - MIN_DUTY) / 4095) + MIN_DUTY;

// 		// Envia o valor para a fila
// 		if (xQueueSend(xMotor, &normalizedValue, portMAX_DELAY) != pdPASS) {
// 			printf("Erro ao enviar valor para a fila!\n");
// 		}
// 		printf("Pot value: %d \n", normalizedValue);
// 		// Pequeno atraso para suavizar leituras
// 		vTaskDelay(pdMS_TO_TICKS(50));
// 	}
// }

// void vPIDParametersTask(void *pvParameters) {
// 	PIDParams_t* params = static_cast<PIDParams_t *>(pvParameters);

// 	float time = 0, elapsedTime = 0, timePrev = 0;

// 	float previous_error = 0, error = 0;
// 	float pid_p = 0, pid_i = 0, pid_d = 0;

// 	// Constantes do PID
// 	MpuMailbox_t mailbox;

// 	while(true) {
// 		if(xQueueReceive(params->mpuMailbox, &mailbox, portMAX_DELAY) != pdPASS) {
// 			printf("Don't possible get data from mailbox mpu queue");
// 			continue;
// 		}

// 		timePrev = time;
// 		time = mailbox.xTimeStamp;
// 		elapsedTime = time - timePrev;

// 		error = mailbox.angle - desired_angle;

// 		pid_p = kp * error;
// 		if (-3 < error && error < 3) {
// 			pid_i += ki * error;
// 		}
// 		pid_d = kd * ((error - previous_error) / elapsedTime);

// 		float PID = pid_p + pid_i + pid_d;

// 		printf("P: %.2f I: %.2f D: %.2f \n", pid_p, pid_i, pid_d);
// 		// Limitar valores do PID
// 		if (PID < -1000) PID = -1000;
// 		if (PID > 1000) PID = 1000;

// 		printf("PID: %.2f\n", PID);

// 		float pwmLeft  = fminf(fmaxf(throttle + PID, 1300), 2000);
// 		float pwmRight = fminf(fmaxf(throttle - PID, 1300), 2000);

// 		//Normalize duty cycle to percentual form before send for queue
// 		const int dutyLeft = (pwmLeft - 1000) / 10.0;
// 		const int dutyRight = (pwmRight - 1000) / 10.0;

// 		if(xQueueSend(params->leftMotorQueue, &dutyLeft, portMAX_DELAY) == pdPASS &&
// 			xQueueSend(params->rightMotorQueue, &dutyRight, portMAX_DELAY) == pdPASS) {
// 			printf("Motors updated with %d left and %d right\n", dutyLeft, dutyRight);
// 		}else {
// 			printf("Fail on send data for motors queue\n");
// 		}
// 		previous_error = error;

// 		vTaskDelay(pdMS_TO_TICKS(1000));
// 	}
// }

// typedef struct {
//     QueueHandle_t mpuMailbox;
//     QueueHandle_t leftMotorQueue;
//     QueueHandle_t rightMotorQueue;
// } PIDParams_t;

// typedef struct {
//     float angle;
//     float xTimeStamp;
// } MpuMailbox_t;


// Declaração dos ganhos do PID - Ajuste conforme necessário
float desired_angle =0.0, current_angle= 0.0;
int duty_l = 0, duty_r = 0;
const float kp = 0.65;   // Ganho Proporcional
const float ki = 0.006;  // Ganho Integral
const float kd = 160;   // Ganho Derivativo
float accel_x=0, accel_y=0,accel_z=0;


void vPIDParametersTask(void *pvParameters) {
    PIDParams_t* params = static_cast<PIDParams_t *>(pvParameters);

    TickType_t time = 0, elapsedTime = 0, timePrev = 0;
    float previous_error = 0, error = 0;
    float pid_p = 0, pid_i = 0, pid_d = 0;

    const float pid_i_max = 17.0;  
    const float pid_i_min = -15.0;
    const float pid_d_max = 20;
    const float pid_d_min = -13;

    MpuMailbox_t mailbox;

    while(true) {
        if(xQueueReceive(params->mpuMailbox, &mailbox, portMAX_DELAY) != pdPASS) {
            printf("Erro ao receber dados do MPU\n");
            continue;
        }
        accel_x= static_cast<float>(mailbox.mpu.accelData.accel_x) / (float)8192.0;
        accel_y= static_cast<float>(mailbox.mpu.accelData.accel_y) / (float)8192.0;
        accel_z= static_cast<float>(mailbox.mpu.accelData.accel_z) / (float)8192.0;
        time = mailbox.xTimeStamp;
        elapsedTime = time - timePrev;
		timePrev = time;

        // if (elapsedTime <= 0) elapsedTime = 0.01f;  // Evita divisão por zero

        // Inverte o sinal do ângulo imediatamente ao receber
		current_angle = mailbox.angle;
        //mailbox.angle = -mailbox.angle;

        // Calcula o erro normalmente
        error = desired_angle - current_angle;
		if((error <= 0.8) && (error >= (-0.8))) error = 0.0;

        // Termos do PID
        pid_p = kp * error;
        pid_i += ki * error * elapsedTime;
		if(error==0) pid_i = 0;
        // Aplicar limite ao termo integral para evitar "windup"
        if (pid_i > pid_i_max) pid_i = pid_i_max;
        if (pid_i < pid_i_min) pid_i = pid_i_min;

        // Cálculo do termo derivativo
        pid_d = kd * ((error - previous_error) / elapsedTime);

        // Aplicar limite ao termo derivativo para evitar resposta excessiva
        if (pid_d > pid_d_max) pid_d = pid_d_max;
        if (pid_d < pid_d_min) pid_d = pid_d_min;

        // Soma do PID - limitado ao intervalo [-16, 16]
        float PID = pid_p + pid_i + pid_d;
        if (PID > 25) PID = 25;
        if (PID < -20) PID = -20;

        // O duty cycle do motor esquerdo varia de 16% a 48% (centro = 32%)
        const int dutyNeutral = 39;
        const int dutyLeft = dutyNeutral + static_cast<int>(PID);
        const int dutyRight = 44;  // Motor direito fixo

        // Garante que o duty cycle fique dentro do intervalo permitido
        int dutyLeft_clamped = dutyLeft;
        if (dutyLeft_clamped > 100) dutyLeft_clamped = 100;
        if (dutyLeft_clamped < 0) dutyLeft_clamped = 0;
        duty_l = dutyLeft_clamped;
        duty_r = dutyRight;

        // Envia comandos para os motores
        if(xQueueSend(params->leftMotorQueue, &dutyLeft_clamped, portMAX_DELAY) == pdPASS &&
           xQueueSend(params->rightMotorQueue, &dutyRight, portMAX_DELAY) == pdPASS) {
			// printf("P: %.2f, I: %.2f, D: %.2f, Angle: %.2f ERROR: %.2f time: %d\n", pid_p, pid_i, pid_d, mailbox.angle, error,time);
            // printf("Motores atualizados: Left = %d%% (PID: %.2f), Right = %d%%\n", 
            //         dutyLeft_clamped, PID, dutyRight);
			// printf("angulo desejado: %.2f\n",desired_angle);
        } else {
            printf("Falha ao enviar dados para os motores\n");
        }

        previous_error = error;
        vTaskDelay(pdMS_TO_TICKS(61));
    }
}


// const float desired_angle = 0.0;
// const float kp = 0.35;   // Ganho Proporcional
// const float ki = 0.001;  // Ganho Integral
// const float kd = 130;   // Ganho Derivativo


// TickType_t time = 0, elapsedTime = 0, timePrev = 0;
// float previous_error = 0, error = 0;
// float pid_p = 0, pid_i = 0, pid_d = 0;

// const float pid_i_max = 8.0;  // Limite seguro para o termo I
// const float pid_i_min = -8.0;
// const float pid_d_max = 20;  // Limite para o termo D
// const float pid_d_min = -20;