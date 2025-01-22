#include "serial.h"
std::string uart_receive_data() {
    std::string received_data;
    while (true) {
        if (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            if (c == '\n') {
                break; // Fim da linha
            }
            received_data += c;
        }
    }
    return received_data;
}

void vUartTask(void* pvParameters) {
    QueueHandle_t uart_queue = (QueueHandle_t)pvParameters;
    Message data{
        "Request", "Manual", "getAngle",
        12345, 67890, 1.23f, 0.45f, 0.67f, 12.34f,
        {100, 200, 300}
    };
     std::string json_data = serialize(data);

    while (true) {
        //std::string received_json = uart_receive_data();
        if (xQueueSend(uart_queue, &json_data, portMAX_DELAY) != pdPASS) {
            printf("Erro ao colocar dados na fila\n");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));  // Atraso para evitar ocupação excessiva da CPU
    }
}

void vProcessTask(void* pvParameters) {
    QueueHandle_t uart_queue = (QueueHandle_t)pvParameters;
    std::string json_data;
    while (true) {
        if (xQueueReceive(uart_queue, &json_data, portMAX_DELAY) == pdPASS) {
            Message message = deserialize(json_data);
            // Processa a mensagem (por exemplo, exibir o tipo de mensagem)
            printf("Mensagem recebida: %s\n", json_data.c_str());
            printf("MessageType: %s\n", message.messageType.c_str());
            // Faça algo com o objeto message aqui
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
