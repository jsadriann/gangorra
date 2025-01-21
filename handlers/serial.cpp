#include "serial.h"

void uart_send_data(const char* data) {
    while (*data != '\0') {
        uart_putc(UART_ID, *data);  
        data++;                     
    }
}

char uart_receive_data() {
    while (!uart_is_readable(UART_ID)) {
        tight_loop_contents(); 
    }
    return uart_getc(UART_ID);
}


void vTaskSendSerialData(void *pvParameters) {
    const char* message = "Enviando dados pela UART...\n";
    while (1) {
        uart_send_data(message);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Tarefa para receber dados pela UART
void vTaskReceiveSerialData(void *pvParameters) {
    while (1) {
        char received = uart_receive_data();
        uart_putc(UART_ID, received);         
        vTaskDelay(pdMS_TO_TICKS(100));      
    }
}