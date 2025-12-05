/*
 * =====================================================================================
 *
 *       Filename:  UartManager.cpp
 *
 *    Description:  -
 *
 *        Version:  1.0
 *        Created:  03/01/2025 10:15:45
 *       Revision:  none
 *       Compiler:  -
 *
 *         Author:  Isaac Vinicius, isaacvinicius2121@alu.ufc.br
 *   Organization:  UFC-Quixadá
 *
 * =====================================================================================
 */

#include <Arduino.h>
#include "UartManager.hpp"
#include "GlobalVariables.hpp"

// Inicializa a UART com os pinos e a velocidade desejada
void UARTManager_init(long baudRate, int rxPin, int txPin) {
  Serial1.begin(baudRate, SERIAL_8N1, rxPin, txPin);
}

// Funções de processamento para cada comando
void getKp(void) {
  Serial.println("Kp: " + String(state.Kp));
}

void getKi(void) {
  Serial.println("Ki: " + String(state.Ki));
}

void getKd(void)
{
  Serial.println("Kd: " + String(state.Kd));
}

void getTof_r(void) {
  Serial.println("TOF Direito: " + String(state.tof_r));
}

void getTof_l(void) {
  Serial.println("TOF Esquerdo: " + String(state.tof_l));
}

void getAccx(void) {
  Serial.println("Acelerômetro X: " + String(state.accx));
}

void getAccy(void) {
  Serial.println("Acelerômetro Y: " + String(state.accy));
}

void getAccz(void) {
  Serial.println("Acelerômetro Z: " + String(state.accz));
}

void getAngle(void) {
  Serial.println("Ângulo: " + String(state.angle));
}

void getCurrent(void) {
  Serial.println("Corrente: " + String(state.current));
}

void getAll(void) {
  Serial.println("TOF Direito: " + String(state.tof_r));
  Serial.println("TOF Esquerdo: " + String(state.tof_l));
  Serial.println("Acelerômetro X: " + String(state.accx));
  Serial.println("Acelerômetro Y: " + String(state.accy));
  Serial.println("Acelerômetro Z: " + String(state.accz));
  Serial.println("Ângulo: " + String(state.angle));
  Serial.println("Corrente: " + String(state.current));
}

// Estrutura para mapeamento de comandos
typedef struct {
  String command;
  void (*processFunction)();
} CommandMapping;

// Tabela de mapeamento de comandos
CommandMapping commandTable[] = {

  {"0", getKp},
  {"A", getKi},
  {"2", getKd},
  {"3", getTof_r},
  {"4", getTof_l},
  {"5", getAccx},
  {"6", getAccy},
  {"7", getAccz},
  {"8", getAngle},
  {"9", getCurrent},
  {"1", getAll},
};

void execute_command(const String &command) {
  for (const auto &entry : commandTable) {
    if (entry.command == command) {
      entry.processFunction();
      return;
    }
  }
  Serial.println("Erro: Comando não reconhecido - " + command);
}

String receivedPackage = ""; // Armazena o pacote completo

void receive_package(void) {
  while (Serial1.available()){ // Se há bytes disponíveis na UART
    char c = Serial1.read(); // Lê um caractere
    if (c == '#') {
      receivedPackage = "#"; // Inicia a recepção do pacote
    }
    else if (c == '/') {
      receivedPackage += "/"; // Finaliza o pacote
      Serial.println("Pacote completo recebido: " + receivedPackage);
      process_package(receivedPackage); // Processa o pacote recebido
      receivedPackage = "";             // Limpa para o próximo pacote
    } else{
      receivedPackage += c; // Continua armazenando os caracteres
    }
  }
}

void process_package(String package) {
  if (package.startsWith("#") && package.endsWith("/")){  // Verifica se é um pacote válido
    package.remove(0, 1);                    // Remove o '#'
    package.remove(package.length() - 1, 1); // Remove o '/'

    int cmdIndex = package.indexOf('$');
    int typeIndex = package.indexOf('@');
    int dataStart = package.indexOf(':');
    int dataEnd = package.lastIndexOf(':');

    if (cmdIndex == -1 || typeIndex == -1 || dataStart == -1 || dataEnd == -1) {
      Serial.println("Erro: Pacote mal formatado");
      return;
    }

    // Extração das partes
    String command = package.substring(0, cmdIndex);          // Comando
    String type = package.substring(cmdIndex + 1, typeIndex); // Tipo de mensagem (response ou request)
    String data = package.substring(dataStart + 1, dataEnd);  // Dados dos sensores

    Serial.println("Comando: " + command);
    Serial.print("Tipo de Mensagem: ");
    Serial.println(state.messageType == 0 ? "Request (0)" : "Response (1)");
    Serial.println("Dados: " + data);

    // Separando os valores dos sensores
    String sensorValues[10]; // Array para armazenar os 7 valores
    int index = 0;
    int pos = 0;

    while ((pos = data.indexOf(',')) != -1 && index < 10) {
      sensorValues[index] = data.substring(0, pos);
      data.remove(0, pos + 1);
      index++;
    }

    if (index < 9) { // Deve ser 9 porque o último valor é adicionado fora do loop
      Serial.println("Erro: Pacote incompleto. Esperados 10 valores, recebidos apenas " + String(index + 1));
      return;
    }
    sensorValues[index] = data; // Último valor

    // Atribuir os valores às variáveis corretas
    state.Kp = sensorValues[0].toFloat();
    state.Ki = sensorValues[1].toFloat();
    state.Kd = sensorValues[2].toFloat();
    state.tof_r = sensorValues[3].toFloat();
    state.tof_l = sensorValues[4].toFloat();
    state.accx = sensorValues[5].toFloat();
    state.accy = sensorValues[6].toFloat();
    state.accz = sensorValues[7].toFloat();
    state.angle = sensorValues[8].toFloat();
    state.current = sensorValues[9].toFloat();

    execute_command(command);
  } else {
    Serial.println("Erro: Pacote inválido");
  }
}

// Função para formatar e transmitir o pacote
void sendDataPackage(String command, int type) {
  // Formata o pacote de dados no padrão especificado
  String package = "#";
  package += command;
  package += "$";
  package += String(type);
  package += "@:";

  if (command == "2") {//pid
    package += String(state.Kp, 2) + ",";
    package += String(state.Ki, 2) + ",";
    package += String(state.Kd, 2);
    package += String(state.angle, 2);
  } else if (command == "1") {//angulo
    package += String(state.Kp, 2) + ",";
    package += String(state.Ki, 2) + ",";
    package += String(state.Kd, 2) + ",";
    package += String(state.angle, 2);
  } else if (command == "3") {//auto
    package += "none";
  }

  package += ":/";

  Serial1.println(package);
  Serial.println("Enviado para Raspberry Pi via UART: " + package);
}
