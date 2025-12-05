/*
 * =====================================================================================
 *
 *       Filename:  BluetoothManager.cpp
 *
 *    Description:  -
 *
 *        Version:  1.0
 *        Created:  17/11/2024 18:37:36
 *       Revision:  none
 *       Compiler:  -
 *
 *         Author:  Isaac Vinicius, isaacvinicius2121@alu.ufc.br
 *   Organization:  UFC-Quixadá
 *
 * =====================================================================================
*/

#include "BluetoothManager.hpp"
#include "JsonManager.hpp"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Arduino.h>
#include "GlobalVariables.hpp"
#include "UartManager.hpp"

// Variáveis globais
BLECharacteristic *characteristicTX; // Característica para envio
BLECharacteristic *characteristicRX; // Característica para recepção
bool deviceConnected = false;        // Controle de dispositivo conectado
const int LED = 2;                   // Pino do LED


class CharacteristicCallbacks : public BLECharacteristicCallbacks {
  std::string buffer; // Buffer para armazenar dados recebidos
  int braceCount = 0; // Contador de aberturas e fechamentos de chaves

  void onWrite(BLECharacteristic *characteristic) {
    std::string rxValue = characteristic->getValue();
    if (!rxValue.empty()) {
      for (char c : rxValue) {
        buffer += c;
        if (c == '{') {
          braceCount++;
        } else if (c == '}') {
          braceCount--;
        }
      }

      if (braceCount == 0 && !buffer.empty()) {
        DynamicJsonDocument doc(256);
        DeserializationError error = deserializeJson(doc, buffer);

        if (error) {
          Serial.print("Erro ao interpretar JSON: ");
          Serial.println(error.c_str());
        } else {
          // Extrai os campos do JSON
          std::string messageType = doc["messageType"] | "";
          std::string modetype = doc["modetype"] | "";
          std::string conftype = doc["conftype"] | "";

          if (messageType == "Reply" && modetype == "manual") {
            if (conftype == "angulo") {
              // Atualiza o ângulo
              state.currentCommand = "1";//manual_algulo
              state.angle = doc["angle"] | state.cangle;
              state.cangle = doc["angle"] | state.cangle;
              /* Enviando dados para a raspberry via uart */
              sendDataPackage(state.currentCommand, 1);
              Serial.printf("Modo manual com Ângulo ativado: %.2f\n", state.angle);


            } else if (conftype == "pid") {
              // Atualiza os valores PID
              state.currentCommand = "2";//manual_pid
              state.Kp = doc["Kp"] | state.Kp;
              state.Ki = doc["Ki"] | state.Ki;
              state.Kd = doc["Kd"] | state.Kd;
              /* Enviando dados para a raspberry via uart */
              sendDataPackage(state.currentCommand, 1);
              Serial.printf("Modo manual com PID ativado - Kp:%.2f,Ki:%.2f,Kd:%.2f\n", state.Kp, state.Ki, state.Kd);

            }
          } else if (messageType == "Reply" && modetype == "auto" && conftype == "none") {
            // Alterna para o modo automático
            state.currentCommand = "3";//auto
            /*enviando dados para a raspberry via uart */
            sendDataPackage(state.currentCommand, 1);
            Serial.println("Modo automático ativado.");
          } else {
            Serial.println("Erro: messageType ou modetype inválido!");
          }
        }

        // Limpa o buffer e o contador após processar o JSON
        buffer.clear();
        braceCount = 0;
        //state.updated = true;
      }
    }
  }
};

/* Callback para gerenciar estado de conexão do dispositivo BLE */
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    //state.currentCommand = "auto"; // Ativa o envio automático ao conectar
    Serial.println("Dispositivo conectado!");
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
    Serial.println("Dispositivo desconectado!");
  }
};

/* Função dedicada para configurar o Bluetooth */
void setupBluetooth() {
  Serial.println("Inicializando Bluetooth...");

  pinMode(LED, OUTPUT);

  /* Criando um novo dispositivo BLE com o nome do servidor BLE */
  BLEDevice::init(bleServerName);


  /* Definindo o dispositivo como servidor BLE */
  BLEServer *server = BLEDevice::createServer();

 
  server->setCallbacks(new ServerCallbacks());


  /* Criando serviço no dispositivo BLE */
  BLEService *service = server->createService(SERVICE_UUID);

  /* Criando a característica para envio de dados (TX) */
  characteristicTX = service->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY);
  characteristicTX->addDescriptor(new BLE2902()); // Habilita notificações

  /* Criando a característica para recepção de dados (RX) */
  characteristicRX = service->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE);
  characteristicRX->setCallbacks(new CharacteristicCallbacks());

  /* Inicializando o serviço */
  service->start();
}

/* Função dedicada para iniciar a propagação (advertising) */
void startAdvertising() {
  BLEDevice::getAdvertising()->start();
  Serial.println("Aguardando conexão de dispositivos...");
}

/* Função para enviar uma string via BLE */
void sendString(const char *message) {
  if (deviceConnected) {
    String data = String(message);
   int chunkSize = 20;

    for (int i = 0; i < data.length(); i += chunkSize) {
      String chunk = data.substring(i, min(i + chunkSize,(int) data.length()));
      characteristicTX->setValue(chunk.c_str());
      characteristicTX->notify(); // Envia o pacote
      delay(10); // Pequeno delay para evitar congestionamento
    }
    Serial.print("Mensagem Enviada ");
    Serial.println(message);
  } else {
    Serial.println("Nenhum dispositivo conectado. Não foi possível enviar a string.");
  }
}

/*****************************END OF FILE**************************************/