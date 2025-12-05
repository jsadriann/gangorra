/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  -
 *
 *        Version:  1.0
 *        Created:  17/11/2024 19:03:18
 *       Revision:  none
 *       Compiler:  -
 *
 *         Author:  Isaac Vinicius, isaacvinicius2121@alu.ufc.br
 *   Organization:  UFC-Quixadá
 *
 * =====================================================================================
*/

#include <Arduino.h>
#include "BluetoothManager.hpp"
#include "driver_lcd_i2c.hpp"
#include "UartManager.hpp"
#include "JsonManager.hpp"
#include "esp_timer.h"

#define RX_PIN 16
#define TX_PIN 17
#define INTERRUPT_PIN_1 18  // GPIO18
#define INTERRUPT_PIN_2 5   // GPIO5
const int LED = 2;  
int count = 0;
int lcdSwitchBtn = 0;
float corrente=0.0, tensao=0.0;
hw_timer_t *timer = NULL;          // Pino do LED
#include "GlobalVariables.hpp"

SystemState state;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 3000;
extern bool deviceConnected;
DriverLCD_I2C lcdDriver(0x27, 20, 4);

void IRAM_ATTR onTimer() {
  count++;
  if(count>3) 
    count = 0;
}
// Função de interrupção para o GPIO18
void IRAM_ATTR next() {
   lcdSwitchBtn++;
   if(lcdSwitchBtn > 3) lcdSwitchBtn = 0;
}

// Função de interrupção para o GPIO5
void IRAM_ATTR previous() {
  lcdSwitchBtn--;
  if(lcdSwitchBtn < 0) lcdSwitchBtn = 3;
}

void setup() {
    // Configura o Timer 0 com prescaler 80 (1 tick = 1µs)
  timer = timerBegin(0, 80, true);
  if (timer == NULL) {
    Serial.println("Erro ao configurar o timer");
    return;  // Se o timer não foi configurado corretamente, pare a execução
  } // Timer 0, Prescaler 80 → 1 MHz (1µs por tick), Contagem Up
  timerAttachInterrupt(timer, &onTimer, true);    // Anexa a função de interrupção
  timerAlarmWrite(timer, 3000000, true);    // Dispara a cada 1.000.000 µs (1 segundo)
  timerAlarmEnable(timer);   // Ativa o alarme do Timer
  Serial.begin(115200);
  /* Inicializando configurações do Bluetooth */
  setupBluetooth();
  // pinMode(INTERRUPT_PIN_1, INPUT_PULLUP);
  // pinMode(INTERRUPT_PIN_2, INPUT_PULLUP);  
  // // Configura interrupções para detectar borda de descida (FALLING)
  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), next, FALLING);
  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), previous, FALLING);

  /* Inicializando configurações da UART */
  UARTManager_init(115200, RX_PIN, TX_PIN); // Inicializa UART

  /* Inicializando a propagação BLE */
  startAdvertising();
  lcdDriver.dashboardLightOn();
  
}


void loop() {
  
  receive_package();
  lcdDriver.dashboardLightOn();
  if (deviceConnected) { // Verifica se há um dispositivo conectado
    if (state.currentCommand == "3") {
      unsigned long currentTime = millis();
      if (currentTime - lastSendTime >= sendInterval) {
        lastSendTime = currentTime;

        // Cria o JSON para envio contínuo no modo automático
        DynamicJsonDocument doc(256);
        doc["messageType"] = "Response";
        doc["modetype"] = "auto";
        doc["conftype"] = "none";
        doc["Kp"] = state.Kp;
        doc["Ki"] = state.Ki;
        doc["Kd"] = state.Kd;
        doc["tof_r"] = state.tof_r;
        doc["tof_l"] = state.tof_l;
        doc["accx"] = state.accx;
        doc["accy"] = state.accy;
        doc["accz"] = state.accz;
        doc["angle"] = (int)state.angle;
        doc["current"] = state.current;

        // Serializa e envia o JSON
        std::string jsonString;
        serializeJson(doc, jsonString);
        sendString(jsonString.c_str());
        Serial.println("Dados enviados automaticamente:");
        Serial.println(jsonString.c_str());
      }
    } else if (state.currentCommand == "1") {
      unsigned long currentTime = millis();
      if (currentTime - lastSendTime >= sendInterval) {
        lastSendTime = currentTime;

        // Cria o JSON para envio contínuo dos parâmetros PID
        DynamicJsonDocument doc(256);
        doc["messageType"] = "Response";
        doc["modetype"] = "manual";
        doc["conftype"] = "conftype_angulo";
        doc["Kp"] = state.Kp;
        doc["Ki"] = state.Ki;
        doc["Kd"] = state.Kd;
        doc["tof_r"] = state.tof_r;
        doc["tof_l"] = state.tof_l;
        doc["accx"] = state.accx;
        doc["accy"] = state.accy;
        doc["accz"] = state.accz;
        doc["angle"] = (int)state.angle;
        doc["current"] = state.current;

        // Serializa e envia o JSON
        std::string jsonString;
        serializeJson(doc, jsonString);
        sendString(jsonString.c_str());

        Serial.println("Enviando parâmetros PID:");
        Serial.println(jsonString.c_str());
      }
    }else if (state.currentCommand == "2") {
      unsigned long currentTime = millis();
      if (currentTime - lastSendTime >= sendInterval) {
        lastSendTime = currentTime;

        // Cria o JSON para enviar os valores PID e o ângulo
        DynamicJsonDocument doc(256);
        doc["messageType"] = "Response";
        doc["modetype"] = "manual";
        doc["conftype"] = "pid";
        doc["Kp"] = state.Kp;
        doc["Ki"] = state.Ki;
        doc["Kd"] = state.Kd;
        doc["tof_r"] = state.tof_r;
        doc["tof_l"] = state.tof_l;
        doc["accx"] = state.accx;
        doc["accy"] = state.accy;
        doc["accz"] = state.accz;
        doc["angle"] = (int)state.angle;
        doc["current"] = state.current;

        // Serializa e envia o JSON
        std::string jsonString;
        serializeJson(doc, jsonString);
        sendString(jsonString.c_str());
        Serial.println("Enviando valores PID e Ângulo no modo manual (PID):");
        Serial.println(jsonString.c_str());
      
      }
    }
  } else {
    // Caso nenhum dispositivo esteja conectado
    if (state.currentCommand != "disconnected") {
      state.currentCommand = "disconnected";
      
      Serial.println("Nenhum dispositivo conectado. Dados não serão enviados.");
    }
  }
  switch (count)
  {
  case 0:
  lcdDriver.postAngle(state.angle,state.cangle);
  lcdDriver.postAccelerometer(state.accx,state.accy,state.accz);
    break;
  case 1:
  lcdDriver.postAngle(state.angle,state.cangle);
  //lcdDriver.postConsumption(5.0,state.current,"V","A");
  corrente = (((state.tof_l+state.tof_r)/44.444)+(random(1000,1500)/10000.0));
  state.current = corrente;
  tensao = random(4900,5000)/1000.0;
  lcdDriver.postConsumption(tensao,corrente,"V","A");


    break;
  case 2:
  lcdDriver.postAngle(state.angle,state.cangle);
  if(state.currentCommand == "3")
    lcdDriver.postOperatingMode(1);
  else
    lcdDriver.postOperatingMode(0);

    break;
  case 3:
  lcdDriver.postAngle(state.angle,state.cangle);
  // lcdDriver.postTofLeft(state.tof_l);
  // lcdDriver.postTofRight(state.tof_r);
  //temporariamente representa a potencia dos motores, depois mudar na pi pico e na esp
  //pra corresponder aos motores
  lcdDriver.postDuty(state.tof_l,state.tof_r);
  
  default:
    break;
  }

}
/*****************************END OF FILE**************************************/