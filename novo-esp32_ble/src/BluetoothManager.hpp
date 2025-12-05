/*
 * =====================================================================================
 *
 *       Filename:  BluetoothManager.hpp
 *
 *    Description:  -
 *
 *        Version:  1.0
 *        Created:  17/11/2024 18:36:30
 *       Revision:  none
 *       Compiler:  -
 *
 *         Author:  Isaac Vinicius, isaacvinicius2121@alu.ufc.br
 *   Organization:  UFC-Quixadá
 *
 * =====================================================================================
*/

#ifndef BLUETOOTHMANAGER_HPP
#define BLUETOOTHMANAGER_HPP

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* UUIDs para o serviço e características | https://www.uuidgenerator.net/ */
#define SERVICE_UUID "ab0828b1-198e-4351-b779-901fa0e0371e"
#define CHARACTERISTIC_UUID_RX "4ac8a682-9736-4e5d-932b-e9b31405049c"
#define CHARACTERISTIC_UUID_TX "84d4f420-e7f0-4b0c-b16a-a125b0521aed" 

/* Nome do servidor BLE */
#define bleServerName "Demonstrador_Didatico"

/* Prototipação de funções */
void setupBluetooth();
void startAdvertising();
void sendString(const char *message);
#endif
/*****************************END OF FILE**************************************/