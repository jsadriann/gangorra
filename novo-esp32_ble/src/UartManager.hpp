/*
 * =====================================================================================
 *
 *       Filename:  UartManager.hpp
 *
 *    Description:  -
 *
 *        Version:  1.0
 *        Created:  03/01/2025 10:11:15
 *       Revision:  none
 *       Compiler:  -
 *
 *         Author:  Isaac Vinicius, isaacvinicius2121@alu.ufc.br
 *   Organization:  UFC-Quixadá
 *
 * =====================================================================================
*/

#ifndef UARTMANAGER_HPP
#define UARTMANAGER_HPP
#include <string>

void UARTManager_init(long baudRate, int rxPin, int txPin);
void receive_package(void);
void process_package(String package);
void getKp(void);
void getKi(void);
void getKd(void);
void getTof_r(void);
void getTof_l(void);
void getAccx(void);
void getAccy(void);
void getAccz(void);
void getAngle(void);
void getCurrent(void);
void getAll(void);
void execute_command(const String &command);
void sendDataPackage(String command, int type);
#endif
/*****************************END OF FILE**************************************/