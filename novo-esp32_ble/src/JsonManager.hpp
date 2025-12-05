/*
 * =====================================================================================
 *
 *       Filename:  JsonManager.hpp
 *
 *    Description:  -
 *
 *        Version:  1.0
 *        Created:  03/01/2025 00:07:12
 *       Revision:  none
 *       Compiler:  -
 *
 *         Author:  Isaac Vinicius, isaacvinicius2121@alu.ufc.br
 *   Organization:  UFC-Quixadá
 *
 * =====================================================================================
*/

#ifndef JSONMANAGER_HPP
#define JSONMANAGER_HPP

#include <ArduinoJson.h>
#include <string>

// Declaração de funções e classes para gerenciar JSON
class JsonManager {
public:
    static bool parseJson(const std::string &jsonString, std::string &comando);
};
#endif
/*****************************END OF FILE**************************************/