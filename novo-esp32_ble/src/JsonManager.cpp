/*
 * =====================================================================================
 *
 *       Filename:  JsonManager.cpp
 *
 *    Description:  -
 *
 *        Version:  1.0
 *        Created:  03/01/2025 00:12:55
 *       Revision:  none
 *       Compiler:  -
 *
 *         Author:  Isaac Vinicius, isaacvinicius2121@alu.ufc.br
 *   Organization:  UFC-Quixadá
 *
 * =====================================================================================
*/

#include "JsonManager.hpp"

bool JsonManager::parseJson(const std::string &jsonString, std::string &comando) {
    // Use DynamicJsonDocument para o buffer de 200 bytes
    DynamicJsonDocument doc(200);

    // Tenta interpretar o JSON
    DeserializationError erro = deserializeJson(doc, jsonString);

    // Se ocorrer erro, retorna falso
    if (erro) {
        Serial.print("Erro ao interpretar JSON: ");
        Serial.println(erro.c_str());
        return false;
    }

    // Verifica se a chave "parametros" existe
    if (!doc.containsKey("parametros")) {
        Serial.println("Erro: JSON inválido, chave 'parametros' ausente.");
        return false;
    }

    // Acessa o objeto "parametros"
    JsonObject parametros = doc["parametros"];

    // Verifica se a chave "comando" existe
    if (!doc.containsKey("comando")) {
        Serial.println("Erro: JSON inválido, chave 'comando' ausente.");
        return false;
    }

    // Atribui o comando à variável
    comando = doc["comando"].as<std::string>();

    // Retorna verdadeiro se tudo foi processado corretamente
    return true;
}