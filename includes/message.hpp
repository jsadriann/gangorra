#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <string>
#include "json.hpp"

using json = nlohmann::json;

class Message {
public:
    std::string messageType;
    std::string modeType;
    std::string command;
    int tofLeft;
    int tofRight;
    float P;
    float I;
    float D;
    float angle;

    struct Accel {
        int x;
        int y;
        int z;

        json to_json() const;
        static Accel from_json(const json& j);
    } accel;

    json to_json() const;
    static Message from_json(const json& j);
};

std::string serialize(const Message& message);
Message deserialize(const std::string& json_string);

#endif // MESSAGE_HPP
