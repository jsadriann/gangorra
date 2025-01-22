#include "message.hpp"

json Message::Accel::to_json() const {
    return json{{"x", x}, {"y", y}, {"z", z}};
}

Message::Accel Message::Accel::from_json(const json& j) {
    return Accel{j.at("x").get<int>(), j.at("y").get<int>(), j.at("z").get<int>()};
}

json Message::to_json() const {
    return json{
        {"messageType", messageType},
        {"modeType", modeType},
        {"command", command},
        {"tofLeft", tofLeft},
        {"tofRight", tofRight},
        {"P", P},
        {"I", I},
        {"D", D},
        {"angle", angle},
        {"accel", accel.to_json()}
    };
}

Message Message::from_json(const json& j) {
    return Message{
        j.at("messageType").get<std::string>(),
        j.at("modeType").get<std::string>(),
        j.at("command").get<std::string>(),
        j.at("tofLeft").get<int>(),
        j.at("tofRight").get<int>(),
        j.at("P").get<float>(),
        j.at("I").get<float>(),
        j.at("D").get<float>(),
        j.at("angle").get<float>(),
        Accel::from_json(j.at("accel"))
    };
}

std::string serialize(const Message& message) {
    return message.to_json().dump(4);
}

Message deserialize(const std::string& json_string) {
    json j = json::parse(json_string);
    return Message::from_json(j);
}
