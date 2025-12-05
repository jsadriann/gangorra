/*
 * =====================================================================================
 *
 *       Filename:  GlobalVariables.hpp
 *
 *    Description:  -
 *
 *        Version:  1.0
 *        Created:  13/01/2025 19:16:14
 *       Revision:  none
 *       Compiler:  -
 *
 *         Author:  Isaac Vinicius, isaacvinicius2121@alu.ufc.br
 *   Organization:  UFC-Quixadá
 *
 * =====================================================================================
*/

#ifndef GLOBAL_VARIABLES_HPP
#define GLOBAL_VARIABLES_HPP

#include <string>

struct SystemState {
    float tof_r, tof_l;
    float accx, accy, accz;
    float angle,cangle, current;
    float Kp, Ki, Kd;
    int messageType;
    String currentCommand;
    // bool updated;

    SystemState() : tof_r(0), tof_l(0), accx(0.0), accy(0.0), accz(0.0),
                    angle(0.0), cangle(0.0), current(0.0), Kp(0.0), Ki(0.0), Kd(0.0),
                    messageType(0), currentCommand("") {}
};

extern SystemState state;

#endif
/*****************************END OF FILE**************************************/