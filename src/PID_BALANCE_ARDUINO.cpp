#include <cmath>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "motor.hpp"
#include "mpu6050.hpp"

float elapsedTime, time, timePrev;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0, pid_i = 0, pid_d = 0;

// Constantes do PID
double kp = 3.55;
double ki = 0.005;
double kd = 2.05;

double throttle = 1300;
float desired_angle = 0;

void loop() {
    printf("-------------------------------------------------------------------------------------\n");
    timePrev = time;
    time = to_us_since_boot(get_absolute_time()) / 1000000.0f;
    elapsedTime = time - timePrev;
    printf("Time %.10f %.10f \n", elapsedTime, time);

    error = Total_angle[1] - desired_angle;
    pid_p = kp * error;
    if (-3 < error && error < 3) {
        pid_i += ki * error;
    }
    pid_d = kd * ((error - previous_error) / elapsedTime);
    PID = pid_p + pid_i + pid_d;
    printf("P: %.2f I: %.2f D: %.2f \n", pid_p, pid_i, pid_d);
    // Limitar valores do PID
    if (PID < -1000) PID = -1000;
    if (PID > 1000) PID = 1000;

    printf("PID: %.2f\n", PID);

    // Cálculo do PWM
    pwmLeft = throttle + PID;
    pwmRight = throttle - PID;
    if (pwmLeft < 1000) pwmLeft = 1000;
    if (pwmLeft > 2000) pwmLeft = 2000;
    if (pwmRight < 1000) pwmRight = 1000;
    if (pwmRight > 2000) pwmRight = 2000;

    const int dutyLeft = (pwmLeft - 1000) / 10.0;
    const int dutyRight = (pwmRight - 1000) / 10.0;

    previous_error = error;
    printf("-------------------------------------------------------------------------------------\n");

}

int main() {
    while (true) {
        loop();
        sleep_ms(1000);
    }
}
