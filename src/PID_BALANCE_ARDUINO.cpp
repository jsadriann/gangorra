#include <cmath>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "motor.hpp"
#include "mpu6050.hpp"

/* Variáveis do MPU6050 */
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, time, timePrev;
float rad_to_deg = 180 / 3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0, pid_i = 0, pid_d = 0;

// Constantes do PID
double kp = 3.55;
double ki = 0.005;
double kd = 2.05;

double throttle = 1300;
float desired_angle = 0;
__attribute__((section(".rtos_heap"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

gyro gyroData;
accel accelData;
mpu6050 sensor(i2c0, 0, 1);

Motor right_motor(300, 15); // Motor direito (frequência = 50 Hz, GPIO = 2)
Motor left_motor(300, 13);  // Motor esquerdo (frequência = 50 Hz, GPIO = 3)

void setup() {
    sleep_ms(2000);

    // Esperar para conectar os motores
    // Inicialização dos motores
    gpio_set_function(left_motor.gpio1(), GPIO_FUNC_PWM);
    gpio_set_function(right_motor.gpio1(), GPIO_FUNC_PWM);

    left_motor.pwm_set_freq();
    right_motor.pwm_set_freq();

    left_motor.setSpeed(0);
    right_motor.setSpeed(0);

    pwm_set_enabled(left_motor.get_slice_num(), true);
    pwm_set_enabled(right_motor.get_slice_num(), true);

    printf("Inincializando, colocando 30%% para começar\n");
    left_motor.setSpeed(30);
    right_motor.setSpeed(30);

    sleep_ms(3000);
    printf("Começou, colocando 40%% como vel padrão\n");
    left_motor.setSpeed(40);
    right_motor.setSpeed(35);
    sleep_ms(1000);
    printf("acabei a função de init \n");
}

void loop() {
    printf("-------------------------------------------------------------------------------------\n");
    timePrev = time;
    time = to_us_since_boot(get_absolute_time()) / 1000000.0f;
    elapsedTime = time - timePrev;
    printf("Time %.10f %.10f \n", elapsedTime, time);

    sensor.getAccel(&accelData);
    Acc_rawX = accelData.accel_x;
    Acc_rawY = accelData.accel_y;
    Acc_rawZ = accelData.accel_z;
    printf("Acc raw %d %d %d\n", Acc_rawX, Acc_rawY, Acc_rawZ);

    Acceleration_angle[0] = atan((Acc_rawY / ACCEL_SCALE_FACTOR) / sqrt(pow((Acc_rawX / ACCEL_SCALE_FACTOR), 2) + pow((Acc_rawZ / ACCEL_SCALE_FACTOR), 2))) * rad_to_deg;
    Acceleration_angle[1] = atan(-1 * (Acc_rawX / ACCEL_SCALE_FACTOR) / sqrt(pow((Acc_rawY / ACCEL_SCALE_FACTOR), 2) + pow((Acc_rawZ / ACCEL_SCALE_FACTOR), 2))) * rad_to_deg;
    printf("Acc angle %.2f %.2f\n", Acceleration_angle[0], Acceleration_angle[1]);

    sensor.getGyro(&gyroData);
    Gyr_rawX = gyroData.gyro_x;
    Gyr_rawY = gyroData.gyro_y;

    Gyro_angle[0] = Gyr_rawX / GYRO_SCALE_FACTOR;
    Gyro_angle[1] = Gyr_rawY / GYRO_SCALE_FACTOR;
    printf("%d %d \n", Gyr_rawX, Gyr_rawY);
    printf("Gyro angle %.2f %.2f\n", Gyro_angle[0], Gyro_angle[1]);

    Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0] * elapsedTime) + 0.02 * Acceleration_angle[0];
    Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_angle[1] * elapsedTime) + 0.02 * Acceleration_angle[1];
    printf("Total Angle %.4f %.4f \n", Total_angle[0], Total_angle[1]);

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

    printf("Pwm Left: %.2f Pwm right: %.2f \n", pwmLeft, pwmRight);

    const int dutyLeft = (pwmLeft - 1000) / 10.0;
    const int dutyRight = (pwmRight - 1000) / 10.0;

    // Atualização dos motores
    left_motor.setSpeed(dutyLeft);
    right_motor.setSpeed(dutyRight);

    printf("Duty Left: %d Right: %d \n", dutyLeft, dutyRight);
    previous_error = error;
    sensor.print_raw_data(accelData, gyroData);
    printf("-------------------------------------------------------------------------------------\n");

}

int main() {
    setup();
    while (true) {
        loop();
        sleep_ms(1000);
    }
}
