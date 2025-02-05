#include "mpu6050.hpp"

inline void mpu6050::_delay_ms(int ms) const {
    for (volatile int i = 0; i < ms * 800; i++) {
        __asm__("nop");
    }
}

mpu6050::mpu6050(i2c_inst *i2c_port, uint16_t sda, uint16_t scl) {
    this->i2c_port = i2c_port;
    this->scl = scl;
    this->sda = sda;

    mpu6050_init();
}

mpu6050::~mpu6050() {
    mpu6050_reset();
}

void mpu6050::mpu6050_reset() const {
    uint8_t reset[] = {REG_PWR_MGMT_1, 0x80};
    i2c_write_blocking(i2c_port, MPU6050_ADDR, reset, 2, false);
    _delay_ms(2000);
    


    uint8_t wake[] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(i2c_port, MPU6050_ADDR, wake, 2, false);
    _delay_ms(200);
    //vTaskDelay(pdMS_TO_TICKS(200)); 
}

void mpu6050::mpu6050_port_configure() const {
    // Initialize chosen serial port
    stdio_init_all();
    // Initialize I2C
    i2c_init(i2c_port, 400 * 1000);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(scl);
}

void mpu6050::mpu6050_sensors_configure() const {
    // Set sample rate
    uint8_t sample_rate[] = {REG_SMPLRT_DIV, SAMPLE_RATE_DIV};
    i2c_write_blocking(i2c_port, MPU6050_ADDR, sample_rate, 2, false);
    //vTaskDelay(pdMS_TO_TICKS(200)); 
    _delay_ms(200);

    uint8_t accel_config[] = {REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE};
    i2c_write_blocking(i2c_port, MPU6050_ADDR, accel_config, 2, false);
    //vTaskDelay(pdMS_TO_TICKS(200)); 
    _delay_ms(200);
    // Set gyroscope range
    uint8_t gyro_config[] = {REG_GYRO_CONFIG, GYRO_CONFIG_VALUE};
    i2c_write_blocking(i2c_port, MPU6050_ADDR, gyro_config, 2, false);
    //vTaskDelay(pdMS_TO_TICKS(200)); 
    _delay_ms(200);
}

void mpu6050::mpu6050_init() const {
    // Reset and configure MPU6050
    mpu6050_port_configure();
    mpu6050_reset();
    mpu6050_sensors_configure();

    uint8_t who_am_i = 0;
    uint8_t reg = WHO_AM_I_REG;

    i2c_write_blocking(i2c_port, MPU6050_ADDR, &reg, 1, true);
    _delay_ms(200);
    i2c_read_blocking(i2c_port, MPU6050_ADDR, &who_am_i, 1, false);
    _delay_ms(200);

    printf("MPU6050 WHO_AM_I: 0x%02X\n", who_am_i);
    _delay_ms(200);

    if (who_am_i != WHO_AM_I_VALUE) {
        printf("MPU6050 not found!\n");
    }
}

int16_t getAcellValues(i2c_inst *i2c_port, uint8_t reg) {
    uint8_t buf[2];

    // Escreve o registro inicial de leitura
    int ret = i2c_write_blocking(i2c_port, MPU6050_ADDR, &reg, 1, true);
    if (ret < 0) {
        printf("Falha na escrita I2C: %d\n", ret);
    }
    int teste = 0;
    // Lê os 14 bytes (acelerômetro, temperatura e giroscópio)
    ret = i2c_read_blocking(i2c_port, MPU6050_ADDR, buf, 2, false);
    if (ret < 0) {
        printf("Falha na leitura I2C: %d\n", ret);
    }

    return (buf[0] << 8) | buf[1];
}

void mpu6050::getAccel(accel *accel) const {
    accel->accel_x = getAcellValues(i2c_port, REG_ACCEL_XOUT_H);
    accel->accel_y = getAcellValues(i2c_port, REG_ACCEL_YOUT_H) ;
    accel->accel_z = getAcellValues(i2c_port, REG_ACCEL_ZOUT_H);
}

void mpu6050::getGyro(gyro *gyro) const {
    gyro->gyro_x = getAcellValues(i2c_port, REG_GYRO_XOUT_H);
    gyro->gyro_y = getAcellValues(i2c_port, REG_GYRO_YOUT_H);
    gyro->gyro_z = getAcellValues(i2c_port, REG_GYRO_ZOUT_H);
}

void mpu6050::print_raw_data(accel accelData, gyro gyroData) const {
    getAccel(&accelData);
    getGyro(&gyroData);

    float accel_g[3];
    accel_g[0] = static_cast<float>(accelData.accel_x) / ACCEL_SCALE_FACTOR;
    accel_g[1] = static_cast<float>(accelData.accel_y) / ACCEL_SCALE_FACTOR;
    accel_g[2] = static_cast<float>(accelData.accel_z) / ACCEL_SCALE_FACTOR;

    // Convert raw gyroscope values to degrees per second
    float gyro_dps[3];
    gyro_dps[0] = gyroData.gyro_x / GYRO_SCALE_FACTOR;
    gyro_dps[1] = gyroData.gyro_y / GYRO_SCALE_FACTOR;
    gyro_dps[2] = gyroData.gyro_z / GYRO_SCALE_FACTOR;

    // Print converted values
    printf("aX = %.2f g | aY = %.2f g | aZ = %.2f g \n", accel_g[0], accel_g[1], accel_g[2]);

    printf("gX = %.2f dps | gY = %.2f dps | gZ = %.2f dps | \n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
}

void mpu6050::toString(char *buffer, size_t buffer_size, accel accelData, gyro gyroData) const {
    getAccel(&accelData);
    getGyro(&gyroData);

    float accel_g[3];
    accel_g[0] = static_cast<float>(accelData.accel_x) / ACCEL_SCALE_FACTOR;
    accel_g[1] = static_cast<float>(accelData.accel_y) / ACCEL_SCALE_FACTOR;
    accel_g[2] = static_cast<float>(accelData.accel_z) / ACCEL_SCALE_FACTOR;

    // Convert raw gyroscope values to degrees per second
    float gyro_dps[3];
    gyro_dps[0] = gyroData.gyro_x / GYRO_SCALE_FACTOR;
    gyro_dps[1] = gyroData.gyro_y / GYRO_SCALE_FACTOR;
    gyro_dps[2] = gyroData.gyro_z / GYRO_SCALE_FACTOR;

    int minimal_Size = snprintf(nullptr, 0,
        "aX = %.2f g | aY = %.2f g | aZ = %.2f g \n"
        "gX = %.2f dps | gY = %.2f dps | gZ = %.2f dps | \n",
        accel_g[0], accel_g[1], accel_g[2],
        gyro_dps[0], gyro_dps[1], gyro_dps[2]);

    if(minimal_Size + 1> buffer_size) {
        snprintf(buffer, buffer_size, "erro, tamanho insuficiente");
        return;
    }

    snprintf(buffer, buffer_size,
        "aX = %.2f g | aY = %.2f g | aZ = %.2f g \n"
        "gX = %.2f dps | gY = %.2f dps | gZ = %.2f dps | \n",
        accel_g[0], accel_g[1], accel_g[2],
        gyro_dps[0], gyro_dps[1], gyro_dps[2]);
}

void mpu6050::getAngle(float &angle, accel accelData) {
    // Converter os dados para unidades de gravidade
    const float accel_x = static_cast<float>(accelData.accel_x) / ACCEL_SCALE_FACTOR;
    const float accel_y = static_cast<float>(accelData.accel_y) / ACCEL_SCALE_FACTOR;
    const float accel_z = static_cast<float>(accelData.accel_z) / ACCEL_SCALE_FACTOR;

    // Calcular o ângulo em relação ao eixo X
    angle = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * (180.0 / M_PI);
}