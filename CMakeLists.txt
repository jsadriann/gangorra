cmake_minimum_required(VERSION 3.13)

# Importa o SDK da Raspberry Pi Pico
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Importa o FreeRTOS
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Nome do projeto
project(app C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Inicializa o Pico SDK
pico_sdk_init()

# Adiciona o executável principal
add_executable(main
    src/main.cpp     
    src/message.cpp          # Código principal
    drivers/i2c_sensors.cpp
    drivers/tof.cpp
    handlers/tof_handler.cpp
    drivers/mpu6050.cpp        # Implementação do driver do sensor
    handlers/mpu6050_handler.cpp # Implementação das tarefas relacionadas aos sensores
    handlers/serial.cpp 
)

# Adiciona os diretórios de include
target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/includes  # Diretório dos cabeçalhos
    ${CMAKE_CURRENT_LIST_DIR}/drivers  # Diretório com implementações dos drivers
    ${CMAKE_CURRENT_LIST_DIR}/handlers # Diretório com implementações dos handlers
)

add_library(freertos_config INTERFACE)

# Vincula as bibliotecas necessárias
target_link_libraries(main
    pico_stdlib               # Biblioteca padrão do Pico SDK
    hardware_i2c              # Biblioteca de hardware I2C do Pico SDK
    FreeRTOS-Kernel  
    freertos_config         # Núcleo do FreeRTOS
    FreeRTOS-Kernel-Heap4     # Gerenciador de heap do FreeRTOS
)

pico_enable_stdio_usb(main 1)

# Gera arquivos extras (UF2, bin, map, etc.)
pico_add_extra_outputs(main)

