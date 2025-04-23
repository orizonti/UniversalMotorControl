cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project("UniversalMotorControllerG473" C CXX ASM)

include(cmake/st-project.cmake)

add_executable(${PROJECT_NAME})
add_st_target_properties(${PROJECT_NAME})

include_directories("${PROJECT_SOURCE_DIR}/Drivers/ETHERNET")
include_directories("{PROJECT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc")
include_directories("{PROJECT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy")
include_directories("{PROJECT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/CMSIS/Include")
include_directories("{PROJECT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/CMSIS/Device/ST/STM32G4XX/Include")
include_directories("{PROJECT_SOURCE_DIR}/Core/Inc")
include_directories("C:/HEADERS/STM32_HEADERS")
include_directories("C:/STD_HEADERS")

message("${PROJECT_SOURCE_DIR}/Drivers/ETHERNET")
message("BINARY DIR: ${CMAKE_CURRENT_BINARY_DIR}")
set(USE_HAL_DRIVER 1)

add_custom_target(
    copy-compile-commands ALL
    ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
    ${CMAKE_CURRENT_BINARY_DIR}/../../compile_commands.json
    )