cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME PID_Pendulum)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
#
# 分层结构（详细说明见 README）：
#   driver/common     公共类型（error_t）
#   driver/bus        总线驱动，直接贴着 HAL
#   driver/device     器件驱动，只依赖抽象总线接口
#   bsp               板级支持，把上面几层按本板接线组装起来
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/bus/i2c/i2c_soft.c
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/device/ssd1306/ssd1306.c
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/device/ssd1306/ssd1306_fonts.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/oled/bsp_oled.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/tick/bsp_tick.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/key/bsp_key.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/common
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/bus/i2c
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/device/ssd1306
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/oled
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/tick
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/key
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
