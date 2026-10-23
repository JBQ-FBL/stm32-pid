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
set(CMAKE_PROJECT_NAME Pid)

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
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # ── 自己写的应用层（App/，不来自驱动库，drv 不碰）──
    ${CMAKE_CURRENT_SOURCE_DIR}/App/pid/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/cmd/cmd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/cmd/uart-link.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/speed-loop/speed-loop.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/vofa/vofa.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/ui/ui.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/debug/fault-report.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/motor-tb6612/motor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/motor-tb6612/motor_encoder.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/oled-ssd1306/oled.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/oled-ssd1306/oled_font.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/App/pid
    ${CMAKE_CURRENT_SOURCE_DIR}/App/cmd
    ${CMAKE_CURRENT_SOURCE_DIR}/App/speed-loop
    ${CMAKE_CURRENT_SOURCE_DIR}/App/vofa
    ${CMAKE_CURRENT_SOURCE_DIR}/App/ui
    ${CMAKE_CURRENT_SOURCE_DIR}/App/debug
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/motor-tb6612
    ${CMAKE_CURRENT_SOURCE_DIR}/Components/oled-ssd1306
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols

    # ── motor-tb6612 驱动在本工程的接线（库默认值是 TIM-Motor 工程的接法，必须覆盖）──
    MOTOR_PWM_TIM_CHANNEL=TIM_CHANNEL_1   # 本工程 PA8 = TIM1_CH1；库默认 TIM_CHANNEL_3
    MOTOR_ENC_PPR=11                      # 编码器电机轴每圈 11 个脉冲
    MOTOR_ENC_GEAR_RATIO=30.0f            # 减速比 1:30（库默认 1.0 = 直驱）
    MOTOR_ENC_MA_N=3                      # 测速窗 3×5ms=15ms → 量子 9.09→3.03 rpm
                                          # 群延迟 (N-1)/2×5=5 ms；想让屏上更平滑可试 4~5，但上升时间 +2.5ms/级
    MOTOR_USE_STBY=0                      # STBY 硬件拉高到 3V3，不用 GPIO 管
                                          # （若改成 1，必须在 CubeMX 里把某脚 Label 设成 STBY，
                                          #   否则 motor.h 会回退到 PA6 = 编码器 A 相，测速直接废）
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
