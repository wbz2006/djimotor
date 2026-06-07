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

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 解决SEGGER RTT在GCC 12+版本下的未知目标类型错误
add_definitions(-D__ARM_ARCH=7)
add_definitions(-D__ARM_ARCH_7M__)
add_definitions(-D__ARM_ARCH_PROFILE='M')

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME djimotor)

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

file(GLOB_RECURSE LIB_SOURCES
    # Add user defined library sources here, e.g.
    # ${CMAKE_SOURCE_DIR}/path/to/lib/*.c
    "bsp/*.c"
    "modules/*.c"
    "SEGGER/RTT/*.c"
    "SEGGER/Config/*.c"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${LIB_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/SEGGER/RTT
    ${CMAKE_SOURCE_DIR}/SEGGER/Config
    ${CMAKE_SOURCE_DIR}/bsp/can
    ${CMAKE_SOURCE_DIR}/bsp/dwt
    ${CMAKE_SOURCE_DIR}/bsp/log
    ${CMAKE_SOURCE_DIR}/bsp/usart
    ${CMAKE_SOURCE_DIR}/modules
    ${CMAKE_SOURCE_DIR}/modules/algorithm
    ${CMAKE_SOURCE_DIR}/modules/motor
    ${CMAKE_SOURCE_DIR}/modules/motor/DJImotor
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
