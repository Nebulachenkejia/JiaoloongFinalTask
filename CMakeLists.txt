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
set(CMAKE_PROJECT_NAME JiaoloongFinalTask)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/freertos.c
        Core/Src/stm32f4xx_hal_timebase_tim.c
        Core/Inc/FreeRTOSConfig.h
        UserCode/RTOS/user_tasks.cpp
        UserCode/RTOS/user_tasks.h
        UserCode/imu/bmi088.h
        UserCode/imu/imu.h
        UserCode/imu/mahony.h
        UserCode/imu/imu.cpp
        UserCode/Motor/GM6020.cpp
        UserCode/Motor/GM6020.h
        UserCode/imu/bmi088.cpp
        UserCode/Motor/pid.cpp
        UserCode/Motor/pid.h
        UserCode/RC/DT7_RC.cpp
        UserCode/RC/DT7_RC.h
        UserCode/app/callback.cpp
        UserCode/app/callback.h
        UserCode/imu/mahony.cpp
        UserCode/imu/matrix.h
        UserCode/RTOS/user_taskes_cpp.hpp)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        UserCode/RTOS/user_tasks.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        UserCode/Inc
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
