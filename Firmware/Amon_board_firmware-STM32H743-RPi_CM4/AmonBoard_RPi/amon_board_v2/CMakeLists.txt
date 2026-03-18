cmake_minimum_required(VERSION 3.10)
project(AmonLander_RPi C)

# Essential for VS Code IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # Generates compile_commands.json

# Set C standard
set(CMAKE_C_STANDARD 11)            # Set the C standard to C11
set(CMAKE_C_STANDARD_REQUIRED ON)   # Require exactly this standard

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Include directories (searched for header files)
include_directories(include)
include_directories(modules/gnss/include)

# Add subdirectory for the module (find another CMakeLists)
add_subdirectory(modules/gnss)

# Add main executable
add_executable(amon_lander src/main.c)

# Link the module library to main executable
target_link_libraries(amon_lander gnss_module)