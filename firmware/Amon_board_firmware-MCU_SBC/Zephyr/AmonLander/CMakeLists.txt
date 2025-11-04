# CMakeLists.txt

cmake_minimum_required(VERSION 3.20.0)

# Add cunstom module to Kconfig - LoRa
#set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/modules/wms_comm")

#list(APPEND BOARD_ROOT ${APPLICATION_SOURCE_DIR})#

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(AmonLander)

# Set custom board location
set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND ZEPHYR_BOARD_DIR ${CMAKE_SOURCE_DIR}/boards)

# Ozone debugging path correction
if (WIN32)
    # Windows path
    set(WINDOWS_SOURCE_PATH "Y:/home/tinta/workspace/AmonLander")
    set(DOCKER_SOURCE_PATH "/workspace/AmonLander")
    
    # Remap Docker paths to Windows paths
    zephyr_compile_options(
        -fmacro-prefix-map=${DOCKER_SOURCE_PATH}=${WINDOWS_SOURCE_PATH}
        -ffile-prefix-map=${DOCKER_SOURCE_PATH}=${WINDOWS_SOURCE_PATH}
    )
endif()

target_sources(app PRIVATE src/main.c)
