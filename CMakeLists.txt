cmake_minimum_required(VERSION 3.14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)
set(PICO_BOARD adafruit_feather_rp2040)

# check for valid pico-sdk installation
if(DEFINED PICO_SDK_PATH)
    if(NOT EXISTS ${CMAKE_SOURCE_DIR}/pico_sdk_import.cmake)
        message("Copying ${PICO_SDK_PATH}/external/pico_sdk_import.cmake to ${CMAKE_SOURCE_DIR}")
        file(COPY ${PICO_SDK_PATH}/external/pico_sdk_import.cmake
            DESTINATION ${CMAKE_SOURCE_DIR})
    endif()
    include(pico_sdk_import.cmake)
else()
    if(EXISTS ${CMAKE_SOURCE_DIR}/lib/pico-sdk/pico_sdk_init.cmake)
        include(lib/pico-sdk/pico_sdk_init.cmake)
    else()
        message(FATAL_ERROR "Path to pico-sdk source not found. Either provide the path to CMake with\n"
        "   -DPICO_SDK_PATH=/path/to/dir\n"
        "or run\n"
        "   git submodule update --init lib/pico-sdk\n"
        "   git -C lib/pico-sdk submodule update --init")
    endif()
endif()

project(ht_sensor LANGUAGES C CXX ASM)

pico_sdk_init()

# check for valid FreeRTOS-Kernel installation
if(NOT DEFINED PICO_SDK_FREERTOS_SOURCE)
    # it may be located here
    if(EXISTS ${PICO_SDK_PATH}/../FreeRTOS-Kernel/CMakeLists.txt)
        set(PICO_SDK_FREERTOS_SOURCE ${PICO_SDK_PATH}/../FreeRTOS-Kernel)
    elseif(EXISTS ${CMAKE_SOURCE_DIR}/lib/FreeRTOS-Kernel/CMakeLists.txt)
        set(PICO_SDK_FREERTOS_SOURCE ${CMAKE_SOURCE_DIR}/lib/FreeRTOS-Kernel)
    else()
        message(FATAL_ERROR "Path to FreeRTOS source not found. Either provide the path to CMake with\n"
        "   -DPICO_SDK_FREERTOS_SOURCE=/path/to/dir\n"
        "or run\n"
        "   git -C lib/FreeRTOS-Kernel submodule update --init")
    endif()
    message(STATUS "Found FreeRTOS-Kernel at ${PICO_SDK_FREERTOS_SOURCE}")
else()
    message(STATUS "Found FreeRTOS-Kernel at ${PICO_SDK_FREERTOS_SOURCE}")
endif()

add_library(freertos
    ${PICO_SDK_FREERTOS_SOURCE}/event_groups.c
    ${PICO_SDK_FREERTOS_SOURCE}/list.c
    ${PICO_SDK_FREERTOS_SOURCE}/queue.c
    ${PICO_SDK_FREERTOS_SOURCE}/stream_buffer.c
    ${PICO_SDK_FREERTOS_SOURCE}/tasks.c
    ${PICO_SDK_FREERTOS_SOURCE}/timers.c
    ${PICO_SDK_FREERTOS_SOURCE}/portable/MemMang/heap_3.c
    ${PICO_SDK_FREERTOS_SOURCE}/portable/GCC/ARM_CM0/port.c
    )

target_include_directories(freertos PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${PICO_SDK_FREERTOS_SOURCE}/include
    ${PICO_SDK_FREERTOS_SOURCE}/portable/GCC/ARM_CM0
    )

add_executable(${CMAKE_PROJECT_NAME} src/main.c)

pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 1)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})

target_link_libraries(${CMAKE_PROJECT_NAME}
    pico_stdlib
    hardware_gpio
    hardware_i2c
    hardware_adc
    freertos
    )
