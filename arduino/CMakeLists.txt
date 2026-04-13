cmake_minimum_required(VERSION 3.23)

project(robot_control_interface LANGUAGES C CXX ASM)

set(MCU atmega2560)
set(F_CPU 16000000UL)
set(ARDUINO_BOARD_DEFINES
    ARDUINO=10819
    ARDUINO_ARCH_AVR
    ARDUINO_AVR_MEGA2560
)

set(ARDUINO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ArduinoCore-avr-1.8.7)
set(ARDUINO_CORE_DIR ${ARDUINO_DIR}/cores/arduino)
set(ARDUINO_VARIANT_DIR ${ARDUINO_DIR}/variants/mega)
set(VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor)
set(HEX_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Generic")
  message(FATAL_ERROR "Configure this project with an AVR toolchain file. See CMakePresets.json.")
endif()

if(DEFINED AVR_TOOLCHAIN_ROOT AND AVR_TOOLCHAIN_ROOT)
  set(CMAKE_FIND_ROOT_PATH "${AVR_TOOLCHAIN_ROOT};${CMAKE_FIND_ROOT_PATH}")
endif()

if(DEFINED AVR_STDLIB_INCLUDE_DIR AND AVR_STDLIB_INCLUDE_DIR)
  add_compile_options(-isystem "${AVR_STDLIB_INCLUDE_DIR}")
endif()

find_program(AVR_OBJCOPY_PROGRAM NAMES avr-objcopy REQUIRED)
find_program(AVR_SIZE_PROGRAM NAMES avr-size REQUIRED)
find_program(AVRDUDE_PROGRAM NAMES avrdude)

set(AVRDUDE_CONFIG
    ${ARDUINO_DIR}/bootloaders/gemma/avrdude.conf
    CACHE FILEPATH
    "Path to avrdude.conf"
)
set(UPLOAD_PORT COM3 CACHE STRING "Serial port used to flash the board")
set(UPLOAD_BAUD 115200 CACHE STRING "Upload baud rate")
set(UPLOAD_PROTOCOL wiring CACHE STRING "avrdude programmer/protocol")

file(GLOB ARDUINO_CORE_SOURCES CONFIGURE_DEPENDS
    ${ARDUINO_CORE_DIR}/*.c
    ${ARDUINO_CORE_DIR}/*.cpp
    ${ARDUINO_CORE_DIR}/*.S
    ${ARDUINO_VARIANT_DIR}/*.c
)

add_library(arduino_core STATIC ${ARDUINO_CORE_SOURCES})
target_include_directories(arduino_core PUBLIC
    ${ARDUINO_CORE_DIR}
    ${ARDUINO_VARIANT_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VENDOR_DIR}
    ${VENDOR_DIR}/GyverMotor2
    ${VENDOR_DIR}/uPID
    ${VENDOR_DIR}/uEncoder/src
    ${VENDOR_DIR}/GyverIO/src
)
target_compile_definitions(arduino_core PUBLIC ${ARDUINO_BOARD_DEFINES})
target_compile_options(arduino_core PUBLIC
    -mmcu=${MCU}
    -DF_CPU=${F_CPU}
    -Os
    -ffunction-sections
    -fdata-sections
    -fno-exceptions
    -fno-threadsafe-statics
    -flto
)
target_compile_features(arduino_core PUBLIC c_std_11 cxx_std_11)
target_compile_options(arduino_core PUBLIC
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)
set_target_properties(arduino_core PROPERTIES
    ARCHIVE_OUTPUT_NAME arduino_core
)

add_executable(${PROJECT_NAME}.elf
    src/main.cpp
    src/encoder.cpp
)
target_include_directories(${PROJECT_NAME}.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VENDOR_DIR}
    ${VENDOR_DIR}/GyverMotor2
    ${VENDOR_DIR}/uPID
    ${VENDOR_DIR}/uEncoder/src
    ${VENDOR_DIR}/GyverIO/src
)
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE ${ARDUINO_BOARD_DEFINES})
target_compile_options(${PROJECT_NAME}.elf PRIVATE
    -mmcu=${MCU}
    -DF_CPU=${F_CPU}
    -Os
    -ffunction-sections
    -fdata-sections
    -fno-exceptions
    -fno-threadsafe-statics
    -flto
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)
target_compile_features(${PROJECT_NAME}.elf PRIVATE c_std_11 cxx_std_11)
target_link_options(${PROJECT_NAME}.elf PRIVATE
    -mmcu=${MCU}
    -Wl,--gc-sections
    -flto
    -Wl,--whole-archive
    $<TARGET_FILE:arduino_core>
    -Wl,--no-whole-archive
)
target_link_libraries(${PROJECT_NAME}.elf PRIVATE arduino_core)

add_custom_command(
    TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${AVR_OBJCOPY_PROGRAM} -O ihex -R .eeprom
        $<TARGET_FILE:${PROJECT_NAME}.elf>
        ${HEX_OUTPUT}
    COMMAND ${AVR_SIZE_PROGRAM} --mcu=${MCU} --format=avr
        $<TARGET_FILE:${PROJECT_NAME}.elf>
    VERBATIM
)

if(AVRDUDE_PROGRAM)
  add_custom_target(flash
        COMMAND ${AVRDUDE_PROGRAM}
            -C ${AVRDUDE_CONFIG}
            -p ${MCU}
            -c ${UPLOAD_PROTOCOL}
            -P ${UPLOAD_PORT}
            -b ${UPLOAD_BAUD}
            -D
            -U flash:w:${HEX_OUTPUT}:i
        DEPENDS ${PROJECT_NAME}.elf
        USES_TERMINAL
        VERBATIM
    )
else()
  add_custom_target(flash
        COMMAND ${CMAKE_COMMAND} -E echo "avrdude was not found. Set AVRDUDE_PROGRAM to the avrdude executable path."
        COMMAND ${CMAKE_COMMAND} -E false
        DEPENDS ${PROJECT_NAME}.elf
        VERBATIM
    )
endif()
