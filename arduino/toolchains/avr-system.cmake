set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(AVR_GCC_PROGRAM NAMES avr-gcc REQUIRED)
find_program(AVR_GXX_PROGRAM NAMES avr-g++ REQUIRED)

set(CMAKE_C_COMPILER "${AVR_GCC_PROGRAM}" CACHE FILEPATH "" FORCE)
set(CMAKE_CXX_COMPILER "${AVR_GXX_PROGRAM}" CACHE FILEPATH "" FORCE)
set(CMAKE_ASM_COMPILER "${AVR_GCC_PROGRAM}" CACHE FILEPATH "" FORCE)

# Disable standard library detection for AVR cross-compilation
set(CMAKE_CXX_ABI_INCLUDE_DIR "" CACHE FILEPATH "" FORCE)
set(CMAKE_CXX_ABI "" CACHE STRING "" FORCE)

# Set proper flags for AVR compilation
set(CMAKE_C_FLAGS_INIT "-Wall -Wextra")

# avr-libc 2.0.0 (Debian/Raspberry Pi OS bookworm-trixie) guards DECIMAL_DIG
# in float.h behind __STDC_VERSION__, which is unset for C++, so WString.cpp
# fails to build. __DBL_DECIMAL_DIG__ is a GCC builtin, independent of
# avr-libc's headers, and matches what a fixed float.h would define (AVR's
# double is single precision, so DBL and FLT decimal digits are the same).
set(CMAKE_CXX_FLAGS_INIT "-Wall -Wextra -std=c++11 -DDECIMAL_DIG=__DBL_DECIMAL_DIG__")
