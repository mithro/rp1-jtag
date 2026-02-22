cmake_minimum_required(VERSION 3.16)
project(rp1-jtag
    VERSION 0.1.0
    LANGUAGES C CXX
    DESCRIPTION "High-speed JTAG via Raspberry Pi 5 RP1 PIO"
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Check for PIOLib (optional -- only available on RPi 5)
find_path(PIOLIB_INCLUDE_DIR NAMES pio_platform.h
    PATHS /usr/local/include /usr/include
)
find_library(PIOLIB_LIBRARY NAMES piolib
    PATHS /usr/local/lib /usr/lib
)

if(PIOLIB_INCLUDE_DIR AND PIOLIB_LIBRARY)
    set(HAVE_PIOLIB TRUE)
    message(STATUS "PIOLib found: ${PIOLIB_INCLUDE_DIR}")
else()
    set(HAVE_PIOLIB FALSE)
    message(STATUS "PIOLib not found -- building without hardware support")
endif()

# Core library
add_subdirectory(lib)

# Tests
enable_testing()
add_subdirectory(tests)

# Examples (only with PIOLib)
if(HAVE_PIOLIB)
    add_subdirectory(examples)
endif()
