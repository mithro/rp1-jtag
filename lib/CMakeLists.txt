
# librp1jtag - Core C library

# Sources that always build (no PIOLib dependency)
set(LIB_SOURCES
    src/rp1_jtag.c
    src/pio_backend_mock.c
)

# Real backend only when PIOLib is available
if(HAVE_PIOLIB)
    list(APPEND LIB_SOURCES src/pio_backend_rp1.c)
endif()

add_library(rp1jtag ${LIB_SOURCES})

target_include_directories(rp1jtag
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(HAVE_PIOLIB)
    target_compile_definitions(rp1jtag PRIVATE HAVE_PIOLIB)
    target_include_directories(rp1jtag PRIVATE ${PIOLIB_INCLUDE_DIR})
    target_link_libraries(rp1jtag PRIVATE ${PIOLIB_LIBRARY})
endif()

# Export the mock backend for test use
# (tests link against rp1jtag and use mock backend directly)
