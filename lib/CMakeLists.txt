
# librp1jtag - Core C library

# All sources always build. pio_backend_rp1.c has an #ifdef HAVE_PIOLIB
# guard that provides a stub returning NULL when PIOLib is unavailable.
set(LIB_SOURCES
    src/rp1_jtag.c
    src/pio_backend_mock.c
    src/pio_backend_rp1.c
)

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
