
# librp1jtag - Core C library

include(GNUInstallDirs)

# All sources always build. pio_backend_rp1.c has an #ifdef HAVE_PIOLIB
# guard that provides a stub returning NULL when PIOLib is unavailable.
set(LIB_SOURCES
    src/rp1_jtag.c
    src/pio_backend_mock.c
    src/pio_backend_rp1.c
)

# Build both shared and static libraries
add_library(rp1jtag SHARED ${LIB_SOURCES})
add_library(rp1jtag_static STATIC ${LIB_SOURCES})
set_target_properties(rp1jtag_static PROPERTIES OUTPUT_NAME rp1jtag)

# Shared library versioning
set_target_properties(rp1jtag PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0
)

foreach(_tgt rp1jtag rp1jtag_static)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    if(HAVE_PIOLIB)
        find_package(Threads REQUIRED)
        target_compile_definitions(${_tgt} PRIVATE HAVE_PIOLIB)
        target_include_directories(${_tgt} PRIVATE ${PIOLIB_INCLUDE_DIR})
        target_link_libraries(${_tgt} PRIVATE ${PIOLIB_LIBRARY} Threads::Threads)
    endif()
endforeach()

# Install targets
install(TARGETS rp1jtag rp1jtag_static
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES include/rp1_jtag.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
