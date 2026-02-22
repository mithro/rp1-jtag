
# Hardware integration tests (require RPi 5 with PIOLib)

add_executable(test_pio_loopback test_pio_loopback.c)
target_link_libraries(test_pio_loopback rp1jtag_static)

add_executable(test_gpio_loopback test_gpio_loopback.c)
target_link_libraries(test_gpio_loopback rp1jtag_static)

add_executable(test_target_loopback test_target_loopback.c)
target_link_libraries(test_target_loopback rp1jtag_static)

add_executable(test_idcode test_idcode.c)
target_link_libraries(test_idcode rp1jtag_static)

add_executable(test_bitstream test_bitstream.c)
target_link_libraries(test_bitstream rp1jtag_static)

add_executable(bench_throughput bench_throughput.c)
target_link_libraries(bench_throughput rp1jtag_static)

add_executable(test_dma_explore test_dma_explore.c)
target_link_libraries(test_dma_explore rp1jtag_static)
if(HAVE_PIOLIB)
    # test_dma_explore calls PIOLib directly (not through backend),
    # so it needs HAVE_PIOLIB, PIOLib includes, and the PIO program headers.
    target_compile_definitions(test_dma_explore PRIVATE HAVE_PIOLIB)
    target_include_directories(test_dma_explore PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src
    )
    target_link_libraries(test_dma_explore ${PIOLIB_LIBRARY})
endif()

# Hardware tests are NOT added to CTest (they need real hardware).
# Run them manually:
#   sudo ./test_pio_loopback
#   sudo ./test_gpio_loopback
#   sudo ./test_idcode
#   sudo ./test_dma_explore
