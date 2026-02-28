
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

add_executable(bench_dma_loopback bench_dma_loopback.c)
target_link_libraries(bench_dma_loopback rp1jtag_static)

add_executable(test_many_shifts test_many_shifts.c)
target_link_libraries(test_many_shifts rp1jtag_static)

add_executable(test_single_shift test_single_shift.c)
target_link_libraries(test_single_shift rp1jtag_static)

add_executable(test_pio_minimal test_pio_minimal.c)
target_link_libraries(test_pio_minimal rp1jtag_static)
if(HAVE_PIOLIB)
    target_compile_definitions(test_pio_minimal PRIVATE HAVE_PIOLIB)
    target_include_directories(test_pio_minimal PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_pio_minimal ${PIOLIB_LIBRARY})
endif()

add_executable(test_dma_sweep test_dma_sweep.c)
target_link_libraries(test_dma_sweep rp1jtag_static)
if(HAVE_PIOLIB)
    target_compile_definitions(test_dma_sweep PRIVATE HAVE_PIOLIB)
    target_include_directories(test_dma_sweep PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_dma_sweep ${PIOLIB_LIBRARY})
endif()

# Hardware tests are NOT added to CTest (they need real hardware).
# Run them manually:
#   sudo ./test_pio_loopback
#   sudo ./test_gpio_loopback
#   sudo ./test_idcode
#   sudo ./bench_dma_loopback
#   sudo ./bench_throughput
