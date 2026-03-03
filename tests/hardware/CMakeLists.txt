
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

add_executable(test_bypass_loopback test_bypass_loopback.c)
target_link_libraries(test_bypass_loopback rp1jtag_static)

add_executable(test_dma_diag test_dma_diag.c)
target_link_libraries(test_dma_diag rp1jtag_static)

add_executable(test_nodma_shift test_nodma_shift.c)
target_link_libraries(test_nodma_shift rp1jtag_static)
if(HAVE_PIOLIB)
    target_compile_definitions(test_nodma_shift PRIVATE HAVE_PIOLIB)
    target_include_directories(test_nodma_shift PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_nodma_shift ${PIOLIB_LIBRARY})
endif()

if(HAVE_PIOLIB)
    add_executable(test_txonly_dma test_txonly_dma.c)
    target_compile_definitions(test_txonly_dma PRIVATE HAVE_PIOLIB)
    target_include_directories(test_txonly_dma PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_txonly_dma ${PIOLIB_LIBRARY})
endif()

if(HAVE_PIOLIB)
    add_executable(test_txonly_hybrid test_txonly_hybrid.c)
    target_compile_definitions(test_txonly_hybrid PRIVATE HAVE_PIOLIB)
    target_include_directories(test_txonly_hybrid PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_txonly_hybrid ${PIOLIB_LIBRARY})

    add_executable(test_txonly_padded test_txonly_padded.c)
    target_compile_definitions(test_txonly_padded PRIVATE HAVE_PIOLIB)
    target_include_directories(test_txonly_padded PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_txonly_padded ${PIOLIB_LIBRARY})

    add_executable(test_txonly_diag test_txonly_diag.c)
    target_compile_definitions(test_txonly_diag PRIVATE HAVE_PIOLIB)
    target_include_directories(test_txonly_diag PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_txonly_diag ${PIOLIB_LIBRARY})

    add_executable(test_txonly_hybrid2 test_txonly_hybrid2.c)
    target_compile_definitions(test_txonly_hybrid2 PRIVATE HAVE_PIOLIB)
    target_include_directories(test_txonly_hybrid2 PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_txonly_hybrid2 ${PIOLIB_LIBRARY})

    add_executable(test_dma_sweep test_dma_sweep.c)
    target_compile_definitions(test_dma_sweep PRIVATE HAVE_PIOLIB)
    target_include_directories(test_dma_sweep PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_dma_sweep ${PIOLIB_LIBRARY})

    add_executable(test_streaming test_streaming.c)
    target_compile_definitions(test_streaming PRIVATE HAVE_PIOLIB)
    target_include_directories(test_streaming PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_streaming ${PIOLIB_LIBRARY} pthread)

    add_executable(test_fifo_write test_fifo_write.c)
    target_compile_definitions(test_fifo_write PRIVATE HAVE_PIOLIB)
    target_include_directories(test_fifo_write PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_fifo_write ${PIOLIB_LIBRARY})

    add_executable(test_txonly_minimal test_txonly_minimal.c)
    target_compile_definitions(test_txonly_minimal PRIVATE HAVE_PIOLIB)
    target_include_directories(test_txonly_minimal PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_txonly_minimal ${PIOLIB_LIBRARY})

    add_executable(test_concurrent_dma test_concurrent_dma.c)
    target_compile_definitions(test_concurrent_dma PRIVATE HAVE_PIOLIB)
    target_include_directories(test_concurrent_dma PRIVATE
        ${PIOLIB_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_link_libraries(test_concurrent_dma ${PIOLIB_LIBRARY} pthread)
endif()

# Hardware tests are NOT added to CTest (they need real hardware).
# Run them manually:
#   sudo ./test_pio_loopback
#   sudo ./test_gpio_loopback
#   sudo ./test_idcode
#   sudo ./bench_dma_loopback
#   sudo ./bench_throughput
