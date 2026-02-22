
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

# Hardware tests are NOT added to CTest (they need real hardware).
# Run them manually:
#   sudo ./test_pio_loopback
#   sudo ./test_gpio_loopback
#   sudo ./test_idcode
