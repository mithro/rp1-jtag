
# PIO simulator and tests

add_library(pio_sim STATIC pio_sim.c)
target_include_directories(pio_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Simulator self-tests
add_executable(test_pio_sim test_pio_sim.c)
target_link_libraries(test_pio_sim pio_sim)
add_test(NAME pio_sim_self_test COMMAND test_pio_sim)

# JTAG program tests (use simulator + generated PIO headers)
add_executable(test_jtag_shift test_jtag_shift.c)
target_link_libraries(test_jtag_shift pio_sim)
add_test(NAME pio_sim_jtag_shift COMMAND test_jtag_shift)

add_executable(test_jtag_tms test_jtag_tms.c)
target_link_libraries(test_jtag_tms pio_sim)
add_test(NAME pio_sim_jtag_tms COMMAND test_jtag_tms)

add_executable(test_bit_ordering test_bit_ordering.c)
target_link_libraries(test_bit_ordering pio_sim)
add_test(NAME pio_sim_bit_ordering COMMAND test_bit_ordering)
