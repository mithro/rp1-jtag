
# Tests

# PIO simulator tests (no hardware needed)
add_subdirectory(pio_sim)

# Unit tests with mock backend (no hardware needed)
add_subdirectory(unit)

# Hardware integration tests (RPi 5 + optional FPGA)
# These always build (stub functions when PIOLib is unavailable),
# but only run on RPi 5 hardware.
add_subdirectory(hardware)
