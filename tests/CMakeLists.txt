
# Tests

# PIO simulator tests (no hardware needed)
add_subdirectory(pio_sim)

# Unit tests with mock backend (no hardware needed)
add_subdirectory(unit)

# Hardware integration tests (RPi 5 + optional FPGA)
if(HAVE_PIOLIB)
    add_subdirectory(hardware)
endif()
