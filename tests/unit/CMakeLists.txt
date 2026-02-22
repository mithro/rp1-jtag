
# Unit tests (no hardware needed, uses mock PIOLib backend)

# TMS run-length splitting
add_executable(test_tms_splitting test_tms_splitting.c)
target_link_libraries(test_tms_splitting rp1jtag_static)
target_include_directories(test_tms_splitting PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/src
)
add_test(NAME unit_tms_splitting COMMAND test_tms_splitting)

# FIFO interleaving
add_executable(test_fifo_interleave test_fifo_interleave.c)
target_link_libraries(test_fifo_interleave rp1jtag_static)
target_include_directories(test_fifo_interleave PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/src
)
add_test(NAME unit_fifo_interleave COMMAND test_fifo_interleave)

# Data packing
add_executable(test_data_packing test_data_packing.c)
target_link_libraries(test_data_packing rp1jtag_static)
target_include_directories(test_data_packing PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/src
)
add_test(NAME unit_data_packing COMMAND test_data_packing)
