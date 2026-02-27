
# Unit tests (no hardware needed, pure calculation tests)

# Data packing (bits_to_words / words_to_bits roundtrip)
add_executable(test_data_packing test_data_packing.c)
target_link_libraries(test_data_packing rp1jtag_static)
target_include_directories(test_data_packing PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/src
)
add_test(NAME unit_data_packing COMMAND test_data_packing)

# DMA buffer layout (word counts, SM0/SM1 TX format, partial-word fix)
add_executable(test_dma_buffer_packing test_dma_buffer_packing.c)
target_link_libraries(test_dma_buffer_packing rp1jtag_static)
target_include_directories(test_dma_buffer_packing PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/src
)
add_test(NAME unit_dma_buffer_packing COMMAND test_dma_buffer_packing)
