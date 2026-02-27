
# XVC daemon (requires PIOLib for librp1jtag hardware support)
add_executable(rp1-xvc xvc_server.c)
target_link_libraries(rp1-xvc rp1jtag_static)
install(TARGETS rp1-xvc RUNTIME DESTINATION bin)
