// ------------------------------------------------------------------ //
// This file is hand-written to match pioasm output for jtag_stream.pio //
// ------------------------------------------------------------------ //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ----------- //
// jtag_stream //
// ----------- //

#define jtag_stream_wrap_target 0
#define jtag_stream_wrap 1
#define jtag_stream_pio_version 0

static const uint16_t jtag_stream_program_instructions[] = {
            //     .wrap_target
    0x6001, //  0: out    pins, 1         side 0     ; TDI, TCK LOW
    0x5001, //  1: in     pins, 1         side 1     ; TDO, TCK HIGH
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program jtag_stream_program = {
    .instructions = jtag_stream_program_instructions,
    .length = 2,
    .origin = -1,
    .pio_version = jtag_stream_pio_version,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = 0x0
#endif
};

static inline pio_sm_config jtag_stream_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + jtag_stream_wrap_target, offset + jtag_stream_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}
#endif
