/*
 * rp1PioJtag.cpp - openFPGALoader cable driver for RP1 PIO JTAG
 *
 * Maps openFPGALoader's JtagInterface methods to librp1jtag calls.
 *
 * Style: PascalCase class, camelCase methods, _prefix private members,
 * spaces not tabs -- matching openFPGALoader conventions.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1PioJtag.hpp"

extern "C" {
#include "rp1_jtag.h"
}

#include <cstring>
#include <cstdlib>

Rp1PioJtag::Rp1PioJtag(int tck, int tms, int tdi, int tdo,
                         uint32_t freq_hz)
    : _jtag(nullptr), _freq_hz(freq_hz)
{
    rp1_jtag_pins_t pins = {
        .tck = tck, .tms = tms, .tdi = tdi, .tdo = tdo,
        .srst = -1, .trst = -1
    };

    _jtag = rp1_jtag_init(&pins);
    if (_jtag && freq_hz > 0)
        rp1_jtag_set_freq(_jtag, freq_hz);
}

Rp1PioJtag::~Rp1PioJtag()
{
    if (_jtag) {
        rp1_jtag_close(_jtag);
        _jtag = nullptr;
    }
}

int Rp1PioJtag::writeTMS(const uint8_t *tms, uint32_t len,
                          bool /* flush_buffer */)
{
    if (!_jtag || !tms || len == 0)
        return -1;

    /* TDI held high per JTAG convention during TMS transitions */
    uint32_t bytes = (len + 7) / 8;
    uint8_t *tdi = static_cast<uint8_t *>(malloc(bytes));
    if (!tdi)
        return -1;
    memset(tdi, 0xFF, bytes);

    int rc = rp1_jtag_shift(_jtag, len, tms, tdi, nullptr);
    free(tdi);
    return rc;
}

int Rp1PioJtag::writeTDI(const uint8_t *tx, uint8_t *rx,
                          uint32_t len, bool end)
{
    if (!_jtag || !tx || len == 0)
        return -1;

    /* Build TMS vector: all 0s, last bit = end (1 to exit Shift-DR/IR) */
    uint32_t bytes = (len + 7) / 8;
    uint8_t *tms = static_cast<uint8_t *>(calloc(bytes, 1));
    if (!tms)
        return -1;

    if (end && len > 0) {
        /* Set last bit of TMS to 1 */
        tms[(len - 1) / 8] |= (1u << ((len - 1) % 8));
    }

    int rc = rp1_jtag_shift(_jtag, len, tms, tx, rx);
    free(tms);
    return rc;
}

int Rp1PioJtag::writeTMSTDI(const uint8_t *tms, const uint8_t *tx,
                              uint8_t *rx, uint32_t len)
{
    if (!_jtag || !tms || !tx || len == 0)
        return -1;

    /* Direct passthrough to library */
    return rp1_jtag_shift(_jtag, len, tms, tx, rx);
}

int Rp1PioJtag::toggleClk(uint32_t nb)
{
    if (!_jtag || nb == 0)
        return -1;

    /* TMS=0 (stay in current state), TDI=1 (convention) */
    return rp1_jtag_toggle_clk(_jtag, nb, false, true);
}

int Rp1PioJtag::setClkFreq(uint32_t freq_hz)
{
    if (!_jtag)
        return -1;

    int rc = rp1_jtag_set_freq(_jtag, freq_hz);
    if (rc == 0)
        _freq_hz = freq_hz;
    return rc;
}

uint32_t Rp1PioJtag::getClkFreq()
{
    return _freq_hz;
}
