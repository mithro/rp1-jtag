/*
 * rp1PioJtag.hpp - openFPGALoader cable driver for RP1 PIO JTAG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <string>
#include <cstdint>

/* Forward declaration -- avoids exposing C library internals */
struct rp1_jtag;

class Rp1PioJtag {
 public:
    Rp1PioJtag(int tck, int tms, int tdi, int tdo,
               uint32_t freq_hz);
    ~Rp1PioJtag();

    /* JtagInterface methods */
    int writeTMS(const uint8_t *tms, uint32_t len, bool flush_buffer);
    int writeTDI(const uint8_t *tx, uint8_t *rx, uint32_t len, bool end);
    int writeTMSTDI(const uint8_t *tms, const uint8_t *tx, uint8_t *rx,
                    uint32_t len);
    int toggleClk(uint32_t nb);
    int setClkFreq(uint32_t freq_hz);
    uint32_t getClkFreq();
    bool isOK() const { return _jtag != nullptr; }

 private:
    struct rp1_jtag *_jtag;
    uint32_t _freq_hz;
};
