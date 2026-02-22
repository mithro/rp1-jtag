// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2025 Tim Ansell <me@mith.ro>
 *
 * rp1PioJtag.hpp - openFPGALoader cable driver for RP1 PIO JTAG
 *
 * Uses librp1jtag for high-speed JTAG via the Raspberry Pi 5's RP1 PIO
 * subsystem, bypassing the PCIe latency that makes sysfsgpio/linuxgpiod slow.
 */

#ifndef SRC_RP1PIOJTAG_HPP_
#define SRC_RP1PIOJTAG_HPP_

#include "jtagInterface.hpp"
#include "board.hpp"

#include <string>
#include <cstdint>

/* Forward declaration -- avoids exposing C library internals */
struct rp1_jtag;

class Rp1PioJtag : public JtagInterface {
 public:
	Rp1PioJtag(const jtag_pins_conf_t *pin_conf,
		const std::string &dev, uint32_t clkHZ, int8_t verbose);
	virtual ~Rp1PioJtag();

	int setClkFreq(uint32_t clkHZ) override;

	int writeTMS(const uint8_t *tms, uint32_t len, bool flush_buffer,
		const uint8_t tdi = 1) override;
	int writeTDI(const uint8_t *tx, uint8_t *rx, uint32_t len,
		bool end) override;
	bool writeTMSTDI(const uint8_t *tms, const uint8_t *tdi,
		uint8_t *tdo, uint32_t len) override;
	int toggleClk(uint8_t tms, uint8_t tdi, uint32_t clk_len) override;

	int get_buffer_size() override { return 0; }
	bool isFull() override { return false; }
	int flush() override { return 0; }

 private:
	struct rp1_jtag *_jtag;
	int8_t _verbose;
};

#endif  // SRC_RP1PIOJTAG_HPP_
