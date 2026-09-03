// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2025 Tim Ansell <me@mith.ro>
 *
 * rp1PioJtag.cpp - openFPGALoader cable driver for RP1 PIO JTAG
 *
 * Maps openFPGALoader's JtagInterface methods to librp1jtag calls.
 *
 * Style: PascalCase class, camelCase methods, _prefix private members,
 * tabs for indentation -- matching openFPGALoader conventions.
 */

#include "rp1PioJtag.hpp"

extern "C" {
#include "rp1_jtag.h"
}

#include <cstring>
#include <cstdlib>
#include <iostream>

Rp1PioJtag::Rp1PioJtag(const jtag_pins_conf_t *pin_conf,
		const std::string &dev, uint32_t clkHZ, int8_t verbose)
	: _jtag(nullptr), _verbose(verbose > 1)
{
	(void)dev;  /* unused -- no device path needed for PIO */

	/* Default pins: NeTV2 wiring (TCK=4, TMS=17, TDI=27, TDO=22).
	 * Override with --pins TDI:TDO:TCK:TMS (e.g. --pins 27:22:4:17).
	 */
	const int def_tck = 4, def_tms = 17, def_tdi = 27, def_tdo = 22;

	rp1_jtag_pins_t pins;
	if (!pin_conf ||
	    (pin_conf->tck_pin == 0 && pin_conf->tms_pin == 0 &&
	     pin_conf->tdi_pin == 0 && pin_conf->tdo_pin == 0)) {
		pins = {def_tck, def_tms, def_tdi, def_tdo, -1, -1};
		std::cerr << "rp1pio: using default pins"
			<< " TCK=" << def_tck << " TMS=" << def_tms
			<< " TDI=" << def_tdi << " TDO=" << def_tdo
			<< std::endl;
	} else {
		pins = {
			.tck = pin_conf->tck_pin,
			.tms = pin_conf->tms_pin,
			.tdi = pin_conf->tdi_pin,
			.tdo = pin_conf->tdo_pin,
			.srst = -1,
			.trst = -1
		};
	};

	if (_verbose) {
		std::cerr << "rp1pio: TCK=" << (int)pins.tck
			<< " TMS=" << (int)pins.tms
			<< " TDI=" << (int)pins.tdi
			<< " TDO=" << (int)pins.tdo << std::endl;
	}

	_jtag = rp1_jtag_init(&pins);
	if (!_jtag) {
		std::cerr << "rp1pio: failed to initialize RP1 PIO JTAG" << std::endl;
		std::cerr << "rp1pio: check /dev/pio0 exists and you have permission" << std::endl;
		return;
	}

	_clkHZ = clkHZ;
	if (clkHZ > 0)
		rp1_jtag_set_freq(_jtag, clkHZ);

	if (_verbose)
		std::cerr << "rp1pio: initialized at " << clkHZ << " Hz" << std::endl;
}

Rp1PioJtag::~Rp1PioJtag()
{
	if (_jtag) {
		rp1_jtag_close(_jtag);
		_jtag = nullptr;
	}
}

int Rp1PioJtag::setClkFreq(uint32_t clkHZ)
{
	if (!_jtag)
		return -1;

	int rc = rp1_jtag_set_freq(_jtag, clkHZ);
	if (rc == 0)
		_clkHZ = clkHZ;
	return rc;
}

int Rp1PioJtag::writeTMS(const uint8_t *tms, uint32_t len,
		bool /* flush_buffer */, const uint8_t tdi)
{
	if (!_jtag || !tms || len == 0)
		return -1;

	/* Build constant TDI vector with the given tdi value */
	uint32_t bytes = (len + 7) / 8;
	uint8_t *tdi_vec = static_cast<uint8_t *>(malloc(bytes));
	if (!tdi_vec)
		return -1;
	memset(tdi_vec, tdi ? 0xFF : 0x00, bytes);

	int rc = rp1_jtag_shift(_jtag, len, tms, tdi_vec, nullptr);
	free(tdi_vec);
	return rc < 0 ? rc : len;
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
	return rc < 0 ? rc : len;
}

bool Rp1PioJtag::writeTMSTDI(const uint8_t *tms, const uint8_t *tdi,
		uint8_t *tdo, uint32_t len)
{
	if (!_jtag || !tms || !tdi || len == 0)
		return false;

	int rc = rp1_jtag_shift(_jtag, len, tms, tdi, tdo);
	return rc == 0;
}

int Rp1PioJtag::toggleClk(uint8_t tms, uint8_t tdi, uint32_t clk_len)
{
	if (!_jtag || clk_len == 0)
		return -1;

	int rc = rp1_jtag_toggle_clk(_jtag, clk_len, tms != 0, tdi != 0);
	return rc < 0 ? rc : clk_len;
}
