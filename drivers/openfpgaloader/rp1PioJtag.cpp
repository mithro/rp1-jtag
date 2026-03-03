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
	: _jtag(nullptr), _verbose(verbose > 1), _buf_word(0), _buf_bits(0)
{
	(void)dev;  /* unused -- no device path needed for PIO */

	if (!pin_conf) {
		std::cerr << "rp1pio: pin configuration required (--pins tck=N:tms=N:tdi=N:tdo=N)" << std::endl;
		return;
	}

	rp1_jtag_pins_t pins = {
		.tck = pin_conf->tck_pin,
		.tms = pin_conf->tms_pin,
		.tdi = pin_conf->tdi_pin,
		.tdo = pin_conf->tdo_pin,
		.srst = -1,
		.trst = -1
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
		flushBuffer();
		rp1_jtag_close(_jtag);
		_jtag = nullptr;
	}
}

int Rp1PioJtag::flushBuffer()
{
	if (_buf_bits == 0)
		return 0;
	if (!_jtag)
		return -1;

	/* Send buffered bits with TMS=0 (stay in Shift-DR/IR) */
	uint8_t tms[4] = {0};
	uint8_t tdi[4];
	memcpy(tdi, &_buf_word, sizeof(_buf_word));

	int rc = rp1_jtag_shift(_jtag, _buf_bits, tms, tdi, nullptr);
	if (rc == 0) {
		_buf_word = 0;
		_buf_bits = 0;
	}
	return rc;
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

	int frc = flushBuffer();
	if (frc < 0)
		return frc;

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

	/*
	 * Path 1: end=true or rx!=NULL — must flush buffer first, then
	 * send this segment unbuffered (TMS last-bit and/or RX capture
	 * need exact bit boundaries).
	 */
	if (end || rx) {
		int rc = flushBuffer();
		if (rc < 0)
			return rc;

		uint32_t bytes = (len + 7) / 8;
		uint8_t *tms = static_cast<uint8_t *>(calloc(bytes, 1));
		if (!tms)
			return -1;
		if (end && len > 0)
			tms[(len - 1) / 8] |= (1u << ((len - 1) % 8));

		rc = rp1_jtag_shift(_jtag, len, tms, tx, rx);
		free(tms);
		return rc < 0 ? rc : len;
	}

	/*
	 * Paths 2 & 3: end=false, rx==NULL — buffer for 32-bit alignment.
	 *
	 * Combine any buffered tail bits with new data, send complete
	 * 32-bit words via the library (hits streaming DMA path for large
	 * transfers), and buffer the remaining 0-31 tail bits.
	 */
	uint32_t total_bits = _buf_bits + len;

	/* If we don't even have 32 bits total, just buffer everything */
	if (total_bits < 32) {
		/* Pack new bits after existing buffer bits */
		uint8_t shift = _buf_bits;
		for (uint32_t i = 0; i < len; i++) {
			uint32_t bit = (tx[i / 8] >> (i % 8)) & 1u;
			_buf_word |= (bit << shift);
			shift++;
		}
		_buf_bits = static_cast<uint8_t>(total_bits);
		return len;
	}

	uint32_t send_bits = (total_bits / 32) * 32;
	uint32_t tail_bits = total_bits % 32;
	uint32_t send_bytes = send_bits / 8;

	const uint8_t *send_tx;
	uint8_t *merged = nullptr;

	if (_buf_bits == 0) {
		/*
		 * Path 2: Zero-copy fast path — no buffered bits, pass tx
		 * directly for the full-word portion.
		 */
		send_tx = tx;
	} else {
		/*
		 * Path 3: Merge buffered bits with new tx data.
		 *
		 * Since openFPGALoader sends byte-aligned lengths,
		 * _buf_bits is always 0/8/16/24. The byte-aligned path
		 * uses memcpy (no bit shifting). The general path exists
		 * for robustness but won't be exercised in practice.
		 */
		merged = static_cast<uint8_t *>(malloc(send_bytes +
			(tail_bits ? 4 : 0)));
		if (!merged)
			return -1;

		uint8_t buf_bytes = _buf_bits / 8;

		if ((_buf_bits % 8) == 0) {
			/* Byte-aligned fast path: memcpy */
			memcpy(merged, &_buf_word, buf_bytes);
			memcpy(merged + buf_bytes, tx, (len + 7) / 8);
		} else {
			/* General bit-merge path (robustness) */
			memset(merged, 0, send_bytes + (tail_bits ? 4 : 0));
			/* Copy buffered bits */
			for (uint8_t i = 0; i < _buf_bits; i++) {
				uint32_t bit = (_buf_word >> i) & 1u;
				merged[i / 8] |= (bit << (i % 8));
			}
			/* Copy new tx bits after buffer */
			for (uint32_t i = 0; i < len; i++) {
				uint32_t bit = (tx[i / 8] >> (i % 8)) & 1u;
				uint32_t pos = _buf_bits + i;
				merged[pos / 8] |= (bit << (pos % 8));
			}
		}
		send_tx = merged;
	}

	/* Send the 32-bit-aligned portion with TMS=0 */
	uint8_t *tms = static_cast<uint8_t *>(calloc(send_bytes, 1));
	if (!tms) {
		free(merged);
		return -1;
	}

	int rc = rp1_jtag_shift(_jtag, send_bits, tms, send_tx, nullptr);
	free(tms);

	if (rc < 0) {
		free(merged);
		return rc;
	}

	/* Buffer the tail bits (0-31 remaining) */
	_buf_word = 0;
	_buf_bits = static_cast<uint8_t>(tail_bits);
	if (tail_bits > 0) {
		/*
		 * Extract tail bits from the end of the combined data.
		 * The tail starts at bit offset send_bits in the combined
		 * stream. For the merged case, read from merged[]. For the
		 * zero-copy case, read from tx[] at the appropriate offset.
		 */
		if (merged) {
			uint32_t byte_off = send_bits / 8;
			memcpy(&_buf_word, merged + byte_off, (tail_bits + 7) / 8);
		} else {
			/* Zero-copy: tail is at the end of tx */
			uint32_t bit_off = send_bits;  /* offset into tx */
			uint32_t byte_off = bit_off / 8;
			uint32_t bit_shift = bit_off % 8;
			uint32_t tail_bytes = (tail_bits + 7) / 8;

			if (bit_shift == 0) {
				memcpy(&_buf_word, tx + byte_off, tail_bytes);
			} else {
				/* Not byte-aligned in tx — shift bits */
				_buf_word = 0;
				for (uint32_t i = 0; i < tail_bits; i++) {
					uint32_t src = bit_off + i;
					uint32_t bit = (tx[src / 8] >> (src % 8)) & 1u;
					_buf_word |= (bit << i);
				}
			}
		}
		/* Mask off any excess bits */
		_buf_word &= (tail_bits == 32) ? 0xFFFFFFFF :
			((1u << tail_bits) - 1);
	}

	free(merged);
	return rc < 0 ? rc : len;
}

bool Rp1PioJtag::writeTMSTDI(const uint8_t *tms, const uint8_t *tdi,
		uint8_t *tdo, uint32_t len)
{
	if (!_jtag || !tms || !tdi || len == 0)
		return false;

	if (flushBuffer() < 0)
		return false;

	int rc = rp1_jtag_shift(_jtag, len, tms, tdi, tdo);
	return rc == 0;
}

int Rp1PioJtag::toggleClk(uint8_t tms, uint8_t tdi, uint32_t clk_len)
{
	if (!_jtag || clk_len == 0)
		return -1;

	int frc = flushBuffer();
	if (frc < 0)
		return frc;

	int rc = rp1_jtag_toggle_clk(_jtag, clk_len, tms != 0, tdi != 0);
	return rc < 0 ? rc : clk_len;
}

int Rp1PioJtag::flush()
{
	return flushBuffer();
}
