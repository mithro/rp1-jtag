// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * rp1_pio_jtag.c - OpenOCD adapter driver for RP1 PIO JTAG
 *
 * Copyright (C) 2025 Tim Ansell <me@mith.ro>
 *
 * Uses librp1jtag for high-speed JTAG via the Raspberry Pi 5's RP1 PIO
 * subsystem.  This is a non-bitbang driver: the PIO engine shifts data
 * autonomously while the host supplies TDI/TMS vectors and collects TDO,
 * bypassing the per-bit PCIe round-trip that makes linuxgpiod/sysfsgpio
 * slow on RPi 5.
 *
 * Style: C, snake_case, tab indent, LOG_INFO/LOG_ERROR, ERROR_OK returns
 * (matching OpenOCD conventions).
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <transport/transport.h>
#include <helper/bits.h>
#include <helper/log.h>
#include <helper/time_support.h>

#include <rp1_jtag.h>

#include <stdlib.h>
#include <string.h>

/* Forward declarations */
static int rp1_pio_jtag_reset_handler(int trst, int srst);

/* ---- State ---- */

static rp1_jtag_t *rp1_dev;
static int speed_khz;

static int tck_gpio = -1;
static int tms_gpio = -1;
static int tdi_gpio = -1;
static int tdo_gpio = -1;
static int srst_gpio = -1;
static int trst_gpio = -1;

/* ---- TAP state helpers ---- */

/*
 * Build a TMS bit-vector that walks from cur_state to goal_state using
 * the standard shortest-path lookup.  Returns the number of TMS bits.
 */
static int tms_seq_for_state(enum tap_state cur, enum tap_state goal,
			     uint8_t *tms_buf, int max_bits)
{
	int len = tap_get_tms_path_len(cur, goal);
	uint32_t tms_bits = tap_get_tms_path(cur, goal);

	if (len > max_bits) {
		LOG_ERROR("rp1_pio_jtag: TMS path too long (%d > %d)",
			  len, max_bits);
		return -1;
	}

	memset(tms_buf, 0, DIV_ROUND_UP(len, 8));
	for (int i = 0; i < len; i++) {
		if (tms_bits & (1u << i))
			tms_buf[i / 8] |= (1u << (i % 8));
	}
	return len;
}

/*
 * Shift a short TMS sequence (with TDI=0) and update the TAP state.
 */
static int rp1_pio_state_move(enum tap_state goal)
{
	uint8_t tms_buf[2];
	uint8_t tdi_buf[2] = {0, 0};
	enum tap_state cur = tap_get_state();

	if (cur == goal)
		return ERROR_OK;

	int len = tms_seq_for_state(cur, goal, tms_buf, 16);
	if (len < 0)
		return ERROR_FAIL;

	int ret = rp1_jtag_shift(rp1_dev, len, tms_buf, tdi_buf, NULL);
	if (ret < 0) {
		LOG_ERROR("rp1_pio_jtag: state_move shift failed (%d)", ret);
		return ERROR_FAIL;
	}

	tap_set_state(goal);
	return ERROR_OK;
}

/* ---- Command handlers ---- */

static int rp1_pio_handle_scan(struct scan_command *cmd)
{
	uint8_t *tdi_buf = NULL;
	int scan_bits;
	int retval;

	/* Navigate to Shift-IR or Shift-DR */
	enum tap_state shift_state = cmd->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT;
	retval = rp1_pio_state_move(shift_state);
	if (retval != ERROR_OK)
		return retval;

	/* Let OpenOCD pack all scan fields into a flat TDI buffer */
	scan_bits = jtag_build_buffer(cmd, &tdi_buf);
	if (scan_bits == 0) {
		free(tdi_buf);
		return ERROR_OK;
	}

	int num_bytes = DIV_ROUND_UP(scan_bits, 8);

	/* Build TMS vector: all 0s, last bit = 1 to exit Shift state */
	uint8_t *tms = calloc(num_bytes, 1);
	if (!tms) {
		free(tdi_buf);
		return ERROR_FAIL;
	}
	tms[(scan_bits - 1) / 8] |= (1u << ((scan_bits - 1) % 8));

	/* TDO capture buffer */
	uint8_t *tdo = calloc(num_bytes, 1);
	if (!tdo) {
		free(tms);
		free(tdi_buf);
		return ERROR_FAIL;
	}

	/* Bulk shift via librp1jtag */
	int ret = rp1_jtag_shift(rp1_dev, scan_bits, tms, tdi_buf, tdo);
	if (ret < 0) {
		LOG_ERROR("rp1_pio_jtag: scan shift failed (%d)", ret);
		free(tdo);
		free(tms);
		free(tdi_buf);
		return ERROR_FAIL;
	}

	/* After shifting with TMS=1 on last bit, TAP is in Exit1-xR */
	enum tap_state exit_state = cmd->ir_scan ? TAP_IREXIT1 : TAP_DREXIT1;
	tap_set_state(exit_state);

	/* Copy TDO data back through OpenOCD's scan field machinery */
	retval = jtag_read_buffer(tdo, cmd);

	free(tdo);
	free(tms);
	free(tdi_buf);

	if (retval != ERROR_OK)
		return retval;

	/* Move to the end state requested by the command */
	if (tap_get_state() != cmd->end_state)
		retval = rp1_pio_state_move(cmd->end_state);

	return retval;
}

static int rp1_pio_handle_runtest(unsigned int num_cycles,
				  enum tap_state end_state)
{
	int retval;

	/* Move to Run-Test/Idle first */
	retval = rp1_pio_state_move(TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	/* Clock with TMS=0, TDI=0 in Run-Test/Idle */
	if (num_cycles > 0) {
		int ret = rp1_jtag_toggle_clk(rp1_dev, num_cycles, false, false);
		if (ret < 0) {
			LOG_ERROR("rp1_pio_jtag: runtest toggle_clk failed (%d)", ret);
			return ERROR_FAIL;
		}
	}

	/* Move to end state if different */
	if (end_state != TAP_IDLE)
		retval = rp1_pio_state_move(end_state);

	return retval;
}

static int rp1_pio_handle_stableclocks(unsigned int num_cycles)
{
	/*
	 * Clock in the current stable state.  TMS stays the same as the
	 * value that keeps us in this state.
	 */
	enum tap_state state = tap_get_state();
	bool tms_value;

	switch (state) {
	case TAP_IDLE:
	case TAP_DRPAUSE:
	case TAP_IRPAUSE:
		tms_value = false;
		break;
	case TAP_RESET:
		tms_value = true;
		break;
	default:
		LOG_ERROR("rp1_pio_jtag: stableclocks in non-stable state %s",
			  tap_state_name(state));
		return ERROR_FAIL;
	}

	if (num_cycles > 0) {
		int ret = rp1_jtag_toggle_clk(rp1_dev, num_cycles,
					       tms_value, false);
		if (ret < 0) {
			LOG_ERROR("rp1_pio_jtag: stableclocks failed (%d)", ret);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int rp1_pio_handle_pathmove(struct pathmove_command *cmd)
{
	/*
	 * Walk an explicit sequence of TAP states.  Each transition is a
	 * single TCK with the appropriate TMS value.
	 */
	int num_states = cmd->num_states;

	if (num_states == 0)
		return ERROR_OK;

	int num_bytes = DIV_ROUND_UP(num_states, 8);
	uint8_t *tms = calloc(num_bytes, 1);
	uint8_t *tdi = calloc(num_bytes, 1);
	if (!tms || !tdi) {
		free(tms);
		free(tdi);
		return ERROR_FAIL;
	}

	enum tap_state cur = tap_get_state();

	for (int i = 0; i < num_states; i++) {
		enum tap_state next = cmd->path[i];

		if (tap_state_transition(cur, false) == next) {
			/* TMS=0 transitions to next */
		} else if (tap_state_transition(cur, true) == next) {
			tms[i / 8] |= (1u << (i % 8));
		} else {
			LOG_ERROR("rp1_pio_jtag: invalid path %s -> %s",
				  tap_state_name(cur), tap_state_name(next));
			free(tms);
			free(tdi);
			return ERROR_FAIL;
		}

		cur = next;
	}

	int ret = rp1_jtag_shift(rp1_dev, num_states, tms, tdi, NULL);
	free(tms);
	free(tdi);

	if (ret < 0) {
		LOG_ERROR("rp1_pio_jtag: pathmove shift failed (%d)", ret);
		return ERROR_FAIL;
	}

	tap_set_state(cur);
	return ERROR_OK;
}

static int rp1_pio_handle_tms(struct tms_command *cmd)
{
	int num_bits = cmd->num_bits;
	int num_bytes = DIV_ROUND_UP(num_bits, 8);

	uint8_t *tdi = calloc(num_bytes, 1);
	if (!tdi)
		return ERROR_FAIL;

	int ret = rp1_jtag_shift(rp1_dev, num_bits, cmd->bits, tdi, NULL);
	free(tdi);

	if (ret < 0) {
		LOG_ERROR("rp1_pio_jtag: TMS shift failed (%d)", ret);
		return ERROR_FAIL;
	}

	/*
	 * We must update the TAP state by replaying the TMS bits.
	 * Walk through the TMS bits and let OpenOCD track state.
	 */
	enum tap_state cur = tap_get_state();
	for (int i = 0; i < num_bits; i++) {
		bool tms_bit = (cmd->bits[i / 8] >> (i % 8)) & 1;
		cur = tap_state_transition(cur, tms_bit);
	}
	tap_set_state(cur);

	return ERROR_OK;
}

/* ---- execute_queue ---- */

static int rp1_pio_jtag_execute_queue(struct jtag_command *cmd_queue)
{
	struct jtag_command *cmd;
	int retval = ERROR_OK;

	for (cmd = cmd_queue; retval == ERROR_OK && cmd; cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RESET:
			retval = rp1_pio_jtag_reset_handler(
				cmd->cmd.reset->trst,
				cmd->cmd.reset->srst);
			break;
		case JTAG_RUNTEST:
			retval = rp1_pio_handle_runtest(
				cmd->cmd.runtest->num_cycles,
				cmd->cmd.runtest->end_state);
			break;
		case JTAG_STABLECLOCKS:
			retval = rp1_pio_handle_stableclocks(
				cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			retval = rp1_pio_state_move(
				cmd->cmd.statemove->end_state);
			break;
		case JTAG_PATHMOVE:
			retval = rp1_pio_handle_pathmove(
				cmd->cmd.pathmove);
			break;
		case JTAG_TMS:
			retval = rp1_pio_handle_tms(cmd->cmd.tms);
			break;
		case JTAG_SLEEP:
			jtag_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			retval = rp1_pio_handle_scan(cmd->cmd.scan);
			break;
		default:
			LOG_ERROR("rp1_pio_jtag: unknown command type 0x%X",
				  cmd->type);
			retval = ERROR_FAIL;
			break;
		}
	}
	return retval;
}

/* ---- Reset ---- */

static int rp1_pio_jtag_reset_handler(int trst, int srst)
{
	/*
	 * OpenOCD convention: 1 = assert, 0 = deassert
	 * librp1jtag convention: 0 = assert (active-low), 1 = deassert
	 * Invert:
	 */
	int ret = rp1_jtag_reset(rp1_dev,
				 srst ? 0 : 1,
				 trst ? 0 : 1);
	if (ret < 0) {
		LOG_ERROR("rp1_pio_jtag: reset failed (%d)", ret);
		return ERROR_FAIL;
	}

	if (trst)
		tap_set_state(TAP_RESET);

	return ERROR_OK;
}

/* ---- Speed ---- */

static int rp1_pio_jtag_khz(int khz, int *jtag_speed)
{
	if (khz == 0) {
		LOG_ERROR("rp1_pio_jtag: RTCK not supported");
		return ERROR_FAIL;
	}
	*jtag_speed = khz;
	return ERROR_OK;
}

static int rp1_pio_jtag_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static int rp1_pio_jtag_speed(int speed)
{
	speed_khz = speed;
	if (rp1_dev) {
		int ret = rp1_jtag_set_freq(rp1_dev, speed * 1000);
		if (ret < 0) {
			LOG_ERROR("rp1_pio_jtag: failed to set freq %d kHz",
				  speed);
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

/* ---- Lifecycle ---- */

static int rp1_pio_jtag_init(void)
{
	LOG_INFO("rp1_pio_jtag: initializing RP1 PIO JTAG driver");

	if (tck_gpio < 0 || tms_gpio < 0 || tdi_gpio < 0 || tdo_gpio < 0) {
		LOG_ERROR("rp1_pio_jtag: GPIO pins not configured. "
			  "Use: rp1_pio_jtag jtag_nums <tck> <tms> <tdi> <tdo>");
		return ERROR_JTAG_INIT_FAILED;
	}

	rp1_jtag_pins_t pins = {
		.tck  = tck_gpio,
		.tms  = tms_gpio,
		.tdi  = tdi_gpio,
		.tdo  = tdo_gpio,
		.srst = srst_gpio,
		.trst = trst_gpio,
	};

	LOG_INFO("rp1_pio_jtag: TCK=%d TMS=%d TDI=%d TDO=%d SRST=%d TRST=%d",
		 tck_gpio, tms_gpio, tdi_gpio, tdo_gpio, srst_gpio, trst_gpio);

	rp1_dev = rp1_jtag_init(&pins);
	if (!rp1_dev) {
		LOG_ERROR("rp1_pio_jtag: failed to initialize librp1jtag. "
			  "Check /dev/pio0 exists and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (speed_khz > 0) {
		rp1_jtag_set_freq(rp1_dev, speed_khz * 1000);
		LOG_INFO("rp1_pio_jtag: clock set to %d kHz", speed_khz);
	}

	return ERROR_OK;
}

static int rp1_pio_jtag_quit(void)
{
	if (rp1_dev) {
		rp1_jtag_close(rp1_dev);
		rp1_dev = NULL;
	}
	LOG_INFO("rp1_pio_jtag: driver closed");
	return ERROR_OK;
}

/* ---- Config commands ---- */

COMMAND_HANDLER(rp1_pio_jtag_handle_jtag_nums)
{
	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], tms_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], tdi_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], tdo_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
		      "rp1_pio_jtag GPIO config: tck=%d tms=%d tdi=%d tdo=%d",
		      tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(rp1_pio_jtag_handle_srst_num)
{
	if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD, "rp1_pio_jtag SRST GPIO: %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(rp1_pio_jtag_handle_trst_num)
{
	if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD, "rp1_pio_jtag TRST GPIO: %d", trst_gpio);
	return ERROR_OK;
}

static const struct command_registration rp1_pio_jtag_subcommand_handlers[] = {
	{
		.name = "jtag_nums",
		.handler = rp1_pio_jtag_handle_jtag_nums,
		.mode = COMMAND_CONFIG,
		.help = "Set GPIO numbers for JTAG signals (BCM numbering).",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "srst_num",
		.handler = rp1_pio_jtag_handle_srst_num,
		.mode = COMMAND_CONFIG,
		.help = "Set GPIO number for SRST.",
		.usage = "[gpio]",
	},
	{
		.name = "trst_num",
		.handler = rp1_pio_jtag_handle_trst_num,
		.mode = COMMAND_CONFIG,
		.help = "Set GPIO number for TRST.",
		.usage = "[gpio]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration rp1_pio_jtag_command_handlers[] = {
	{
		.name = "rp1_pio_jtag",
		.mode = COMMAND_ANY,
		.help = "RP1 PIO JTAG adapter commands",
		.chain = rp1_pio_jtag_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

/* ---- Driver registration ---- */

static struct jtag_interface rp1_pio_jtag_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = rp1_pio_jtag_execute_queue,
};

struct adapter_driver rp1_pio_jtag_adapter_driver = {
	.name = "rp1_pio_jtag",
	.transport_ids = TRANSPORT_JTAG,
	.transport_preferred_id = TRANSPORT_JTAG,
	.commands = rp1_pio_jtag_command_handlers,

	.init = rp1_pio_jtag_init,
	.quit = rp1_pio_jtag_quit,
	.reset = rp1_pio_jtag_reset_handler,
	.speed = rp1_pio_jtag_speed,
	.khz = rp1_pio_jtag_khz,
	.speed_div = rp1_pio_jtag_speed_div,

	.jtag_ops = &rp1_pio_jtag_interface,
};
