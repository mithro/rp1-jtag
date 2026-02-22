/*
 * pio_sim.c - Minimal PIO simulator
 *
 * Implements enough of the RP2040/RP1 PIO instruction set to test
 * our JTAG PIO programs. Supports: JMP, WAIT, IN, OUT, PUSH, PULL,
 * MOV, SET, NOP, side-set, autopush, autopull, wrap.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pio_sim.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ---- PIO instruction encoding ---- */

/* Opcodes (bits [15:13]) */
#define OP_JMP   0  /* 000 */
#define OP_WAIT  1  /* 001 */
#define OP_IN    2  /* 010 */
#define OP_OUT   3  /* 011 */
#define OP_PUSH_PULL 4  /* 100 */
#define OP_MOV   5  /* 101 */
#define OP_IRQ   6  /* 110 */
#define OP_SET   7  /* 111 */

/* JMP conditions */
#define JMP_ALWAYS  0
#define JMP_NOT_X   1  /* !X (X == 0) */
#define JMP_X_DEC   2  /* X-- (X != 0, post-decrement) */
#define JMP_NOT_Y   3  /* !Y */
#define JMP_Y_DEC   4  /* Y-- */
#define JMP_X_NE_Y  5  /* X != Y */
#define JMP_PIN     6  /* PIN */
#define JMP_NOT_OSRE 7 /* !OSRE */

/* IN/OUT/SET/MOV sources/destinations */
#define SRC_PINS    0
#define SRC_X       1
#define SRC_Y       2
#define SRC_NULL    3
#define SRC_ISR     6
#define SRC_OSR     7

/* MOV operations */
#define MOV_OP_NONE    0
#define MOV_OP_INVERT  1
#define MOV_OP_REVERSE 2

/* WAIT sources */
#define WAIT_GPIO  0
#define WAIT_PIN   1
#define WAIT_IRQ   2

/* ---- FIFO operations ---- */

static void fifo_init(pio_sim_fifo_t *f)
{
    f->head = 0;
    f->tail = 0;
    f->count = 0;
}

static bool fifo_push(pio_sim_fifo_t *f, uint32_t word)
{
    if (f->count >= PIO_SIM_FIFO_DEPTH)
        return false;
    f->data[f->tail] = word;
    f->tail = (f->tail + 1) % PIO_SIM_FIFO_DEPTH;
    f->count++;
    return true;
}

static bool fifo_pop(pio_sim_fifo_t *f, uint32_t *word)
{
    if (f->count == 0)
        return false;
    *word = f->data[f->head];
    f->head = (f->head + 1) % PIO_SIM_FIFO_DEPTH;
    f->count--;
    return true;
}

/* ---- Bit manipulation ---- */

static uint32_t bit_reverse(uint32_t v)
{
    v = ((v >> 1) & 0x55555555) | ((v & 0x55555555) << 1);
    v = ((v >> 2) & 0x33333333) | ((v & 0x33333333) << 2);
    v = ((v >> 4) & 0x0F0F0F0F) | ((v & 0x0F0F0F0F) << 4);
    v = ((v >> 8) & 0x00FF00FF) | ((v & 0x00FF00FF) << 8);
    v = (v >> 16) | (v << 16);
    return v;
}

/* ---- Instruction decoding ---- */

static int decode_opcode(uint16_t insn)
{
    return (insn >> 13) & 0x7;
}

static int decode_delay_sideset(uint16_t insn, pio_sim_t *sim,
                                int *delay, int *sideset_val)
{
    int field = (insn >> 8) & 0x1F;  /* 5-bit delay/side-set field */

    if (sim->sideset_count > 0) {
        if (sim->sideset_opt) {
            /* Optional side-set: MSB is enable bit */
            bool has_sideset = (field >> 4) & 1;
            if (has_sideset) {
                *sideset_val = (field >> (4 - sim->sideset_count)) &
                               ((1 << sim->sideset_count) - 1);
            } else {
                *sideset_val = -1;  /* No side-set this instruction */
            }
            *delay = field & ((1 << (4 - sim->sideset_count)) - 1);
        } else {
            /* Non-optional: top N bits are side-set */
            *sideset_val = (field >> (5 - sim->sideset_count)) &
                           ((1 << sim->sideset_count) - 1);
            *delay = field & ((1 << (5 - sim->sideset_count)) - 1);
        }
    } else {
        *delay = field;
        *sideset_val = -1;
    }

    return 0;
}

/* Apply side-set value to GPIO output pins */
static void apply_sideset(pio_sim_t *sim, int sideset_val)
{
    if (sideset_val < 0 || sim->sideset_count == 0)
        return;

    for (int i = 0; i < sim->sideset_count; i++) {
        int pin = (sim->sideset_base + i) % PIO_SIM_MAX_GPIO;
        if (sideset_val & (1 << i))
            sim->gpio_out |= (1u << pin);
        else
            sim->gpio_out &= ~(1u << pin);
    }
}

/* Try autopull: if OSR has shifted out enough bits, pull from TX FIFO */
static bool try_autopull(pio_sim_t *sim)
{
    if (!sim->autopull)
        return true;
    if (sim->osr_shift_count >= sim->autopull_threshold) {
        uint32_t word;
        if (!fifo_pop(&sim->tx_fifo, &word))
            return false;  /* Stall */
        sim->osr = word;
        sim->osr_shift_count = 0;
    }
    return true;
}

/* Try autopush: if ISR has shifted in enough bits, push to RX FIFO */
static bool try_autopush(pio_sim_t *sim)
{
    if (!sim->autopush)
        return true;
    if (sim->isr_shift_count >= sim->autopush_threshold) {
        if (!fifo_push(&sim->rx_fifo, sim->isr))
            return false;  /* Stall */
        sim->isr = 0;
        sim->isr_shift_count = 0;
    }
    return true;
}

/* Read pins for IN instruction */
static uint32_t read_in_source(pio_sim_t *sim, int source, int count)
{
    uint32_t val = 0;
    uint32_t mask = (count == 32) ? 0xFFFFFFFF : ((1u << count) - 1);

    switch (source) {
    case SRC_PINS:
        /* Read from gpio_in at in_base */
        val = (sim->gpio_in >> sim->in_base) & mask;
        break;
    case SRC_X:
        val = sim->x & mask;
        break;
    case SRC_Y:
        val = sim->y & mask;
        break;
    case SRC_NULL:
        val = 0;
        break;
    case SRC_ISR:
        val = sim->isr & mask;
        break;
    case SRC_OSR:
        val = sim->osr & mask;
        break;
    default:
        val = 0;
        break;
    }

    return val;
}

/* ---- Instruction execution ---- */

static bool exec_jmp(pio_sim_t *sim, uint16_t insn)
{
    int condition = (insn >> 5) & 0x7;
    int addr = insn & 0x1F;
    bool take = false;

    switch (condition) {
    case JMP_ALWAYS:
        take = true;
        break;
    case JMP_NOT_X:
        take = (sim->x == 0);
        break;
    case JMP_X_DEC:
        take = (sim->x != 0);
        sim->x--;
        break;
    case JMP_NOT_Y:
        take = (sim->y == 0);
        break;
    case JMP_Y_DEC:
        take = (sim->y != 0);
        sim->y--;
        break;
    case JMP_X_NE_Y:
        take = (sim->x != sim->y);
        break;
    case JMP_PIN:
        /* Read the JMP pin (not implemented, assume false) */
        take = false;
        break;
    case JMP_NOT_OSRE:
        take = (sim->osr_shift_count < 32);
        break;
    }

    if (take)
        sim->pc = addr;
    else
        sim->pc++;

    return true;
}

static bool exec_wait(pio_sim_t *sim, uint16_t insn)
{
    int polarity = (insn >> 7) & 1;
    int source = (insn >> 5) & 0x3;
    int index = insn & 0x1F;
    bool pin_val = false;

    switch (source) {
    case WAIT_GPIO:
        pin_val = (sim->gpio_in >> index) & 1;
        break;
    case WAIT_PIN:
        pin_val = (sim->gpio_in >> (sim->in_base + index)) & 1;
        break;
    case WAIT_IRQ:
        /* IRQ wait not implemented */
        sim->pc++;
        return true;
    }

    if ((pin_val ? 1 : 0) == polarity) {
        sim->pc++;
        return true;
    } else {
        /* Stall -- don't advance PC */
        sim->stalled = true;
        return false;
    }
}

static bool exec_in(pio_sim_t *sim, uint16_t insn)
{
    int source = (insn >> 5) & 0x7;
    int count = insn & 0x1F;
    if (count == 0) count = 32;

    uint32_t data = read_in_source(sim, source, count);

    /* Shift data into ISR */
    if (sim->in_shift_right) {
        /* Shift right (LSB-first): new data enters at MSB side */
        sim->isr >>= count;
        sim->isr |= data << (32 - count);
    } else {
        /* Shift left: new data enters at LSB side */
        sim->isr <<= count;
        sim->isr |= data & ((count == 32) ? 0xFFFFFFFF : ((1u << count) - 1));
    }
    sim->isr_shift_count += count;

    /* Try autopush */
    if (!try_autopush(sim)) {
        sim->stalled = true;
        return false;
    }

    sim->pc++;
    return true;
}

static bool exec_out(pio_sim_t *sim, uint16_t insn)
{
    int dest = (insn >> 5) & 0x7;
    int count = insn & 0x1F;
    if (count == 0) count = 32;

    /* Try autopull before shifting */
    if (!try_autopull(sim)) {
        sim->stalled = true;
        return false;
    }

    /* Shift data out of OSR */
    uint32_t data;
    uint32_t mask = (count == 32) ? 0xFFFFFFFF : ((1u << count) - 1);

    if (sim->out_shift_right) {
        /* Shift right (LSB-first): data comes from LSB */
        data = sim->osr & mask;
        sim->osr >>= count;
    } else {
        /* Shift left: data comes from MSB */
        data = (sim->osr >> (32 - count)) & mask;
        sim->osr <<= count;
    }
    sim->osr_shift_count += count;

    /* Write to destination */
    switch (dest) {
    case SRC_PINS: {
        /* Write to out pins */
        for (int i = 0; i < count && i < sim->out_count; i++) {
            int pin = (sim->out_base + i) % PIO_SIM_MAX_GPIO;
            if (data & (1u << i))
                sim->gpio_out |= (1u << pin);
            else
                sim->gpio_out &= ~(1u << pin);
        }
        break;
    }
    case SRC_X:
        sim->x = data;
        break;
    case SRC_Y:
        sim->y = data;
        break;
    case SRC_NULL:
        /* Discard */
        break;
    case 4: /* PINDIRS */
        for (int i = 0; i < count && i < sim->out_count; i++) {
            int pin = (sim->out_base + i) % PIO_SIM_MAX_GPIO;
            if (data & (1u << i))
                sim->gpio_dir |= (1u << pin);
            else
                sim->gpio_dir &= ~(1u << pin);
        }
        break;
    case 5: /* PC */
        sim->pc = data & 0x1F;
        return true;  /* Don't increment PC */
    case SRC_ISR:
        sim->isr = data;
        sim->isr_shift_count = count;
        break;
    case 7: /* EXEC */
        /* Not implemented */
        break;
    }

    sim->pc++;
    return true;
}

static bool exec_push_pull(pio_sim_t *sim, uint16_t insn)
{
    bool is_pull = (insn >> 7) & 1;
    bool if_flag = (insn >> 6) & 1;  /* IfFull/IfEmpty */
    bool block = (insn >> 5) & 1;

    if (is_pull) {
        /* PULL */
        if (if_flag && sim->osr_shift_count < sim->autopull_threshold) {
            /* IfEmpty: only pull if OSR is depleted */
            sim->pc++;
            return true;
        }
        uint32_t word;
        if (fifo_pop(&sim->tx_fifo, &word)) {
            sim->osr = word;
            sim->osr_shift_count = 0;
            sim->pc++;
            return true;
        } else if (block) {
            sim->stalled = true;
            return false;
        } else {
            /* noblock: copy X to OSR */
            sim->osr = sim->x;
            sim->osr_shift_count = 0;
            sim->pc++;
            return true;
        }
    } else {
        /* PUSH */
        if (if_flag && sim->isr_shift_count < sim->autopush_threshold) {
            /* IfFull: only push if ISR has reached threshold */
            sim->pc++;
            return true;
        }
        if (fifo_push(&sim->rx_fifo, sim->isr)) {
            sim->isr = 0;
            sim->isr_shift_count = 0;
            sim->pc++;
            return true;
        } else if (block) {
            sim->stalled = true;
            return false;
        } else {
            /* noblock: discard */
            sim->isr = 0;
            sim->isr_shift_count = 0;
            sim->pc++;
            return true;
        }
    }
}

static uint32_t mov_source(pio_sim_t *sim, int source)
{
    switch (source) {
    case SRC_PINS:  return (sim->gpio_in >> sim->in_base) & 0xFFFFFFFF;
    case SRC_X:     return sim->x;
    case SRC_Y:     return sim->y;
    case SRC_NULL:  return 0;
    case 5:         return 0; /* STATUS - not implemented */
    case SRC_ISR:   return sim->isr;
    case SRC_OSR:   return sim->osr;
    default:        return 0;
    }
}

static bool exec_mov(pio_sim_t *sim, uint16_t insn)
{
    int dest = (insn >> 5) & 0x7;
    int op = (insn >> 3) & 0x3;
    int source = insn & 0x7;

    uint32_t val = mov_source(sim, source);

    /* Apply operation */
    switch (op) {
    case MOV_OP_NONE:
        break;
    case MOV_OP_INVERT:
        val = ~val;
        break;
    case MOV_OP_REVERSE:
        val = bit_reverse(val);
        break;
    }

    /* Write to destination */
    switch (dest) {
    case SRC_PINS:
        /* Write all out pins */
        for (int i = 0; i < sim->out_count; i++) {
            int pin = (sim->out_base + i) % PIO_SIM_MAX_GPIO;
            if (val & (1u << i))
                sim->gpio_out |= (1u << pin);
            else
                sim->gpio_out &= ~(1u << pin);
        }
        break;
    case SRC_X:
        sim->x = val;
        break;
    case SRC_Y:
        sim->y = val;
        break;
    case 4: /* PINDIRS */
        for (int i = 0; i < sim->out_count; i++) {
            int pin = (sim->out_base + i) % PIO_SIM_MAX_GPIO;
            if (val & (1u << i))
                sim->gpio_dir |= (1u << pin);
            else
                sim->gpio_dir &= ~(1u << pin);
        }
        break;
    case 5: /* PC */
        sim->pc = val & 0x1F;
        return true;
    case SRC_ISR:
        sim->isr = val;
        sim->isr_shift_count = 0;
        break;
    case SRC_OSR:
        sim->osr = val;
        sim->osr_shift_count = 0;
        break;
    }

    sim->pc++;
    return true;
}

static bool exec_set(pio_sim_t *sim, uint16_t insn)
{
    int dest = (insn >> 5) & 0x7;
    uint32_t data = insn & 0x1F;

    switch (dest) {
    case SRC_PINS:
        for (int i = 0; i < sim->set_count; i++) {
            int pin = (sim->set_base + i) % PIO_SIM_MAX_GPIO;
            if (data & (1u << i))
                sim->gpio_out |= (1u << pin);
            else
                sim->gpio_out &= ~(1u << pin);
        }
        break;
    case SRC_X:
        sim->x = data;
        break;
    case SRC_Y:
        sim->y = data;
        break;
    case 4: /* PINDIRS */
        for (int i = 0; i < sim->set_count; i++) {
            int pin = (sim->set_base + i) % PIO_SIM_MAX_GPIO;
            if (data & (1u << i))
                sim->gpio_dir |= (1u << pin);
            else
                sim->gpio_dir &= ~(1u << pin);
        }
        break;
    }

    sim->pc++;
    return true;
}

/* ---- Public API ---- */

pio_sim_t *pio_sim_create(const uint16_t *program, int len)
{
    if (!program || len <= 0 || len > PIO_SIM_MAX_PROGRAM)
        return NULL;

    pio_sim_t *sim = calloc(1, sizeof(pio_sim_t));
    if (!sim)
        return NULL;

    memcpy(sim->program, program, len * sizeof(uint16_t));
    sim->program_len = len;
    sim->wrap_target = 0;
    sim->wrap_bottom = len - 1;

    /* Default shift config: right (LSB-first), no auto, 32-bit threshold */
    sim->out_shift_right = true;
    sim->in_shift_right = true;
    sim->autopull_threshold = 32;
    sim->autopush_threshold = 32;

    /* Default pin config */
    sim->out_count = 1;
    sim->in_count = 1;
    sim->set_count = 5;

    fifo_init(&sim->tx_fifo);
    fifo_init(&sim->rx_fifo);

    return sim;
}

void pio_sim_set_wrap(pio_sim_t *sim, int wrap_target, int wrap_bottom)
{
    sim->wrap_target = wrap_target;
    sim->wrap_bottom = wrap_bottom;
}

void pio_sim_set_sideset(pio_sim_t *sim, int count, bool opt, int base)
{
    sim->sideset_count = count;
    sim->sideset_opt = opt;
    sim->sideset_base = base;
}

void pio_sim_set_out_pins(pio_sim_t *sim, int base, int count)
{
    sim->out_base = base;
    sim->out_count = count;
}

void pio_sim_set_in_pins(pio_sim_t *sim, int base)
{
    sim->in_base = base;
}

void pio_sim_set_set_pins(pio_sim_t *sim, int base, int count)
{
    sim->set_base = base;
    sim->set_count = count;
}

void pio_sim_set_out_shift(pio_sim_t *sim, bool shift_right,
                           bool autopull, int threshold)
{
    sim->out_shift_right = shift_right;
    sim->autopull = autopull;
    sim->autopull_threshold = (threshold == 0) ? 32 : threshold;
}

void pio_sim_set_in_shift(pio_sim_t *sim, bool shift_right,
                          bool autopush, int threshold)
{
    sim->in_shift_right = shift_right;
    sim->autopush = autopush;
    sim->autopush_threshold = (threshold == 0) ? 32 : threshold;
}

bool pio_sim_tx_push(pio_sim_t *sim, uint32_t word)
{
    return fifo_push(&sim->tx_fifo, word);
}

bool pio_sim_rx_pop(pio_sim_t *sim, uint32_t *word)
{
    return fifo_pop(&sim->rx_fifo, word);
}

bool pio_sim_tx_full(pio_sim_t *sim)
{
    return sim->tx_fifo.count >= PIO_SIM_FIFO_DEPTH;
}

bool pio_sim_tx_empty(pio_sim_t *sim)
{
    return sim->tx_fifo.count == 0;
}

bool pio_sim_rx_full(pio_sim_t *sim)
{
    return sim->rx_fifo.count >= PIO_SIM_FIFO_DEPTH;
}

bool pio_sim_rx_empty(pio_sim_t *sim)
{
    return sim->rx_fifo.count == 0;
}

bool pio_sim_step(pio_sim_t *sim)
{
    if (sim->pc >= sim->program_len) {
        sim->halted = true;
        return false;
    }

    uint16_t insn = sim->program[sim->pc];
    int opcode = decode_opcode(insn);

    /* Decode delay and side-set */
    int delay = 0;
    int sideset_val = -1;
    decode_delay_sideset(insn, sim, &delay, &sideset_val);

    /* Apply side-set BEFORE instruction execution (happens simultaneously) */
    apply_sideset(sim, sideset_val);

    /* Execute instruction */
    sim->stalled = false;
    int old_pc = sim->pc;
    bool ok;

    switch (opcode) {
    case OP_JMP:       ok = exec_jmp(sim, insn); break;
    case OP_WAIT:      ok = exec_wait(sim, insn); break;
    case OP_IN:        ok = exec_in(sim, insn); break;
    case OP_OUT:       ok = exec_out(sim, insn); break;
    case OP_PUSH_PULL: ok = exec_push_pull(sim, insn); break;
    case OP_MOV:       ok = exec_mov(sim, insn); break;
    case OP_IRQ:       sim->pc++; ok = true; break;  /* IRQ not implemented */
    case OP_SET:       ok = exec_set(sim, insn); break;
    default:           sim->pc++; ok = true; break;
    }

    if (!ok)
        return false;

    /* Apply wrap */
    if (old_pc == sim->wrap_bottom && sim->pc == sim->wrap_bottom + 1) {
        sim->pc = sim->wrap_target;
    }

    /* Execute delay cycles */
    sim->cycle_count += 1 + delay;

    return true;
}

int pio_sim_run(pio_sim_t *sim, int max_cycles)
{
    int executed = 0;
    while (executed < max_cycles) {
        if (!pio_sim_step(sim))
            break;
        executed++;
    }
    return executed;
}

uint32_t pio_sim_gpio_get(pio_sim_t *sim)
{
    return sim->gpio_out;
}

void pio_sim_gpio_set(pio_sim_t *sim, int pin, bool value)
{
    if (pin < 0 || pin >= PIO_SIM_MAX_GPIO)
        return;
    if (value)
        sim->gpio_in |= (1u << pin);
    else
        sim->gpio_in &= ~(1u << pin);
}

bool pio_sim_gpio_get_pin(pio_sim_t *sim, int pin)
{
    if (pin < 0 || pin >= PIO_SIM_MAX_GPIO)
        return false;
    return (sim->gpio_out >> pin) & 1;
}

void pio_sim_destroy(pio_sim_t *sim)
{
    free(sim);
}
