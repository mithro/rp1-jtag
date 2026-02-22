/*
 * pio_backend_mock.c - Mock PIOLib backend for unit tests
 *
 * Simulates PIO FIFO behavior without /dev/pio0 or real hardware.
 * Tracks GPIO state, records TX data, and returns pre-loaded TDO data.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pio_backend.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define MOCK_FIFO_SIZE 256  /* Max words in simulated FIFO */
#define MOCK_MAX_GPIO  28   /* BCM GPIO 0-27 */

typedef struct {
    pio_backend_t base;

    /* State */
    bool initialized;
    bool enabled;
    pio_program_id_t program;
    float clk_div;

    /* Simulated TX FIFO (written by sm_put) */
    uint32_t tx_data[MOCK_FIFO_SIZE];
    int tx_count;

    /* Simulated RX FIFO (pre-loaded TDO data, read by sm_get) */
    uint32_t rx_data[MOCK_FIFO_SIZE];
    int rx_count;
    int rx_read_pos;

    /* GPIO state */
    bool gpio_state[MOCK_MAX_GPIO];

    /* Counters */
    int put_count;
    int get_count;
} mock_backend_t;

static int mock_init(pio_backend_t *be, pio_program_id_t prog,
                     const pio_sm_pins_t *pins)
{
    mock_backend_t *m = (mock_backend_t *)be;
    m->initialized = true;
    m->program = prog;
    if (pins)
        m->clk_div = pins->clk_div;
    return 0;
}

static void mock_close(pio_backend_t *be)
{
    mock_backend_t *m = (mock_backend_t *)be;
    m->initialized = false;
    m->enabled = false;
}

static int mock_set_clk_div(pio_backend_t *be, float div)
{
    mock_backend_t *m = (mock_backend_t *)be;
    m->clk_div = div;
    return 0;
}

static int mock_gpio_set(pio_backend_t *be, int pin, bool value)
{
    mock_backend_t *m = (mock_backend_t *)be;
    if (pin < 0 || pin >= MOCK_MAX_GPIO)
        return -1;
    m->gpio_state[pin] = value;
    return 0;
}

static int mock_sm_put(pio_backend_t *be, uint32_t word)
{
    mock_backend_t *m = (mock_backend_t *)be;
    if (m->tx_count >= MOCK_FIFO_SIZE)
        return -1;
    m->tx_data[m->tx_count++] = word;
    m->put_count++;
    return 0;
}

static int mock_sm_get(pio_backend_t *be, uint32_t *word)
{
    mock_backend_t *m = (mock_backend_t *)be;
    if (m->rx_read_pos >= m->rx_count) {
        /* No pre-loaded data left, return 0 */
        *word = 0;
    } else {
        *word = m->rx_data[m->rx_read_pos++];
    }
    m->get_count++;
    return 0;
}

static bool mock_tx_fifo_has_space(pio_backend_t *be)
{
    mock_backend_t *m = (mock_backend_t *)be;
    return m->tx_count < MOCK_FIFO_SIZE;
}

static bool mock_rx_fifo_has_data(pio_backend_t *be)
{
    mock_backend_t *m = (mock_backend_t *)be;
    return m->rx_read_pos < m->rx_count;
}

static int mock_sm_set_enabled(pio_backend_t *be, bool enabled)
{
    mock_backend_t *m = (mock_backend_t *)be;
    m->enabled = enabled;
    return 0;
}

static int mock_load_program(pio_backend_t *be, pio_program_id_t prog)
{
    mock_backend_t *m = (mock_backend_t *)be;
    m->program = prog;
    return 0;
}

static int mock_config_xfer(pio_backend_t *be, int dir,
                            uint32_t buf_size, uint32_t buf_count)
{
    (void)be; (void)dir; (void)buf_size; (void)buf_count;
    return 0;
}

static int mock_xfer_data(pio_backend_t *be, int dir,
                          uint32_t data_bytes, void *data)
{
    mock_backend_t *m = (mock_backend_t *)be;

    uint32_t num_words = data_bytes / 4;
    uint32_t *words = (uint32_t *)data;

    if (dir == 0) {
        /* TX: store data as if sm_put was called per word */
        for (uint32_t i = 0; i < num_words && m->tx_count < MOCK_FIFO_SIZE; i++) {
            m->tx_data[m->tx_count++] = words[i];
            m->put_count++;
        }
    } else {
        /* RX: return pre-loaded TDO data as if sm_get was called per word */
        for (uint32_t i = 0; i < num_words; i++) {
            if (m->rx_read_pos < m->rx_count)
                words[i] = m->rx_data[m->rx_read_pos++];
            else
                words[i] = 0;
            m->get_count++;
        }
    }
    return 0;
}

static const pio_backend_ops_t mock_ops = {
    .init             = mock_init,
    .close            = mock_close,
    .set_clk_div      = mock_set_clk_div,
    .gpio_set         = mock_gpio_set,
    .sm_put           = mock_sm_put,
    .sm_get           = mock_sm_get,
    .tx_fifo_has_space = mock_tx_fifo_has_space,
    .rx_fifo_has_data  = mock_rx_fifo_has_data,
    .sm_set_enabled   = mock_sm_set_enabled,
    .load_program     = mock_load_program,
    .config_xfer      = mock_config_xfer,
    .xfer_data        = mock_xfer_data,
};

pio_backend_t *pio_backend_mock_create(void)
{
    mock_backend_t *m = calloc(1, sizeof(mock_backend_t));
    if (!m)
        return NULL;
    m->base.ops = &mock_ops;
    return &m->base;
}

void pio_backend_destroy(pio_backend_t *be)
{
    if (!be)
        return;
    if (be->ops->close)
        be->ops->close(be);
    free(be);
}

/* Test helpers */

void pio_backend_mock_set_tdo_data(pio_backend_t *be,
                                   const uint32_t *words,
                                   int num_words)
{
    mock_backend_t *m = (mock_backend_t *)be;
    int count = num_words < MOCK_FIFO_SIZE ? num_words : MOCK_FIFO_SIZE;
    memcpy(m->rx_data, words, count * sizeof(uint32_t));
    m->rx_count = count;
    m->rx_read_pos = 0;
}

int pio_backend_mock_get_tdi_data(pio_backend_t *be,
                                  uint32_t *words,
                                  int max_words)
{
    mock_backend_t *m = (mock_backend_t *)be;
    int count = m->tx_count < max_words ? m->tx_count : max_words;
    memcpy(words, m->tx_data, count * sizeof(uint32_t));
    return count;
}

bool pio_backend_mock_get_gpio(pio_backend_t *be, int pin)
{
    mock_backend_t *m = (mock_backend_t *)be;
    if (pin < 0 || pin >= MOCK_MAX_GPIO)
        return false;
    return m->gpio_state[pin];
}

int pio_backend_mock_get_put_count(pio_backend_t *be)
{
    mock_backend_t *m = (mock_backend_t *)be;
    return m->put_count;
}

int pio_backend_mock_get_get_count(pio_backend_t *be)
{
    mock_backend_t *m = (mock_backend_t *)be;
    return m->get_count;
}

void pio_backend_mock_reset(pio_backend_t *be)
{
    mock_backend_t *m = (mock_backend_t *)be;
    m->tx_count = 0;
    m->rx_count = 0;
    m->rx_read_pos = 0;
    m->put_count = 0;
    m->get_count = 0;
    memset(m->gpio_state, 0, sizeof(m->gpio_state));
}
