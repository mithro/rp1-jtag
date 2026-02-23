/*
 * xvc_server.c - Xilinx Virtual Cable daemon for RP1 PIO JTAG
 *
 * Implements the XVC v1.0 protocol over TCP, using librp1jtag for
 * high-speed JTAG via the Raspberry Pi 5 RP1 PIO.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

#define DEFAULT_PORT 2542
#define DEFAULT_FREQ 6000000
#define XVC_MAX_VECTOR_BYTES 32768

static volatile sig_atomic_t running = 1;

static void signal_handler(int sig)
{
    (void)sig;
    running = 0;
}

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s --pins TDI:TDO:TCK:TMS [OPTIONS]\n"
        "\n"
        "Xilinx Virtual Cable (XVC) server using RP1 PIO JTAG.\n"
        "\n"
        "Required:\n"
        "  --pins TDI:TDO:TCK:TMS  BCM GPIO pin numbers (colon-separated)\n"
        "\n"
        "Options:\n"
        "  --port N       TCP port to listen on (default: %d)\n"
        "  --freq N       JTAG clock frequency in Hz (default: %d)\n"
        "  -v, --verbose  Enable verbose output\n"
        "  -h, --help     Show this help message\n"
        "\n"
        "Example:\n"
        "  sudo %s --pins 27:22:4:17 --port 2542\n",
        prog, DEFAULT_PORT, DEFAULT_FREQ, prog);
}

/*
 * Parse a colon-separated pin specification "TDI:TDO:TCK:TMS".
 * Returns 0 on success, -1 on error.
 */
static int parse_pins(const char *arg, rp1_jtag_pins_t *pins)
{
    char buf[64];
    char *saveptr = NULL;
    char *token;
    int values[4];
    int count = 0;

    if (strlen(arg) >= sizeof(buf)) {
        fprintf(stderr, "Error: --pins argument too long\n");
        return -1;
    }
    strcpy(buf, arg);

    token = strtok_r(buf, ":", &saveptr);
    while (token != NULL && count < 4) {
        char *endptr;
        long val = strtol(token, &endptr, 10);
        if (*endptr != '\0' || endptr == token) {
            fprintf(stderr, "Error: invalid pin number '%s'\n", token);
            return -1;
        }
        if (val < 0 || val > 27) {
            fprintf(stderr, "Error: pin %ld out of range (0-27)\n", val);
            return -1;
        }
        values[count++] = (int)val;
        token = strtok_r(NULL, ":", &saveptr);
    }

    if (count != 4 || token != NULL) {
        fprintf(stderr, "Error: --pins requires exactly 4 values (TDI:TDO:TCK:TMS)\n");
        return -1;
    }

    pins->tdi  = values[0];
    pins->tdo  = values[1];
    pins->tck  = values[2];
    pins->tms  = values[3];
    pins->srst = -1;
    pins->trst = -1;
    return 0;
}

/*
 * Read exactly n bytes from fd into buf.
 * Returns 0 on success, -1 on error or disconnect.
 */
static int read_exact(int fd, void *buf, size_t n)
{
    uint8_t *p = buf;
    size_t remaining = n;

    while (remaining > 0) {
        ssize_t rc = read(fd, p, remaining);
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (rc == 0)
            return -1;
        p += rc;
        remaining -= (size_t)rc;
    }
    return 0;
}

/*
 * Write exactly n bytes from buf to fd.
 * Returns 0 on success, -1 on error.
 */
static int write_exact(int fd, const void *buf, size_t n)
{
    const uint8_t *p = buf;
    size_t remaining = n;

    while (remaining > 0) {
        ssize_t rc = write(fd, p, remaining);
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        p += rc;
        remaining -= (size_t)rc;
    }
    return 0;
}

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | (uint32_t)p[1] << 8
         | (uint32_t)p[2] << 16
         | (uint32_t)p[3] << 24;
}

static void write_le32(uint8_t *p, uint32_t val)
{
    p[0] = (uint8_t)(val);
    p[1] = (uint8_t)(val >> 8);
    p[2] = (uint8_t)(val >> 16);
    p[3] = (uint8_t)(val >> 24);
}

/* Static buffers for shift data (avoids per-command allocation) */
static uint8_t tms_buf[XVC_MAX_VECTOR_BYTES];
static uint8_t tdi_buf[XVC_MAX_VECTOR_BYTES];
static uint8_t tdo_buf[XVC_MAX_VECTOR_BYTES];

/*
 * Handle a single XVC client connection.
 * Loops reading XVC commands until the client disconnects or an error occurs.
 */
static void handle_client(int fd, rp1_jtag_t *jtag, int verbose)
{
    uint8_t cmd_buf[8];

    while (running) {
        /* Read first byte to distinguish command */
        if (read_exact(fd, cmd_buf, 1) < 0)
            break;

        if (cmd_buf[0] == 'g') {
            /* getinfo: (8 bytes total) */
            if (read_exact(fd, cmd_buf + 1, 7) < 0)
                break;
            if (memcmp(cmd_buf, "getinfo:", 8) != 0) {
                fprintf(stderr, "Error: malformed getinfo command\n");
                break;
            }

            const char *info = "xvcServer_v1.0:32768\n";
            if (verbose)
                printf("  getinfo -> %.*s\n", (int)(strlen(info) - 1), info);
            if (write_exact(fd, info, strlen(info)) < 0)
                break;

        } else if (cmd_buf[0] == 's') {
            /* Could be "shift:" (6 bytes) or "settck:" (7 bytes) */
            if (read_exact(fd, cmd_buf + 1, 1) < 0)
                break;

            if (cmd_buf[1] == 'h') {
                /* shift: */
                if (read_exact(fd, cmd_buf + 2, 4) < 0)
                    break;
                if (memcmp(cmd_buf, "shift:", 6) != 0) {
                    fprintf(stderr, "Error: malformed shift command\n");
                    break;
                }

                uint8_t len_buf[4];
                if (read_exact(fd, len_buf, 4) < 0)
                    break;
                uint32_t num_bits = read_le32(len_buf);
                uint32_t byte_count = (num_bits + 7) / 8;

                if (byte_count > XVC_MAX_VECTOR_BYTES) {
                    fprintf(stderr, "Error: shift %u bits (%u bytes) exceeds max %d\n",
                            num_bits, byte_count, XVC_MAX_VECTOR_BYTES);
                    break;
                }

                if (read_exact(fd, tms_buf, byte_count) < 0)
                    break;
                if (read_exact(fd, tdi_buf, byte_count) < 0)
                    break;

                memset(tdo_buf, 0, byte_count);

                int rc = rp1_jtag_shift(jtag, num_bits, tms_buf, tdi_buf, tdo_buf);
                if (rc < 0) {
                    fprintf(stderr, "Error: rp1_jtag_shift failed: %d\n", rc);
                    break;
                }

                if (verbose)
                    printf("  shift %u bits\n", num_bits);

                if (write_exact(fd, tdo_buf, byte_count) < 0)
                    break;

            } else if (cmd_buf[1] == 'e') {
                /* settck: */
                if (read_exact(fd, cmd_buf + 2, 5) < 0)
                    break;
                if (memcmp(cmd_buf, "settck:", 7) != 0) {
                    fprintf(stderr, "Error: malformed settck command\n");
                    break;
                }

                uint8_t period_buf[4];
                if (read_exact(fd, period_buf, 4) < 0)
                    break;
                uint32_t period_ns = read_le32(period_buf);

                uint32_t freq_hz = 0;
                if (period_ns > 0)
                    freq_hz = 1000000000 / period_ns;

                rp1_jtag_set_freq(jtag, freq_hz);
                uint32_t actual_freq = rp1_jtag_get_freq(jtag);
                uint32_t actual_period_ns = 0;
                if (actual_freq > 0)
                    actual_period_ns = 1000000000 / actual_freq;

                if (verbose)
                    printf("  settck %u ns -> %u Hz (actual %u ns)\n",
                           period_ns, actual_freq, actual_period_ns);

                uint8_t resp[4];
                write_le32(resp, actual_period_ns);
                if (write_exact(fd, resp, 4) < 0)
                    break;

            } else {
                fprintf(stderr, "Error: unknown command starting with 's%c'\n",
                        cmd_buf[1]);
                break;
            }

        } else {
            fprintf(stderr, "Error: unknown command byte 0x%02x\n", cmd_buf[0]);
            break;
        }
    }
}

int main(int argc, char *argv[])
{
    const char *pins_arg = NULL;
    int port = DEFAULT_PORT;
    int freq = DEFAULT_FREQ;
    int verbose = 0;

    static const struct option long_options[] = {
        {"pins",    required_argument, NULL, 'p'},
        {"port",    required_argument, NULL, 'P'},
        {"freq",    required_argument, NULL, 'f'},
        {"verbose", no_argument,       NULL, 'v'},
        {"help",    no_argument,       NULL, 'h'},
        {NULL, 0, NULL, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "vh", long_options, NULL)) != -1) {
        switch (opt) {
        case 'p':
            pins_arg = optarg;
            break;
        case 'P':
            port = atoi(optarg);
            if (port <= 0 || port > 65535) {
                fprintf(stderr, "Error: invalid port number '%s'\n", optarg);
                return 1;
            }
            break;
        case 'f':
            freq = atoi(optarg);
            if (freq <= 0) {
                fprintf(stderr, "Error: invalid frequency '%s'\n", optarg);
                return 1;
            }
            break;
        case 'v':
            verbose = 1;
            break;
        case 'h':
            usage(argv[0]);
            return 0;
        default:
            usage(argv[0]);
            return 1;
        }
    }

    if (!pins_arg) {
        fprintf(stderr, "Error: --pins is required\n\n");
        usage(argv[0]);
        return 1;
    }

    rp1_jtag_pins_t pins;
    if (parse_pins(pins_arg, &pins) < 0)
        return 1;

    printf("rp1-xvc configuration:\n");
    printf("  Pins: TDI=%d TDO=%d TCK=%d TMS=%d\n",
           pins.tdi, pins.tdo, pins.tck, pins.tms);
    printf("  Port: %d\n", port);
    printf("  Freq: %d Hz\n", freq);
    printf("  Verbose: %s\n", verbose ? "yes" : "no");

    /* Initialize JTAG */
    rp1_jtag_t *jtag = rp1_jtag_init(&pins);
    if (!jtag) {
        fprintf(stderr, "Error: failed to initialize JTAG\n");
        return 1;
    }

    rp1_jtag_set_freq(jtag, (uint32_t)freq);
    printf("  Actual freq: %u Hz\n", rp1_jtag_get_freq(jtag));

    /* Set up signal handling */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* Create TCP server socket */
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket");
        rp1_jtag_close(jtag);
        return 1;
    }

    int opt_val = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt_val, sizeof(opt_val));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);

    if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(server_fd);
        rp1_jtag_close(jtag);
        return 1;
    }

    if (listen(server_fd, 1) < 0) {
        perror("listen");
        close(server_fd);
        rp1_jtag_close(jtag);
        return 1;
    }

    printf("Listening on port %d\n", port);

    /* Accept loop */
    while (running) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd, (struct sockaddr *)&client_addr,
                               &client_len);
        if (client_fd < 0) {
            if (errno == EINTR)
                continue;
            perror("accept");
            break;
        }

        if (verbose)
            printf("Client connected: %s:%d\n",
                   inet_ntoa(client_addr.sin_addr),
                   ntohs(client_addr.sin_port));

        handle_client(client_fd, jtag, verbose);
        close(client_fd);

        if (verbose)
            printf("Client disconnected\n");
    }

    printf("Shutting down\n");
    close(server_fd);
    rp1_jtag_close(jtag);
    return 0;
}
