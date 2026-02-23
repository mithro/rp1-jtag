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

#define DEFAULT_PORT 2542
#define DEFAULT_FREQ 6000000

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

    /* TODO: start XVC server */
    printf("\nTODO: start server\n");

    return 0;
}
