// ucmd_usb.c
#include "ucmd.h"
#include "usb_cdc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef BAREMETAL
#define ENDL "\r\n"
#else
#define ENDL "\n"
#endif

#define MAX_SEND_BYTES 256

extern const drv_face_t dev_usb_cdc;

// Parse hex string to byte
static int hex_string_to_byte(const char *str, uint8_t *out) {
    char *endptr;
    unsigned long val = strtoul(str, &endptr, 16);
    if (*endptr != '\0' || endptr == str) return -1;
    if (val > 0xFF) return -1;
    *out = (uint8_t)val;
    return 0;
}

// Parse continuous hex string (e.g., "aabbcc45ff")
static int parse_hex_string(const char *str, uint8_t *out, size_t max_out) {
    size_t len = strlen(str);
    if (len == 0 || len % 2 != 0) return -1;
    size_t byte_count = len / 2;
    if (byte_count > max_out) return -1;

    for (size_t i = 0; i < byte_count; i++) {
        char byte_str[3] = { str[2 * i], str[2 * i + 1], '\0' };
        char *endptr;
        unsigned long val = strtoul(byte_str, &endptr, 16);
        if (*endptr != '\0' || endptr == byte_str) return -1;
        if (val > 0xFF) return -1;
        out[i] = (uint8_t)val;
    }
    return (int)byte_count;
}

// Print buffer in hex (16 bytes per line)
static void print_hex_buffer(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", data[i]);
        if ((i + 1) % 16 == 0) printf(ENDL);
    }
    if (len % 16 != 0) printf(ENDL);
}

static void print_usage(void) {
    printf("Usage:" ENDL);
    printf("  usb info                  - show USB CDC info" ENDL);
    printf("  usb send <hex bytes...>   - send bytes (separate hex arguments)" ENDL);
    printf("  usb send <hexstring>      - send bytes from continuous hex string" ENDL);
    printf("  usb recv [cnt]            - receive and print [cnt] bytes (default all)" ENDL);
    printf("  usb reconnect             - software USB re-enumeration" ENDL);
    printf("  usb dtr                   - get DTR state" ENDL);
    printf("  usb rts                   - get RTS state" ENDL);
    printf("  usb help                  - show this help" ENDL);
}

int ucmd_usb(int argc, char **argv) {
    if (argc < 2) {
        print_usage();
        return 0;
    }

    const char *cmd = argv[1];

    // ===== help =====
    if (strcmp(cmd, "help") == 0) {
        print_usage();
        return 0;
    }

    // ===== info =====
    if (strcmp(cmd, "info") == 0) {
        const char *info = NULL;
        if (dev_usb_cdc.ioctl(INTERFACE_GET_INFO, (void*)&info) == 0 && info != NULL)
            printf("%s" ENDL, info);
        else
            printf("USB CDC: info not available" ENDL);
        return 0;
    }

    // ===== reconnect =====
    if (strcmp(cmd, "reconnect") == 0) {
        printf("USB reconnecting..." ENDL);
        dev_usb_cdc.ioctl(USB_CDC_SOFT_DISCONNECT, NULL);
        for (volatile int i = 0; i < 1000000; i++);
        dev_usb_cdc.ioctl(USB_CDC_SOFT_RECONNECT, NULL);
        printf("Done!" ENDL);
        return 0;
    }

    // ===== send =====
    if (strcmp(cmd, "send") == 0) {
        if (argc < 3) {
            printf("Usage: usb send <hex bytes...> or <hexstring>" ENDL);
            return UCMD_CMD_NOT_FOUND;
        }

        uint8_t data[MAX_SEND_BYTES];
        int byte_count = 0;

        if (argc == 3) {
            // Single argument - continuous hex string
            byte_count = parse_hex_string(argv[2], data, MAX_SEND_BYTES);
            if (byte_count < 0) {
                printf("Error: invalid hex string '%s' (must be even-length hex)" ENDL, argv[2]);
                return UCMD_CMD_NOT_FOUND;
            }
        } else {
            // Multiple arguments - each is a separate byte
            byte_count = argc - 2;
            if (byte_count > MAX_SEND_BYTES) {
                printf("Error: too many bytes (max %d)" ENDL, MAX_SEND_BYTES);
                return UCMD_CMD_NOT_FOUND;
            }
            for (int i = 2; i < argc; i++) {
                if (hex_string_to_byte(argv[i], &data[i - 2]) != 0) {
                    printf("Error: invalid hex byte '%s'" ENDL, argv[i]);
                    return UCMD_CMD_NOT_FOUND;
                }
            }
        }

        int sent = dev_usb_cdc.write(data, byte_count);
        if (sent < 0) {
            printf("Error: write failed (%d)" ENDL, sent);
            return UCMD_CMD_NOT_FOUND;
        }
        printf("Sent %d bytes" ENDL, sent);
        return 0;
    }

    // ===== recv =====
    if (strcmp(cmd, "recv") == 0) {
        int count = -1; // -1 = all available
        if (argc >= 3) {
            count = atoi(argv[2]);
            if (count <= 0) {
                printf("Error: invalid count '%s'" ENDL, argv[2]);
                return UCMD_CMD_NOT_FOUND;
            }
        }

        int available;
        if (dev_usb_cdc.ioctl(USB_CDC_GET_AVAILABLE, &available) != 0) {
            printf("Error: cannot get available bytes" ENDL);
            return UCMD_CMD_NOT_FOUND;
        }

        if (available == 0) {
            printf("No data available" ENDL);
            return 0;
        }

        if (count == -1 || count > available)
            count = available;

        uint8_t buffer[count];
        int received = dev_usb_cdc.read(buffer, count);
        if (received < 0) {
            printf("Error: read failed (%d)" ENDL, received);
            return UCMD_CMD_NOT_FOUND;
        }

        printf("Received %d bytes:" ENDL, received);
        print_hex_buffer(buffer, received);
        return 0;
    }

    // ===== dtr =====
    if (strcmp(cmd, "dtr") == 0) {
        uint8_t dtr = 0;
        if (dev_usb_cdc.ioctl(USB_CDC_GET_DTR, &dtr) == 0)
            printf("DTR: %s" ENDL, dtr ? "ON" : "OFF");
        else
            printf("Error: cannot get DTR" ENDL);
        return 0;
    }

    // ===== rts =====
    if (strcmp(cmd, "rts") == 0) {
        uint8_t rts = 0;
        if (dev_usb_cdc.ioctl(USB_CDC_GET_RTS, &rts) == 0)
            printf("RTS: %s" ENDL, rts ? "ON" : "OFF");
        else
            printf("Error: cannot get RTS" ENDL);
        return 0;
    }

    // Unknown command
    printf("Unknown command '%s'" ENDL, cmd);
    print_usage();
    return UCMD_CMD_NOT_FOUND;
}
