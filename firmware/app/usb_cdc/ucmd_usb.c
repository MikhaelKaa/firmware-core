// ucmd_usb.c
#include "ucmd.h"
#include "usb_cdc.h"
#include <stdio.h>
#include <string.h>

#ifdef BAREMETAL
#define ENDL "\r\n"
#else
#define ENDL "\n"
#endif

extern const drv_face_t dev_usb_cdc;

static void print_usage(void) {
    printf("Usage: usb <command>" ENDL);
    printf("Commands:" ENDL);
    printf("  read                - Read available data from USB CDC" ENDL);
    printf("  write               - Send test message to USB CDC" ENDL);
    printf("  reconnect           - Software USB re-enumeration" ENDL);
    printf("  help                - Show this help" ENDL);
}

int ucmd_usb(int argc, char **argv) {
    if (argc < 2) {
        print_usage();
        return 0;
    }

    if (strcmp(argv[1], "help") == 0) {
        print_usage();
        return 0;
    }

    if (strcmp(argv[1], "reconnect") == 0) {
        printf("USB reconnecting..." ENDL);
        dev_usb_cdc.ioctl(USB_CDC_SOFT_DISCONNECT, NULL);
        for (volatile int i = 0; i < 1000000; i++);
        dev_usb_cdc.ioctl(USB_CDC_SOFT_RECONNECT, NULL);
        printf("Done!" ENDL);
        return 0;
    }

    if (strcmp(argv[1], "read") == 0) {
        int avail = 0;
        dev_usb_cdc.ioctl(USB_CDC_GET_AVAILABLE, &avail);
        printf("Available: %d bytes" ENDL, avail);
        if (avail > 0) {
            char buf[128];
            int len = (avail > 127) ? 127 : avail;
            int read = dev_usb_cdc.read(buf, len);
            buf[read] = '\0';
            printf("Read %d bytes: '%s'" ENDL, read, buf);
        }
        return 0;
    }

    if (strcmp(argv[1], "write") == 0) {
        const char *msg = "Hello from STM32!" ENDL;
        int written = dev_usb_cdc.write(msg, strlen(msg));
        printf("Written %d bytes" ENDL, written);
        return 0;
    }

    printf("Unknown command: %s" ENDL, argv[1]);
    print_usage();
    return -1;
}
