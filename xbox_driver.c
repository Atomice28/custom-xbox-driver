#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <linux/uinput.h>
#include <libusb-1.0/libusb.h>

#define VENDOR_ID  0x37d7
#define PRODUCT_ID 0x2801
#define MAX_EPS    8

static volatile int keep_running = 1;
void handle_sigint(int sig) {
    (void)sig;
    keep_running = 0;
}

void emit(int fd, int type, int code, int val) {
    struct input_event ie;
    memset(&ie, 0, sizeof(ie));
    ie.type = type;
    ie.code = code;
    ie.value = val;
    write(fd, &ie, sizeof(ie));
}

int setup_uinput_device() {
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        perror("Failed to open /dev/uinput (Run with sudo)");
        return -1;
    }

    ioctl(fd, UI_SET_EVBIT, EV_KEY);
    int buttons[] = {
        BTN_SOUTH, BTN_EAST, BTN_NORTH, BTN_WEST,
        BTN_TL, BTN_TR, BTN_SELECT, BTN_START, BTN_MODE,
        BTN_THUMBL, BTN_THUMBR,
        BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT,
        // Extra Flydigi Buttons: M1-M4, C, Z, LM, RM
        BTN_TRIGGER_HAPPY1, BTN_TRIGGER_HAPPY2,
        BTN_TRIGGER_HAPPY3, BTN_TRIGGER_HAPPY4,
        BTN_TRIGGER_HAPPY5, BTN_TRIGGER_HAPPY6,
        BTN_TRIGGER_HAPPY7, BTN_TRIGGER_HAPPY8
    };
    for (size_t i = 0; i < sizeof(buttons) / sizeof(buttons[0]); i++) {
        ioctl(fd, UI_SET_KEYBIT, buttons[i]);
    }

    ioctl(fd, UI_SET_EVBIT, EV_ABS);
    int sticks[] = {ABS_X, ABS_Y, ABS_RX, ABS_RY};
    for (int i = 0; i < 4; i++) {
        struct uinput_abs_setup abs_setup = {0};
        abs_setup.code = sticks[i];
        abs_setup.absinfo.minimum = -32768;
        abs_setup.absinfo.maximum = 32767;
        abs_setup.absinfo.fuzz = 16;
        abs_setup.absinfo.flat = 128;
        ioctl(fd, UI_ABS_SETUP, &abs_setup);
    }

    int triggers[] = {ABS_Z, ABS_RZ};
    for (int i = 0; i < 2; i++) {
        struct uinput_abs_setup abs_setup = {0};
        abs_setup.code = triggers[i];
        abs_setup.absinfo.minimum = 0;
        abs_setup.absinfo.maximum = 1023;
        ioctl(fd, UI_ABS_SETUP, &abs_setup);
    }

    struct uinput_setup usetup;
    memset(&usetup, 0, sizeof(usetup));
    usetup.id.bustype = BUS_USB;
    usetup.id.vendor  = 0x045e;
    usetup.id.product = 0x0b00; // Xbox Elite ID so Steam recognizes back paddles
    strcpy(usetup.name, "Flydigi Vader 5S Pro Driver");

    ioctl(fd, UI_DEV_SETUP, &usetup);
    ioctl(fd, UI_DEV_CREATE);
    return fd;
}

int main() {
    signal(SIGINT, handle_sigint);
    if (libusb_init(NULL) < 0) return 1;

    libusb_device_handle *devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
    if (!devh) {
        printf("Device 37d7:2801 not found. Is it plugged in and passed through?\n");
        libusb_exit(NULL);
        return 1;
    }

    libusb_set_auto_detach_kernel_driver(devh, 1);

    libusb_device *dev = libusb_get_device(devh);
    struct libusb_config_descriptor *config;
    if (libusb_get_active_config_descriptor(dev, &config) < 0) return 1;

    uint8_t ep_in_list[MAX_EPS];
    uint8_t ep_out_list[MAX_EPS];
    int num_in = 0, num_out = 0;
    int num_interfaces = config->bNumInterfaces;

    // Claim all interfaces (Interface 0 for XInput/GIP + Interface 1/2 for Flydigi Paddles)
    for (int i = 0; i < num_interfaces; i++) {
        if (libusb_claim_interface(devh, i) == 0) {
            printf("Claimed Interface %d\n", i);
        }
        const struct libusb_interface *inter = &config->interface[i];
        if (inter->num_altsetting > 0) {
            const struct libusb_interface_descriptor *desc = &inter->altsetting[0];
            for (int k = 0; k < desc->bNumEndpoints; k++) {
                const struct libusb_endpoint_descriptor *ep = &desc->endpoint[k];
                if ((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_INTERRUPT) {
                    if ((ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) && num_in < MAX_EPS) {
                        ep_in_list[num_in++] = ep->bEndpointAddress;
                        printf("  -> Found IN Endpoint: 0x%02x (Interface %d)\n", ep->bEndpointAddress, i);
                    } else if (!(ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) && num_out < MAX_EPS) {
                        ep_out_list[num_out++] = ep->bEndpointAddress;
                        printf("  -> Found OUT Endpoint: 0x%02x (Interface %d)\n", ep->bEndpointAddress, i);
                    }
                }
            }
        }
    }
    libusb_free_config_descriptor(config);

    // 1. Send Xbox 360 / XInput wake-up control requests on cable
    uint8_t ctrl_buf[20];
    libusb_control_transfer(devh, 0xC1, 0x01, 0x0100, 0x00, ctrl_buf, 0x14, 100);
    libusb_control_transfer(devh, 0xC1, 0x01, 0x0000, 0x00, ctrl_buf, 0x08, 100);
    libusb_control_transfer(devh, 0xC0, 0x01, 0x0000, 0x00, ctrl_buf, 0x04, 100);

    // 2. Send GIP Power-On and Flydigi V2 Acquire Packet (unlocks M1-M4, C, Z, LM, RM)
    uint8_t gip_power_on[] = {0x05, 0x20, 0x00, 0x01, 0x00};
    uint8_t flydigi_acquire[32] = {
        0x5A, 0xA5, 0x1C, 0x17, 0x01, 'S', 'D', 'L', 0x00
    };
    int transferred = 0;
    for (int i = 0; i < num_out; i++) {
        libusb_interrupt_transfer(devh, ep_out_list[i], gip_power_on, sizeof(gip_power_on), &transferred, 100);
        libusb_interrupt_transfer(devh, ep_out_list[i], flydigi_acquire, sizeof(flydigi_acquire), &transferred, 100);
    }

    int uinput_fd = setup_uinput_device();
    if (uinput_fd < 0) return 1;

    printf("Driver running with Back Paddle (M1-M4, C, Z) support! Press Ctrl+C to stop.\n");

    uint8_t buf[64];
    while (keep_running) {
        for (int i = 0; i < num_in; i++) {
            int r = libusb_interrupt_transfer(devh, ep_in_list[i], buf, sizeof(buf), &transferred, 4);
            if (r != 0 || transferred == 0) continue;

            // Locate Flydigi V2 32-byte Vendor Packet (Header: 5A A5 EF)
            int offset = -1;
            if (transferred >= 32 && buf[0] == 0x5A && buf[1] == 0xA5 && buf[2] == 0xEF) {
                offset = 0;
            } else if (transferred >= 33 && buf[1] == 0x5A && buf[2] == 0xA5 && buf[3] == 0xEF) {
                offset = 1; // Skips leading Report ID byte if present
            }

            // Case 1: Flydigi V2 Vendor Packet (Includes Back Paddles M1-M4, C, Z, LM, RM)
            if (offset >= 0) {
                uint8_t *d = &buf[offset];

                // Sticks (Bytes 3-10, 16-bit signed little-endian)
                int16_t lx = (int16_t)(d[3] | (d[4] << 8));
                int16_t ly = (int16_t)(d[5] | (d[6] << 8));
                int16_t rx = (int16_t)(d[7] | (d[8] << 8));
                int16_t ry = (int16_t)(d[9] | (d[10] << 8));
                emit(uinput_fd, EV_ABS, ABS_X,  lx);
                emit(uinput_fd, EV_ABS, ABS_Y,  ly);
                emit(uinput_fd, EV_ABS, ABS_RX, rx);
                emit(uinput_fd, EV_ABS, ABS_RY, ry);

                // Byte 11: D-Pad + A, B, Select, X
                emit(uinput_fd, EV_KEY, BTN_DPAD_UP,    (d[11] & 0x01) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_DPAD_RIGHT, (d[11] & 0x02) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_DPAD_DOWN,  (d[11] & 0x04) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_DPAD_LEFT,  (d[11] & 0x08) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_SOUTH,      (d[11] & 0x10) ? 1 : 0); // A
                emit(uinput_fd, EV_KEY, BTN_EAST,       (d[11] & 0x20) ? 1 : 0); // B
                emit(uinput_fd, EV_KEY, BTN_SELECT,     (d[11] & 0x40) ? 1 : 0); // View
                emit(uinput_fd, EV_KEY, BTN_WEST,       (d[11] & 0x80) ? 1 : 0); // X

                // Byte 12: Y, Start, LB, RB, LS, RS
                emit(uinput_fd, EV_KEY, BTN_NORTH,      (d[12] & 0x01) ? 1 : 0); // Y
                emit(uinput_fd, EV_KEY, BTN_START,      (d[12] & 0x02) ? 1 : 0); // Menu
                emit(uinput_fd, EV_KEY, BTN_TL,         (d[12] & 0x04) ? 1 : 0); // LB
                emit(uinput_fd, EV_KEY, BTN_TR,         (d[12] & 0x08) ? 1 : 0); // RB
                emit(uinput_fd, EV_KEY, BTN_THUMBL,     (d[12] & 0x40) ? 1 : 0); // LS
                emit(uinput_fd, EV_KEY, BTN_THUMBR,     (d[12] & 0x80) ? 1 : 0); // RS

                // Byte 13: Back Paddles (M1-M4) and Extra Buttons (C, Z, LM, RM)
                emit(uinput_fd, EV_KEY, BTN_TRIGGER_HAPPY1, (d[13] & 0x04) ? 1 : 0); // M1 Paddle
                emit(uinput_fd, EV_KEY, BTN_TRIGGER_HAPPY2, (d[13] & 0x08) ? 1 : 0); // M2 Paddle
                emit(uinput_fd, EV_KEY, BTN_TRIGGER_HAPPY3, (d[13] & 0x10) ? 1 : 0); // M3 Paddle
                emit(uinput_fd, EV_KEY, BTN_TRIGGER_HAPPY4, (d[13] & 0x20) ? 1 : 0); // M4 Paddle
                emit(uinput_fd, EV_KEY, BTN_TRIGGER_HAPPY5, (d[13] & 0x01) ? 1 : 0); // C Button
                emit(uinput_fd, EV_KEY, BTN_TRIGGER_HAPPY6, (d[13] & 0x02) ? 1 : 0); // Z Button
                emit(uinput_fd, EV_KEY, BTN_TRIGGER_HAPPY7, (d[13] & 0x40) ? 1 : 0); // LM Bumper
                emit(uinput_fd, EV_KEY, BTN_TRIGGER_HAPPY8, (d[13] & 0x80) ? 1 : 0); // RM Bumper

                // Byte 14: Guide / Xbox Button
                emit(uinput_fd, EV_KEY, BTN_MODE, (d[14] & 0x08) ? 1 : 0);

                // Bytes 15-16: Triggers (0-255 scaled to 0-1023)
                emit(uinput_fd, EV_ABS, ABS_Z,  ((uint16_t)d[15]) * 4);
                emit(uinput_fd, EV_ABS, ABS_RZ, ((uint16_t)d[16]) * 4);

                emit(uinput_fd, EV_SYN, SYN_REPORT, 0);
            }
            // Case 2: Standard 20-byte XInput Packet on Interface 0
            else if (buf[0] == 0x00 && buf[1] == 0x14 && transferred >= 14) {
                emit(uinput_fd, EV_KEY, BTN_DPAD_UP,    (buf[2] & 0x01) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_DPAD_DOWN,  (buf[2] & 0x02) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_DPAD_LEFT,  (buf[2] & 0x04) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_DPAD_RIGHT, (buf[2] & 0x08) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_START,      (buf[2] & 0x10) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_SELECT,     (buf[2] & 0x20) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_THUMBL,     (buf[2] & 0x40) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_THUMBR,     (buf[2] & 0x80) ? 1 : 0);

                emit(uinput_fd, EV_KEY, BTN_TL,         (buf[3] & 0x01) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_TR,         (buf[3] & 0x02) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_MODE,       (buf[3] & 0x04) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_SOUTH,      (buf[3] & 0x10) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_EAST,       (buf[3] & 0x20) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_WEST,       (buf[3] & 0x40) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_NORTH,      (buf[3] & 0x80) ? 1 : 0);

                emit(uinput_fd, EV_ABS, ABS_Z,  ((uint16_t)buf[4]) * 4);
                emit(uinput_fd, EV_ABS, ABS_RZ, ((uint16_t)buf[5]) * 4);

                int16_t lx = (int16_t)(buf[6]  | (buf[7]  << 8));
                int16_t ly = (int16_t)(buf[8]  | (buf[9]  << 8));
                int16_t rx = (int16_t)(buf[10] | (buf[11] << 8));
                int16_t ry = (int16_t)(buf[12] | (buf[13] << 8));
                emit(uinput_fd, EV_ABS, ABS_X,  lx);
                emit(uinput_fd, EV_ABS, ABS_Y,  ~ly);
                emit(uinput_fd, EV_ABS, ABS_RX, rx);
                emit(uinput_fd, EV_ABS, ABS_RY, ~ry);

                emit(uinput_fd, EV_SYN, SYN_REPORT, 0);
            }
            // Case 3: Standard Xbox One GIP Packet (Header 0x20)
            else if (buf[0] == 0x20 && transferred >= 18) {
                emit(uinput_fd, EV_KEY, BTN_START,  (buf[4] & 0x04) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_SELECT, (buf[4] & 0x08) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_SOUTH,  (buf[4] & 0x10) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_EAST,   (buf[4] & 0x20) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_WEST,   (buf[4] & 0x40) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_NORTH,  (buf[4] & 0x80) ? 1 : 0);

                emit(uinput_fd, EV_KEY, BTN_DPAD_UP,    (buf[5] & 0x01) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_DPAD_DOWN,  (buf[5] & 0x02) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_DPAD_LEFT,  (buf[5] & 0x04) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_DPAD_RIGHT, (buf[5] & 0x08) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_TL,         (buf[5] & 0x10) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_TR,         (buf[5] & 0x20) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_THUMBL,     (buf[5] & 0x40) ? 1 : 0);
                emit(uinput_fd, EV_KEY, BTN_THUMBR,     (buf[5] & 0x80) ? 1 : 0);

                uint16_t lt = buf[6] | (buf[7] << 8);
                uint16_t rt = buf[8] | (buf[9] << 8);
                emit(uinput_fd, EV_ABS, ABS_Z,  lt);
                emit(uinput_fd, EV_ABS, ABS_RZ, rt);

                int16_t lx = (int16_t)(buf[10] | (buf[11] << 8));
                int16_t ly = (int16_t)(buf[12] | (buf[13] << 8));
                int16_t rx = (int16_t)(buf[14] | (buf[15] << 8));
                int16_t ry = (int16_t)(buf[16] | (buf[17] << 8));
                emit(uinput_fd, EV_ABS, ABS_X,  lx);
                emit(uinput_fd, EV_ABS, ABS_Y,  ~ly);
                emit(uinput_fd, EV_ABS, ABS_RX, rx);
                emit(uinput_fd, EV_ABS, ABS_RY, ~ry);

                emit(uinput_fd, EV_SYN, SYN_REPORT, 0);
            }
        }
    }

    printf("\nShutting down driver...\n");
    ioctl(uinput_fd, UI_DEV_DESTROY);
    close(uinput_fd);
    for (int i = 0; i < num_interfaces; i++) {
        libusb_release_interface(devh, i);
    }
    libusb_close(devh);
    libusb_exit(NULL);
    return 0;
}