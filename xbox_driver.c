/*
 * vader5s_driver_v5.c - userspace driver for the Flydigi Vader 5S (37d7:2801)
 *
 * Build:  gcc -O2 -Wall -Wextra -o vader5s_driver_v5 vader5s_driver_v5.c -lusb-1.0
 * Run:    sudo ./vader5s_driver_v5 [options]
 *
 *   -v         hex-dump every packet in both directions (very chatty)
 *   -n         listen only: send nothing to the pad (no init, no ACKs, no rumble)
 *   -e         present the virtual pad as an Xbox Elite Series 2 (045e:0b00)
 *              instead of a plain Xbox 360 pad (045e:028e)
 *   -p         chmod 0666 the virtual pad's /dev/input nodes (a udev rule is cleaner)
 *   -t         also drive the two trigger (impulse) motors with the game's rumble. Linux's rumble
 *              API has no trigger channel, so this mirrors the game's main-motor strength onto
 *              both triggers. Event-driven: nothing is sent unless the rumble level changes.
 *   -r         USB-reset the controller before claiming it (re-opens it if it re-enumerates)
 *   -R         real-time scheduling (SCHED_FIFO) + mlockall; lowest, steadiest latency
 *   -w         wait for the controller to appear / reappear instead of exiting
 *   -L         learn mode: print which byte/bit changes when you press extra buttons
 *   -m O:M:N   map a bit of the GIP input report to a button, e.g. -m 22:0x01:share
 *              (O = byte offset in the 0x20 packet, M = bit mask, N = name or key code;
 *              names: share, paddle1..4, extra1..8; a number like 0x2c4 also works)
 *   -d V:P     USB id to open (default 37d7:2801)
 *
 * What is known about this pad (from the lsusb dump in paroj/xpad PR #368 and the
 * startup sequence in OpenJoystickDriver PR #40):
 *   - full-speed USB, class ff/47/d0 = Xbox One "GIP" protocol
 *   - interface 0: IN 0x81 (64 B, bInterval 2 ms) and OUT 0x01 (64 B, bInterval 4 ms)
 *   - interface 1 is audio (isochronous) and is left to the kernel
 *   - it needs a host ACK for its identify message and then the startup sequence
 *     hori-ACK -> LED on -> auth done -> power on, or it stays unresponsive
 * bInterval 2 on a full-speed endpoint means that unit cannot report faster than
 * ~500 Hz (some reviews quote 1000 Hz; the [stats] line shows what your unit really does); the driver's job is to add no delay on top of that, and it prints the
 * rate it actually measures so you can check.
 *
 * Optional udev rule instead of -p (/etc/udev/rules.d/99-vader5s.rules):
 *   SUBSYSTEM=="input", ATTRS{name}=="Flydigi Vader 5S (userspace driver)", TAG+="uaccess"
 */
#define _GNU_SOURCE
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <libusb-1.0/libusb.h>

/* Keycodes that older kernel headers may lack (values from Linux 6.18). */
#ifndef KEY_RECORD
#define KEY_RECORD 167
#endif
#ifndef BTN_GRIPL
#define BTN_GRIPL  0x224
#define BTN_GRIPR  0x225
#define BTN_GRIPL2 0x226
#define BTN_GRIPR2 0x227
#endif

#define MAX_EPS    8
#define MAX_IFACES 16
#define MAX_MAPS   24
#define URB_DEPTH  4            /* pipelined IN transfers per endpoint */
#define FF_SLOTS   16

enum proto { PROTO_UNKNOWN, PROTO_GIP, PROTO_X360, PROTO_HID };
static const char *proto_name[] = { "unknown", "GIP", "XInput360", "HID" };

struct ep_ctx;
struct urb_slot {
    struct ep_ctx *ep;
    struct libusb_transfer *xfer;
    int active;
    uint8_t buf[512];
};

struct ep_ctx {
    uint8_t  addr;
    uint16_t maxpkt;
    int      iface;
    enum proto proto;
    struct urb_slot slots[URB_DEPTH];
    int      inflight, needs_restart, cancel_issued;
    unsigned long packets, last_packets, unparsed, errors;
    uint64_t last_ns, win_max_gap_ns;
};

struct map_entry { int offset; uint8_t mask; int code; };

struct ff_slot {
    struct ff_effect e;
    int valid, playing;
    uint64_t start_ms, stop_ms;         /* stop_ms == 0: until stopped */
};

static struct {
    libusb_device_handle *devh;
    uint16_t vid, pid;
    int ufd;
    int verbose, listen_only, elite, fix_perms, usb_reset, rt, wait_dev, learn, v2_keys, trigger_rumble;
    uint8_t gip_out, x360_out, gip_seq;
    int announces;
    struct ep_ctx eps[MAX_EPS];
    int num_eps, total_inflight, disconnected, stopping;
    int claimed[MAX_IFACES], num_claimed;
    struct map_entry maps[MAX_MAPS];
    int num_maps;
    uint8_t key_state[KEY_CNT];
    int32_t abs_state[ABS_CNT];
    struct ff_slot ff[FF_SLOTS];
    uint32_t ff_gain, rumble_sent_s, rumble_sent_w;
    uint64_t rumble_last_ms, last_init_ms;
    struct pollfd *pfds;
    int npfds, pfd_dirty;
} g = { .ufd = -1, .ff_gain = 0xffff, .pfd_dirty = 1 };

static volatile sig_atomic_t g_run = 1;
static void on_signal(int s) { (void)s; g_run = 0; }

/* ------------------------------------------------------------------ utils */

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}
static uint64_t now_ms(void) { return now_ns() / 1000000ULL; }

static void hexdump(const char *tag, const uint8_t *p, int n)
{
    fprintf(stderr, "%s [%d]:", tag, n);
    for (int i = 0; i < n && i < 64; i++)
        fprintf(stderr, " %02x", p[i]);
    if (n > 64)
        fprintf(stderr, " ...");
    fputc('\n', stderr);
}

static void note_few(const char *tag, const uint8_t *p, int n)
{
    static int shown;
    if (!g.verbose && shown < 16) {
        hexdump(tag, p, n);
        shown++;
    }
}

static inline int le16s(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }
static inline int le16u(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline int clamp1023(int v) { return v > 1023 ? 1023 : v; }

/* --------------------------------- delta-only uinput events (one write per report) */

struct evbuf { struct input_event ev[96]; int n; };

static void ev_push(struct evbuf *b, int type, int code, int val)
{
    if (b->n >= 95)
        return;
    struct input_event *e = &b->ev[b->n++];
    memset(e, 0, sizeof *e);
    e->type = (uint16_t)type;
    e->code = (uint16_t)code;
    e->value = val;
}

static void ev_key(struct evbuf *b, int code, int on)
{
    if (code < 0 || code >= KEY_CNT)
        return;
    on = on ? 1 : 0;
    if (g.key_state[code] == on)
        return;
    g.key_state[code] = (uint8_t)on;
    ev_push(b, EV_KEY, code, on);
}

static void ev_abs(struct evbuf *b, int code, int val)
{
    if (code < 0 || code >= ABS_CNT)
        return;
    if (g.abs_state[code] == val)
        return;
    g.abs_state[code] = val;
    ev_push(b, EV_ABS, code, val);
}

static void ev_flush(struct evbuf *b)
{
    if (b->n == 0)
        return;                                 /* nothing changed: no syscall at all */
    ev_push(b, EV_SYN, SYN_REPORT, 0);
    if (g.ufd >= 0) {
        ssize_t r = write(g.ufd, b->ev, sizeof b->ev[0] * (size_t)b->n);
        if (r < 0 && errno != EAGAIN)
            perror("uinput write");
    }
    b->n = 0;
}

static void release_all(void)                   /* no stuck buttons after a disconnect */
{
    struct evbuf b = { .n = 0 };
    for (int c = 0; c < KEY_CNT; c++)
        if (g.key_state[c])
            ev_key(&b, c, 0);
    for (int c = 0; c < ABS_CNT; c++)
        if (g.abs_state[c])
            ev_abs(&b, c, 0);
    ev_flush(&b);
}

/* ------------------------------------------------------------- USB output */

static void LIBUSB_CALL out_done(struct libusb_transfer *t)
{
    if (t->status != LIBUSB_TRANSFER_COMPLETED && t->status != LIBUSB_TRANSFER_CANCELLED)
        fprintf(stderr, "OUT 0x%02x failed (transfer status %d)\n", t->endpoint, t->status);
}

/* fire-and-forget interrupt OUT; safe to call from inside a callback */
static void usb_send(uint8_t ep, const uint8_t *data, size_t len)
{
    if (!ep || !g.devh || g.listen_only)
        return;
    if (g.verbose) {
        char tag[24];
        snprintf(tag, sizeof tag, "OUT 0x%02x", ep);
        hexdump(tag, data, (int)len);
    }
    struct libusb_transfer *t = libusb_alloc_transfer(0);
    uint8_t *copy = malloc(len);
    if (!t || !copy) {
        free(copy);
        libusb_free_transfer(t);
        return;
    }
    memcpy(copy, data, len);
    libusb_fill_interrupt_transfer(t, g.devh, ep, copy, (int)len, out_done, NULL, 1000);
    t->flags = LIBUSB_TRANSFER_FREE_BUFFER | LIBUSB_TRANSFER_FREE_TRANSFER;
    int r = libusb_submit_transfer(t);
    if (r < 0) {
        fprintf(stderr, "submit OUT 0x%02x: %s\n", ep, libusb_strerror(r));
        libusb_free_transfer(t);                /* also frees 'copy' (FREE_BUFFER) */
    }
}

static uint8_t next_seq(void) { return g.gip_seq++; }

/* send a GIP packet template, stamping the sequence byte like xpad does */
static void gip_send_tpl(const uint8_t *tpl, size_t len)
{
    uint8_t p[64];
    if (len > sizeof p)
        return;
    memcpy(p, tpl, len);
    p[2] = next_seq();
    usb_send(g.gip_out, p, len);
}

/* Startup sequence the Vader 5S needs (OpenJoystickDriver 37d7-2801 profile; the
 * packets are the standard ones from xpad's Xbox One init tables). */
static const uint8_t HORI_ACK[]  = { 0x01, 0x20, 0x00, 0x09, 0x00, 0x04, 0x20, 0x3a, 0x00, 0x00, 0x00, 0x80, 0x00 };
static const uint8_t LED_ON[]    = { 0x0a, 0x20, 0x00, 0x03, 0x00, 0x01, 0x14 };
static const uint8_t AUTH_DONE[] = { 0x06, 0x20, 0x00, 0x02, 0x01, 0x00 };
static const uint8_t POWER_ON[]  = { 0x05, 0x20, 0x00, 0x01, 0x00 };

static void send_gip_init(void)
{
    g.last_init_ms = now_ms();
    gip_send_tpl(HORI_ACK, sizeof HORI_ACK);    /* pre-ACK the pad's identify message */
    gip_send_tpl(LED_ON, sizeof LED_ON);
    gip_send_tpl(AUTH_DONE, sizeof AUTH_DONE);
    gip_send_tpl(POWER_ON, sizeof POWER_ON);
}

static void send_init(void)
{
    if (g.listen_only)
        return;
    if (g.gip_out)
        send_gip_init();
    if (g.x360_out) {
        static const uint8_t led[] = { 0x01, 0x03, 0x06 };   /* player-1 LED steady */
        usb_send(g.x360_out, led, sizeof led);
    }
}

/* generic GIP ACK, same shape as xpad's ACK of the guide-button message */
static void gip_ack(uint8_t cmd, uint8_t opt, uint8_t seq, uint8_t plen)
{
    uint8_t ack[13] = { 0x01, 0x20, seq, 0x09, 0x00, cmd,
                        (uint8_t)(opt & ~0x10), plen, 0, 0, 0, 0, 0 };
    usb_send(g.gip_out, ack, sizeof ack);
}

/* ------------------------------------------------------- rumble / force feedback */

static void rumble_send(uint32_t strong, uint32_t weak)
{
    if (g.gip_out) {
        /* 09 00 seq 09 | 00 mask LT RT strong weak on off repeat ; strength 0..100 */
        uint8_t trig = g.trigger_rumble ? (uint8_t)(((strong + weak) / 2u) * 100u / 65535u) : 0;
        uint8_t p[13] = { 0x09, 0x00, next_seq(), 0x09, 0x00, 0x0f, trig, trig,
                          (uint8_t)(strong * 100u / 65535u), (uint8_t)(weak * 100u / 65535u),
                          0xff, 0x00, 0xff };
        usb_send(g.gip_out, p, sizeof p);
    } else if (g.x360_out) {
        uint8_t p[8] = { 0x00, 0x08, 0x00, (uint8_t)(strong >> 8), (uint8_t)(weak >> 8), 0, 0, 0 };
        usb_send(g.x360_out, p, sizeof p);
    }
}

/* Mix all playing effects, send a rumble packet if the result changed (at most every
 * 4 ms: the OUT endpoint is only serviced every 4 ms). Returns ms until the next time
 * we must wake up for force feedback, or -1. */
static int ff_update(uint64_t now)
{
    uint64_t s = 0, w = 0;
    int64_t next = -1;

    for (int i = 0; i < FF_SLOTS; i++) {
        struct ff_slot *f = &g.ff[i];
        if (!f->valid || !f->playing)
            continue;
        if (now < f->start_ms) {
            int64_t d = (int64_t)(f->start_ms - now);
            if (next < 0 || d < next)
                next = d;
            continue;
        }
        if (f->stop_ms && now >= f->stop_ms) {
            f->playing = 0;
            continue;
        }
        s += f->e.u.rumble.strong_magnitude;
        w += f->e.u.rumble.weak_magnitude;
        if (f->stop_ms) {
            int64_t d = (int64_t)(f->stop_ms - now);
            if (next < 0 || d < next)
                next = d;
        }
    }
    if (s > 0xffff) s = 0xffff;
    if (w > 0xffff) w = 0xffff;
    s = s * g.ff_gain / 0xffff;
    w = w * g.ff_gain / 0xffff;

    if ((uint32_t)s != g.rumble_sent_s || (uint32_t)w != g.rumble_sent_w) {
        uint64_t since = now - g.rumble_last_ms;
        if (g.rumble_last_ms == 0 || since >= 4) {
            rumble_send((uint32_t)s, (uint32_t)w);
            g.rumble_sent_s = (uint32_t)s;
            g.rumble_sent_w = (uint32_t)w;
            g.rumble_last_ms = now ? now : 1;
        } else {
            int64_t d = (int64_t)(4 - since);
            if (next < 0 || d < next)
                next = d;
        }
    }
    return (int)next;
}

/* Service uinput force-feedback requests. A game blocks inside its upload ioctl until
 * we answer, so this must run promptly (the main loop polls this fd). */
static void ff_pump(void)
{
    struct input_event ev;
    while (read(g.ufd, &ev, sizeof ev) == (ssize_t)sizeof ev) {
        if (ev.type == EV_UINPUT && ev.code == UI_FF_UPLOAD) {
            struct uinput_ff_upload up;
            memset(&up, 0, sizeof up);
            up.request_id = ev.value;
            if (ioctl(g.ufd, UI_BEGIN_FF_UPLOAD, &up) == 0) {
                int id = up.effect.id;
                if (id >= 0 && id < FF_SLOTS && up.effect.type == FF_RUMBLE) {
                    g.ff[id].e = up.effect;
                    g.ff[id].valid = 1;
                    up.retval = 0;
                } else {
                    up.retval = -EINVAL;
                }
                ioctl(g.ufd, UI_END_FF_UPLOAD, &up);
            }
        } else if (ev.type == EV_UINPUT && ev.code == UI_FF_ERASE) {
            struct uinput_ff_erase er;
            memset(&er, 0, sizeof er);
            er.request_id = ev.value;
            if (ioctl(g.ufd, UI_BEGIN_FF_ERASE, &er) == 0) {
                if (er.effect_id < FF_SLOTS) {
                    g.ff[er.effect_id].valid = 0;
                    g.ff[er.effect_id].playing = 0;
                }
                er.retval = 0;
                ioctl(g.ufd, UI_END_FF_ERASE, &er);
            }
        } else if (ev.type == EV_FF) {
            if (ev.code == FF_GAIN) {
                g.ff_gain = (uint32_t)ev.value & 0xffff;
            } else if (ev.code < FF_SLOTS && g.ff[ev.code].valid) {
                struct ff_slot *f = &g.ff[ev.code];
                if (ev.value > 0) {
                    uint64_t now = now_ms();
                    f->playing = 1;
                    f->start_ms = now + f->e.replay.delay;
                    f->stop_ms = f->e.replay.length
                        ? f->start_ms + (uint64_t)f->e.replay.length * (uint64_t)ev.value : 0;
                } else {
                    f->playing = 0;
                }
            }
        }
    }
    ff_update(now_ms());
}

/* ---------------------------------------------------------- packet parsers */

static void learn_extra(const uint8_t *d, int n)
{
    static uint8_t prev[256];
    static int prev_n = -1;
    if (n > 256)
        n = 256;
    if (prev_n < 0) {
        fprintf(stderr, "[learn] 0x20 reports are %d bytes (%d beyond the 18-byte core)\n", n, n - 18);
        if (n <= 18)
            fprintf(stderr, "[learn] no extra bytes: paddles/Share may be remapped on-board, or sent in "
                            "another command (watch for 'unknown GIP command' lines)\n");
        memcpy(prev, d, (size_t)n);
        prev_n = n;
        return;
    }
    if (n != prev_n)
        fprintf(stderr, "[learn] report length changed %d -> %d\n", prev_n, n);
    int m = n < prev_n ? n : prev_n;
    for (int i = 4; i < m; i++) {
        if (i > 4 && i < 18)
            continue;                           /* known analog/button bytes */
        uint8_t x = (uint8_t)(d[i] ^ prev[i]);
        if (i == 4)
            x &= 0x03;                          /* only the unmapped bits of byte 4 */
        for (int b = 0; b < 8; b++)
            if (x & (1 << b))
                fprintf(stderr, "[learn] d[%d] bit %d %s   (try: -m %d:0x%02x:NAME)\n", i, b,
                        ((d[i] >> b) & 1) ? "PRESSED" : "released", i, 1 << b);
    }
    memcpy(prev, d, (size_t)n);
    prev_n = n;
}

static void learn_cmd(const uint8_t *d, int n)
{
    static uint8_t last[256][64];
    static uint8_t lastn[256];
    static int prints;
    int c = d[0], m = n > 64 ? 64 : n;
    if (lastn[c] == m && !memcmp(last[c], d, (size_t)m))
        return;
    memcpy(last[c], d, (size_t)m);
    lastn[c] = (uint8_t)m;
    if (prints++ < 200)
        hexdump("[learn] unknown GIP command (changed)", d, n);
}

static int handle_gip(const uint8_t *d, int n)
{
    if (n == 1 && d[0] == 0x00)
        return 1;                                   /* 1-byte dummy packet some pads send during auth */
    if (n < 4)
        return 0;
    uint8_t cmd = d[0], opt = d[1], seq = d[2], plen = d[3];

    if ((opt & 0x10) && !g.listen_only) {
        if (cmd == 0x04)
            gip_send_tpl(HORI_ACK, sizeof HORI_ACK);   /* the pad waits for this after identify */
        else
            gip_ack(cmd, opt, seq, plen);
    }

    switch (cmd) {
    case 0x20: {                                    /* input report */
        if (n < 18)
            return 0;
        struct evbuf b = { .n = 0 };
        ev_key(&b, BTN_START,  d[4] & 0x04);
        ev_key(&b, BTN_SELECT, d[4] & 0x08);
        ev_key(&b, BTN_A,      d[4] & 0x10);
        ev_key(&b, BTN_B,      d[4] & 0x20);
        ev_key(&b, BTN_X,      d[4] & 0x40);        /* BTN_X == BTN_NORTH: what xpad reports for Xbox X */
        ev_key(&b, BTN_Y,      d[4] & 0x80);        /* BTN_Y == BTN_WEST */
        ev_abs(&b, ABS_HAT0X,  !!(d[5] & 0x08) - !!(d[5] & 0x04));
        ev_abs(&b, ABS_HAT0Y,  !!(d[5] & 0x02) - !!(d[5] & 0x01));
        ev_key(&b, BTN_TL,     d[5] & 0x10);
        ev_key(&b, BTN_TR,     d[5] & 0x20);
        ev_key(&b, BTN_THUMBL, d[5] & 0x40);
        ev_key(&b, BTN_THUMBR, d[5] & 0x80);
        ev_abs(&b, ABS_Z,      clamp1023(le16u(d + 6)));
        ev_abs(&b, ABS_RZ,     clamp1023(le16u(d + 8)));
        ev_abs(&b, ABS_X,      le16s(d + 10));
        ev_abs(&b, ABS_Y,      ~le16s(d + 12));
        ev_abs(&b, ABS_RX,     le16s(d + 14));
        ev_abs(&b, ABS_RY,     ~le16s(d + 16));
        for (int i = 0; i < g.num_maps; i++)        /* user-defined extra buttons (-m) */
            ev_key(&b, g.maps[i].code, g.maps[i].offset < n && (d[g.maps[i].offset] & g.maps[i].mask));
        ev_flush(&b);
        if (g.learn)
            learn_extra(d, n);
        return 1;
    }
    case 0x07: {                                    /* Guide ("virtual key") */
        if (n < 5)
            return 0;
        struct evbuf b = { .n = 0 };
        ev_key(&b, BTN_MODE, d[4] & 0x01);
        ev_flush(&b);
        return 1;
    }
    case 0x02:                                      /* Announce: (re)send the startup sequence */
        note_few("GIP announce -> startup sequence", d, n);
        if (!g.listen_only && g.gip_out && g.announces < 3 && now_ms() - g.last_init_ms > 300) {
            g.announces++;
            send_gip_init();
        }
        return 1;
    case 0x04:
        note_few("GIP identify", d, n);
        return 1;
    case 0x01: case 0x03: case 0x05: case 0x06: case 0x0a:
        note_few("GIP system message", d, n);
        return 1;
    default:
        if (g.learn)
            learn_cmd(d, n);
        return 0;
    }
}

static int handle_x360(const uint8_t *d, int n)
{
    if (n < 14 || d[0] != 0x00 || d[1] != 0x14)
        return 0;
    struct evbuf b = { .n = 0 };
    ev_abs(&b, ABS_HAT0Y,  !!(d[2] & 0x02) - !!(d[2] & 0x01));
    ev_abs(&b, ABS_HAT0X,  !!(d[2] & 0x08) - !!(d[2] & 0x04));
    ev_key(&b, BTN_START,  d[2] & 0x10);
    ev_key(&b, BTN_SELECT, d[2] & 0x20);
    ev_key(&b, BTN_THUMBL, d[2] & 0x40);
    ev_key(&b, BTN_THUMBR, d[2] & 0x80);
    ev_key(&b, BTN_TL,     d[3] & 0x01);
    ev_key(&b, BTN_TR,     d[3] & 0x02);
    ev_key(&b, BTN_MODE,   d[3] & 0x04);
    ev_key(&b, BTN_A,      d[3] & 0x10);
    ev_key(&b, BTN_B,      d[3] & 0x20);
    ev_key(&b, BTN_X,      d[3] & 0x40);
    ev_key(&b, BTN_Y,      d[3] & 0x80);
    ev_abs(&b, ABS_Z,      d[4] * 4);
    ev_abs(&b, ABS_RZ,     d[5] * 4);
    ev_abs(&b, ABS_X,      le16s(d + 6));
    ev_abs(&b, ABS_Y,      ~le16s(d + 8));
    ev_abs(&b, ABS_RX,     le16s(d + 10));
    ev_abs(&b, ABS_RY,     ~le16s(d + 12));
    ev_flush(&b);
    return 1;
}

/* Flydigi "V2" vendor HID report (Vader 5 Pro / Apex 5; starts 5A A5 EF). Not used by
 * the 5S, whose descriptor has no HID interface; kept for other Flydigi models. Bytes
 * 13/14 match SDL's driver; the byte 11/12 layout and stick Y inversion are unverified. */
static int handle_flydigi_v2(const uint8_t *buf, int n)
{
    const uint8_t *d;
    if (n >= 32 && buf[0] == 0x5A && buf[1] == 0xA5 && buf[2] == 0xEF)
        d = buf;
    else if (n >= 33 && buf[1] == 0x5A && buf[2] == 0xA5 && buf[3] == 0xEF)
        d = buf + 1;
    else
        return 0;

    struct evbuf b = { .n = 0 };
    ev_abs(&b, ABS_X,  le16s(d + 3));
    ev_abs(&b, ABS_Y,  ~le16s(d + 5));
    ev_abs(&b, ABS_RX, le16s(d + 7));
    ev_abs(&b, ABS_RY, ~le16s(d + 9));
    ev_abs(&b, ABS_HAT0Y, !!(d[11] & 0x04) - !!(d[11] & 0x01));
    ev_abs(&b, ABS_HAT0X, !!(d[11] & 0x02) - !!(d[11] & 0x08));
    ev_key(&b, BTN_A,      d[11] & 0x10);
    ev_key(&b, BTN_B,      d[11] & 0x20);
    ev_key(&b, BTN_SELECT, d[11] & 0x40);
    ev_key(&b, BTN_X,      d[11] & 0x80);
    ev_key(&b, BTN_Y,      d[12] & 0x01);
    ev_key(&b, BTN_START,  d[12] & 0x02);
    ev_key(&b, BTN_TL,     d[12] & 0x04);
    ev_key(&b, BTN_TR,     d[12] & 0x08);
    ev_key(&b, BTN_THUMBL, d[12] & 0x40);
    ev_key(&b, BTN_THUMBR, d[12] & 0x80);
    ev_key(&b, BTN_TRIGGER_HAPPY1, d[13] & 0x04);   /* M1 */
    ev_key(&b, BTN_TRIGGER_HAPPY2, d[13] & 0x08);   /* M2 */
    ev_key(&b, BTN_TRIGGER_HAPPY3, d[13] & 0x10);   /* M3 */
    ev_key(&b, BTN_TRIGGER_HAPPY4, d[13] & 0x20);   /* M4 */
    ev_key(&b, BTN_TRIGGER_HAPPY5, d[13] & 0x01);   /* C  */
    ev_key(&b, BTN_TRIGGER_HAPPY6, d[13] & 0x02);   /* Z  */
    ev_key(&b, BTN_TRIGGER_HAPPY7, d[13] & 0x40);   /* LM */
    ev_key(&b, BTN_TRIGGER_HAPPY8, d[13] & 0x80);   /* RM */
    ev_key(&b, BTN_MODE,           d[14] & 0x08);
    ev_key(&b, BTN_TRIGGER_HAPPY9, d[14] & 0x01);
    ev_abs(&b, ABS_Z,  d[15] * 4);
    ev_abs(&b, ABS_RZ, d[16] * 4);
    ev_flush(&b);
    return 1;
}

static int dispatch(const struct ep_ctx *ep, const uint8_t *d, int n)
{
    switch (ep->proto) {
    case PROTO_GIP:  return handle_gip(d, n);
    case PROTO_X360: return handle_x360(d, n);
    default:         return handle_flydigi_v2(d, n);
    }
}

/* ---------------------------------------------- pipelined async IN reading */

static void LIBUSB_CALL in_done(struct libusb_transfer *t);

static int submit_slot(struct urb_slot *s)
{
    struct ep_ctx *ep = s->ep;
    libusb_fill_interrupt_transfer(s->xfer, g.devh, ep->addr, s->buf, ep->maxpkt, in_done, s, 0);
    int r = libusb_submit_transfer(s->xfer);
    if (r < 0) {
        if (r == LIBUSB_ERROR_NO_DEVICE)
            g.disconnected = 1;
        else
            fprintf(stderr, "submit IN 0x%02x: %s\n", ep->addr, libusb_strerror(r));
        return r;
    }
    s->active = 1;
    ep->inflight++;
    g.total_inflight++;
    return 0;
}

static void LIBUSB_CALL in_done(struct libusb_transfer *t)
{
    struct urb_slot *s = t->user_data;
    struct ep_ctx *ep = s->ep;
    s->active = 0;
    ep->inflight--;
    g.total_inflight--;

    switch (t->status) {
    case LIBUSB_TRANSFER_COMPLETED:
        if (t->actual_length > 0) {
            uint64_t tn = now_ns();
            if (ep->last_ns) {
                uint64_t gap = tn - ep->last_ns;
                if (gap > ep->win_max_gap_ns)
                    ep->win_max_gap_ns = gap;
            }
            ep->last_ns = tn;
            ep->packets++;
            if (g.verbose) {
                char tag[24];
                snprintf(tag, sizeof tag, "IN  0x%02x", ep->addr);
                hexdump(tag, t->buffer, t->actual_length);
            }
            if (!dispatch(ep, t->buffer, t->actual_length)) {
                ep->unparsed++;
                if (!g.verbose && ep->unparsed <= 8) {
                    char tag[48];
                    snprintf(tag, sizeof tag, "IN  0x%02x unparsed (%s)", ep->addr, proto_name[ep->proto]);
                    hexdump(tag, t->buffer, t->actual_length);
                }
            }
        }
        if (g_run && !g.stopping && !ep->needs_restart)
            submit_slot(s);
        break;
    case LIBUSB_TRANSFER_CANCELLED:
        break;
    case LIBUSB_TRANSFER_NO_DEVICE:
        g.disconnected = 1;
        break;
    default:                                    /* STALL / ERROR / OVERFLOW / TIMED_OUT */
        if (++ep->errors <= 3)
            fprintf(stderr, "IN 0x%02x: transfer status %d\n", ep->addr, t->status);
        ep->needs_restart = 1;                  /* main loop cancels the rest, clears the halt, resubmits */
        break;
    }
}

static void start_reads(void)
{
    g.stopping = 0;
    for (int i = 0; i < g.num_eps; i++) {
        struct ep_ctx *ep = &g.eps[i];
        for (int u = 0; u < URB_DEPTH; u++) {
            ep->slots[u].ep = ep;
            if (!ep->slots[u].xfer)
                ep->slots[u].xfer = libusb_alloc_transfer(0);
            if (ep->slots[u].xfer)
                submit_slot(&ep->slots[u]);
        }
    }
}

static void restart_endpoints(void)
{
    for (int i = 0; i < g.num_eps && !g.disconnected; i++) {
        struct ep_ctx *ep = &g.eps[i];
        if (!ep->needs_restart)
            continue;
        if (ep->inflight > 0) {
            if (!ep->cancel_issued) {
                ep->cancel_issued = 1;
                for (int u = 0; u < URB_DEPTH; u++)
                    if (ep->slots[u].active)
                        libusb_cancel_transfer(ep->slots[u].xfer);
            }
            continue;                           /* wait until everything has drained */
        }
        libusb_clear_halt(g.devh, ep->addr);
        ep->needs_restart = 0;
        ep->cancel_issued = 0;
        for (int u = 0; u < URB_DEPTH; u++)
            submit_slot(&ep->slots[u]);
    }
}

static void stop_reads(void)
{
    g.stopping = 1;
    for (int i = 0; i < g.num_eps; i++)
        for (int u = 0; u < URB_DEPTH; u++)
            if (g.eps[i].slots[u].active)
                libusb_cancel_transfer(g.eps[i].slots[u].xfer);
    for (int i = 0; i < 50 && g.total_inflight > 0; i++) {
        struct timeval tv = { 0, 20000 };
        libusb_handle_events_timeout_completed(NULL, &tv, NULL);
    }
    for (int i = 0; i < g.num_eps; i++)
        for (int u = 0; u < URB_DEPTH; u++) {
            if (g.eps[i].slots[u].xfer && !g.eps[i].slots[u].active) {
                libusb_free_transfer(g.eps[i].slots[u].xfer);
                g.eps[i].slots[u].xfer = NULL;
            }
        }
}

/* ------------------------------------------------------------------ uinput */

static void report_nodes(int fd)
{
    char sysname[64] = "";
    if (ioctl(fd, UI_GET_SYSNAME(sizeof sysname - 1), sysname) < 0 || !sysname[0])
        return;
    if (g.fix_perms)
        usleep(500000);                         /* let udev apply its own mode first, or it overrides ours */
    char dirpath[128];
    snprintf(dirpath, sizeof dirpath, "/sys/devices/virtual/input/%.60s", sysname);
    DIR *dir = opendir(dirpath);
    if (!dir)
        return;
    struct dirent *de;
    while ((de = readdir(dir)) != NULL) {
        if (strncmp(de->d_name, "event", 5) && strncmp(de->d_name, "js", 2))
            continue;
        char node[96];
        snprintf(node, sizeof node, "/dev/input/%.60s", de->d_name);
        printf("Virtual pad node: %s\n", node);
        if (g.fix_perms) {
            if (chmod(node, 0666) == 0)
                printf("  chmod 0666 applied\n");
            else
                perror("  chmod");
        }
    }
    closedir(dir);
}

static int uinput_open(void)
{
    int fd = open("/dev/uinput", O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        perror("open /dev/uinput");
        fprintf(stderr, "  (run as root, and make sure the uinput module is loaded: sudo modprobe uinput)\n");
        return -1;
    }
#define UI_CALL(req, val) do { if (ioctl(fd, (req), (val)) < 0) { perror(#req); close(fd); return -1; } } while (0)

    UI_CALL(UI_SET_EVBIT, EV_KEY);
    static const int base_keys[] = {
        BTN_A, BTN_B, BTN_X, BTN_Y, BTN_TL, BTN_TR, BTN_SELECT, BTN_START, BTN_MODE,
        BTN_THUMBL, BTN_THUMBR,
    };
    for (size_t i = 0; i < sizeof base_keys / sizeof base_keys[0]; i++)
        UI_CALL(UI_SET_KEYBIT, base_keys[i]);
    for (int i = 0; i < g.num_maps; i++)
        UI_CALL(UI_SET_KEYBIT, g.maps[i].code);
    if (g.v2_keys)
        for (int k = BTN_TRIGGER_HAPPY1; k <= BTN_TRIGGER_HAPPY9; k++)
            UI_CALL(UI_SET_KEYBIT, k);

    UI_CALL(UI_SET_EVBIT, EV_ABS);
    static const struct { int code, min, max, fuzz, flat; } axes[] = {
        { ABS_X,  -32768, 32767, 16, 128 }, { ABS_Y,  -32768, 32767, 16, 128 },
        { ABS_RX, -32768, 32767, 16, 128 }, { ABS_RY, -32768, 32767, 16, 128 },
        { ABS_Z,  0, 1023, 0, 0 },          { ABS_RZ, 0, 1023, 0, 0 },
        { ABS_HAT0X, -1, 1, 0, 0 },         { ABS_HAT0Y, -1, 1, 0, 0 },
    };
    for (size_t i = 0; i < sizeof axes / sizeof axes[0]; i++) {
        struct uinput_abs_setup as;
        memset(&as, 0, sizeof as);
        as.code = (uint16_t)axes[i].code;
        as.absinfo.minimum = axes[i].min;
        as.absinfo.maximum = axes[i].max;
        as.absinfo.fuzz = axes[i].fuzz;
        as.absinfo.flat = axes[i].flat;
        UI_CALL(UI_SET_ABSBIT, axes[i].code);
        UI_CALL(UI_ABS_SETUP, &as);
    }

    UI_CALL(UI_SET_EVBIT, EV_FF);
    UI_CALL(UI_SET_FFBIT, FF_RUMBLE);
    UI_CALL(UI_SET_FFBIT, FF_GAIN);

    struct uinput_setup us;
    memset(&us, 0, sizeof us);
    us.id.bustype = BUS_USB;
    us.id.vendor  = 0x045e;
    us.id.product = g.elite ? 0x0b00 : 0x028e;      /* Elite Series 2 : Xbox 360 pad */
    us.id.version = 0x0110;
    snprintf(us.name, UINPUT_MAX_NAME_SIZE, "Flydigi Vader 5S (userspace driver)");
    us.ff_effects_max = FF_SLOTS;
    UI_CALL(UI_DEV_SETUP, &us);
    UI_CALL(UI_DEV_CREATE, 0);
#undef UI_CALL
    printf("Virtual pad created as %04x:%04x\n", us.id.vendor, us.id.product);
    report_nodes(fd);
    return fd;
}

/* ------------------------------------------------------------ descriptors */

static enum proto classify(const struct libusb_interface_descriptor *d)
{
    if (d->bInterfaceClass == 0xFF && d->bInterfaceSubClass == 0x47 && d->bInterfaceProtocol == 0xD0)
        return PROTO_GIP;                       /* Xbox One / Xbox-licensed */
    if (d->bInterfaceClass == 0xFF && d->bInterfaceSubClass == 0x5D && d->bInterfaceProtocol == 0x01)
        return PROTO_X360;                      /* Xbox 360 "XInput" pad interface */
    if (d->bInterfaceClass == LIBUSB_CLASS_HID)
        return PROTO_HID;
    return PROTO_UNKNOWN;
}

static const char *speed_name(int s)
{
    switch (s) {
    case LIBUSB_SPEED_LOW:  return "low (1.5 Mb/s)";
    case LIBUSB_SPEED_FULL: return "full (12 Mb/s)";
    case LIBUSB_SPEED_HIGH: return "high (480 Mb/s)";
    case LIBUSB_SPEED_SUPER: case LIBUSB_SPEED_SUPER_PLUS: return "super";
    default: return "unknown";
    }
}

static double max_rate_hz(int speed, int interval)
{
    if (interval < 1)
        return 0;
    if (speed == LIBUSB_SPEED_LOW || speed == LIBUSB_SPEED_FULL)
        return 1000.0 / interval;               /* bInterval is in 1 ms frames */
    if (speed >= LIBUSB_SPEED_HIGH && interval <= 16)
        return 8000.0 / (double)(1 << (interval - 1));
    return 0;
}

static void scan_and_claim(libusb_device *dev, const struct libusb_config_descriptor *cfg)
{
    static const char *types[] = { "control", "isoc", "bulk", "interrupt" };
    int speed = libusb_get_device_speed(dev);
    printf("USB speed: %s\n", speed_name(speed));

    for (int i = 0; i < cfg->bNumInterfaces; i++) {
        const struct libusb_interface *ifc = &cfg->interface[i];

        for (int a = 0; a < ifc->num_altsetting; a++) {
            const struct libusb_interface_descriptor *d = &ifc->altsetting[a];
            printf("Interface %d alt %d: class %02x/%02x/%02x -> %s, %d endpoint(s)\n",
                   d->bInterfaceNumber, d->bAlternateSetting, d->bInterfaceClass,
                   d->bInterfaceSubClass, d->bInterfaceProtocol, proto_name[classify(d)],
                   d->bNumEndpoints);
            for (int k = 0; k < d->bNumEndpoints; k++) {
                const struct libusb_endpoint_descriptor *e = &d->endpoint[k];
                int is_in = e->bEndpointAddress & LIBUSB_ENDPOINT_IN;
                int type = e->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK;
                printf("    EP 0x%02x %-3s %-9s maxpkt %u interval %u", e->bEndpointAddress,
                       is_in ? "IN" : "OUT", types[type], e->wMaxPacketSize & 0x7ff, e->bInterval);
                if (type == LIBUSB_TRANSFER_TYPE_INTERRUPT && max_rate_hz(speed, e->bInterval) > 0)
                    printf("  -> at most ~%.0f %s/s", max_rate_hz(speed, e->bInterval),
                           is_in ? "reports" : "commands");
                putchar('\n');
            }
        }
        if (ifc->num_altsetting < 1)
            continue;

        const struct libusb_interface_descriptor *d = &ifc->altsetting[0];
        enum proto p = classify(d);
        int has_int = 0;
        for (int k = 0; k < d->bNumEndpoints; k++)
            if ((d->endpoint[k].bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_INTERRUPT)
                has_int = 1;
        if (!has_int || p == PROTO_UNKNOWN) {
            printf("  -> interface %d left alone (no interrupt endpoints or unknown protocol)\n", i);
            continue;
        }

        int r = libusb_claim_interface(g.devh, d->bInterfaceNumber);
        if (r < 0) {
            fprintf(stderr, "  -> CLAIM FAILED for interface %d: %s\n", d->bInterfaceNumber, libusb_strerror(r));
            fprintf(stderr, "     Something else owns it. Quit Steam completely, stop other drivers, retry.\n");
            continue;
        }
        printf("  -> claimed interface %d as %s\n", d->bInterfaceNumber, proto_name[p]);
        if (g.num_claimed < MAX_IFACES)
            g.claimed[g.num_claimed++] = d->bInterfaceNumber;
        if (p == PROTO_HID)
            g.v2_keys = 1;

        for (int k = 0; k < d->bNumEndpoints; k++) {
            const struct libusb_endpoint_descriptor *e = &d->endpoint[k];
            if ((e->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_INTERRUPT)
                continue;
            if (e->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                if (g.num_eps < MAX_EPS) {
                    struct ep_ctx *ep = &g.eps[g.num_eps++];
                    memset(ep, 0, sizeof *ep);
                    ep->addr = e->bEndpointAddress;
                    ep->iface = d->bInterfaceNumber;
                    ep->proto = p;
                    ep->maxpkt = (e->wMaxPacketSize & 0x7ff) ? (e->wMaxPacketSize & 0x7ff) : 64;
                    if (ep->maxpkt > sizeof ep->slots[0].buf)
                        ep->maxpkt = sizeof ep->slots[0].buf;
                }
            } else if (p == PROTO_GIP && !g.gip_out) {
                g.gip_out = e->bEndpointAddress;
            } else if (p == PROTO_X360 && !g.x360_out) {
                g.x360_out = e->bEndpointAddress;
            }
        }
    }
}

/* ------------------------------------------------------ USB session handling */

static int usb_open_session(void)
{
    g.devh = libusb_open_device_with_vid_pid(NULL, g.vid, g.pid);
    if (!g.devh)
        return -1;
    libusb_set_auto_detach_kernel_driver(g.devh, 1);

    if (g.usb_reset) {
        g.usb_reset = 0;
        printf("Resetting %04x:%04x ...\n", g.vid, g.pid);
        int rr = libusb_reset_device(g.devh);
        if (rr == LIBUSB_ERROR_NOT_FOUND) {     /* it re-enumerated: our handle is stale */
            libusb_close(g.devh);
            g.devh = NULL;
            for (int i = 0; i < 25 && g_run && !g.devh; i++) {
                usleep(200000);
                g.devh = libusb_open_device_with_vid_pid(NULL, g.vid, g.pid);
            }
            if (!g.devh) {
                fprintf(stderr, "Controller did not come back after the reset.\n");
                return -1;
            }
            libusb_set_auto_detach_kernel_driver(g.devh, 1);
        } else if (rr < 0) {
            fprintf(stderr, "libusb_reset_device: %s (continuing)\n", libusb_strerror(rr));
        }
        usleep(300000);
    }

    libusb_device *dev = libusb_get_device(g.devh);
    struct libusb_device_descriptor dd;
    if (libusb_get_device_descriptor(dev, &dd) == 0) {
        unsigned char name[128] = "";
        if (dd.iProduct)
            libusb_get_string_descriptor_ascii(g.devh, dd.iProduct, name, sizeof name);
        printf("Opened %04x:%04x  bcdDevice %04x  \"%s\"\n", dd.idVendor, dd.idProduct,
               dd.bcdDevice, (char *)name);
    }

    struct libusb_config_descriptor *cfg;
    int r = libusb_get_active_config_descriptor(dev, &cfg);
    if (r < 0) {
        fprintf(stderr, "get_active_config_descriptor: %s\n", libusb_strerror(r));
        return -2;
    }
    g.gip_out = g.x360_out = 0;
    g.num_eps = g.num_claimed = 0;
    g.announces = 0;
    g.gip_seq = 0;
    g.disconnected = 0;
    scan_and_claim(dev, cfg);
    libusb_free_config_descriptor(cfg);

    if (g.num_eps == 0) {
        fprintf(stderr, "No usable interrupt IN endpoint was claimed - nothing to read. See the messages above.\n");
        return -2;
    }
    return 0;
}

static void usb_close_session(void)
{
    if (g.devh) {
        if (!g.disconnected && !g.listen_only && g.gip_out && (g.rumble_sent_s || g.rumble_sent_w)) {
            uint8_t p[13] = { 0x09, 0x00, next_seq(), 0x09, 0x00, 0x0f, 0, 0, 0, 0, 0xff, 0x00, 0xff };
            int sent = 0;
            libusb_interrupt_transfer(g.devh, g.gip_out, p, sizeof p, &sent, 100);   /* motors off */
        }
        for (int i = 0; i < g.num_claimed; i++)
            libusb_release_interface(g.devh, g.claimed[i]);
        libusb_close(g.devh);
        g.devh = NULL;
    }
    g.num_claimed = 0;
    g.rumble_sent_s = g.rumble_sent_w = 0;
    g.rumble_last_ms = 0;
}

/* ----------------------------------------------------------------- poll loop */

static void LIBUSB_CALL pollfd_added(int fd, short events, void *ud) { (void)fd; (void)events; (void)ud; g.pfd_dirty = 1; }
static void LIBUSB_CALL pollfd_removed(int fd, void *ud) { (void)fd; (void)ud; g.pfd_dirty = 1; }

static void rebuild_pollfds(void)
{
    const struct libusb_pollfd **list = libusb_get_pollfds(NULL);
    int n = 0;
    if (list)
        while (list[n])
            n++;
    struct pollfd *np = realloc(g.pfds, (size_t)(n + 1) * sizeof *np);
    if (!np) {
        libusb_free_pollfds(list);
        return;
    }
    g.pfds = np;
    g.pfds[0].fd = g.ufd;
    g.pfds[0].events = POLLIN;
    g.pfds[0].revents = 0;
    for (int i = 0; i < n; i++) {
        g.pfds[i + 1].fd = list[i]->fd;
        g.pfds[i + 1].events = list[i]->events;
        g.pfds[i + 1].revents = 0;
    }
    g.npfds = n + 1;
    libusb_free_pollfds(list);
    g.pfd_dirty = 0;
}

static void print_stats(double dt)
{
    fprintf(stderr, "[stats]");
    for (int i = 0; i < g.num_eps; i++) {
        struct ep_ctx *ep = &g.eps[i];
        unsigned long delta = ep->packets - ep->last_packets;
        ep->last_packets = ep->packets;
        fprintf(stderr, "  0x%02x(%s): %lu pkts, %.0f Hz, max gap %.1f ms, %lu unparsed", ep->addr,
                proto_name[ep->proto], ep->packets, delta / dt,
                (double)ep->win_max_gap_ns / 1e6, ep->unparsed);
        ep->win_max_gap_ns = 0;
    }
    fputc('\n', stderr);
}

static void run_session(void)
{
    send_init();
    start_reads();
    printf("Running%s. Ctrl+C to stop. Press buttons now; the pad reports at most ~every 2 ms, so the Hz\n"
           "figure below is only meaningful while you are moving a stick.\n",
           g.listen_only ? " (listen-only)" : "");

    uint64_t last_stats = now_ms(), started = last_stats;
    int warned = 0, retries = 0;

    while (g_run && !g.disconnected) {
        if (g.pfd_dirty)
            rebuild_pollfds();

        int timeout = 100;
        int nxt = ff_update(now_ms());
        if (nxt >= 0 && nxt < timeout)
            timeout = nxt;
        struct timeval tv;
        if (libusb_get_next_timeout(NULL, &tv) == 1) {
            int ms = (int)(tv.tv_sec * 1000 + (tv.tv_usec + 999) / 1000);
            if (ms < timeout)
                timeout = ms;
        }

        int pr = poll(g.pfds, (nfds_t)g.npfds, timeout);
        if (pr < 0 && errno != EINTR) {
            perror("poll");
            break;
        }
        if (pr > 0 && (g.pfds[0].revents & POLLIN))
            ff_pump();

        struct timeval zero = { 0, 0 };
        int r = libusb_handle_events_timeout_completed(NULL, &zero, NULL);
        if (r < 0 && r != LIBUSB_ERROR_INTERRUPTED && r != LIBUSB_ERROR_TIMEOUT) {
            fprintf(stderr, "libusb_handle_events: %s\n", libusb_strerror(r));
            break;
        }
        restart_endpoints();

        uint64_t now = now_ms();
        if (!g.verbose && now - last_stats >= 2000) {
            unsigned long total = 0;
            for (int i = 0; i < g.num_eps; i++)
                total += g.eps[i].packets;
            print_stats((double)(now - last_stats) / 1000.0);
            last_stats = now;
            if (total == 0 && !g.listen_only && retries < 5) {
                retries++;
                fprintf(stderr, "[retry %d/5] pad is silent, resending the startup sequence\n", retries);
                send_init();
            }
            if (total == 0 && !warned && now - started >= 4000) {
                warned = 1;
                fprintf(stderr,
                        "[hint] No packets yet. Try -r (USB reset), -n (listen only, in case something else already\n"
                        "       initialised the pad), press the Guide button, and make sure Steam is fully closed.\n");
            }
        }
    }

    stop_reads();
}

static void wait_with_ff(int ms)
{
    if (g.ufd < 0) {
        usleep((useconds_t)ms * 1000);
        return;
    }
    struct pollfd p = { .fd = g.ufd, .events = POLLIN };
    if (poll(&p, 1, ms) > 0 && (p.revents & POLLIN))
        ff_pump();                              /* keep answering games' FF uploads while unplugged */
}

/* ------------------------------------------------------------------ options */

static int key_by_name(const char *s)
{
    static const struct { const char *n; int code; } names[] = {
        { "share", KEY_RECORD },
        { "paddle1", BTN_GRIPR }, { "paddle2", BTN_GRIPR2 }, { "paddle3", BTN_GRIPL }, { "paddle4", BTN_GRIPL2 },
        { "extra1", BTN_TRIGGER_HAPPY1 }, { "extra2", BTN_TRIGGER_HAPPY2 },
        { "extra3", BTN_TRIGGER_HAPPY3 }, { "extra4", BTN_TRIGGER_HAPPY4 },
        { "extra5", BTN_TRIGGER_HAPPY5 }, { "extra6", BTN_TRIGGER_HAPPY6 },
        { "extra7", BTN_TRIGGER_HAPPY7 }, { "extra8", BTN_TRIGGER_HAPPY8 },
    };
    for (size_t i = 0; i < sizeof names / sizeof names[0]; i++)
        if (!strcmp(s, names[i].n))
            return names[i].code;
    if (isdigit((unsigned char)s[0])) {
        char *end;
        long v = strtol(s, &end, 0);
        if (*end == '\0' && v > 0 && v < KEY_CNT)
            return (int)v;
    }
    return -1;
}

static int parse_map(const char *arg)
{
    char buf[96];
    if (g.num_maps >= MAX_MAPS || strlen(arg) >= sizeof buf)
        return -1;
    strcpy(buf, arg);
    char *c1 = strchr(buf, ':');
    char *c2 = c1 ? strchr(c1 + 1, ':') : NULL;
    if (!c1 || !c2)
        return -1;
    *c1 = *c2 = '\0';
    char *end;
    long off = strtol(buf, &end, 0);
    if (*end || off < 4 || off > 63)
        return -1;
    long mask = strtol(c1 + 1, &end, 0);
    if (*end || mask < 1 || mask > 255)
        return -1;
    int code = key_by_name(c2 + 1);
    if (code < 0)
        return -1;
    g.maps[g.num_maps].offset = (int)off;
    g.maps[g.num_maps].mask = (uint8_t)mask;
    g.maps[g.num_maps].code = code;
    g.num_maps++;
    return 0;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [-v] [-n] [-e] [-p] [-t] [-r] [-R] [-w] [-L] [-m OFFSET:MASK:NAME]... [-d VID:PID]\n"
            "  -m names: share, paddle1..4, extra1..8, or a key code such as 0x2c4\n", argv0);
}

int main(int argc, char **argv)
{
    int opt, rc = 1, first = 1;
    g.vid = 0x37d7;
    g.pid = 0x2801;

    while ((opt = getopt(argc, argv, "vneptRrwLm:d:h")) != -1) {
        switch (opt) {
        case 'v': g.verbose = 1; break;
        case 'n': g.listen_only = 1; break;
        case 'e': g.elite = 1; break;
        case 't': g.trigger_rumble = 1; break;
        case 'p': g.fix_perms = 1; break;
        case 'r': g.usb_reset = 1; break;
        case 'R': g.rt = 1; break;
        case 'w': g.wait_dev = 1; break;
        case 'L': g.learn = 1; break;
        case 'm':
            if (parse_map(optarg) < 0) {
                fprintf(stderr, "bad -m argument '%s' (expected OFFSET:MASK:NAME, offset 4..63, mask 1..255)\n", optarg);
                return 2;
            }
            break;
        case 'd':
            if (sscanf(optarg, "%hx:%hx", &g.vid, &g.pid) != 2) {
                usage(argv[0]);
                return 2;
            }
            break;
        default:
            usage(argv[0]);
            return 2;
        }
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    if (g.rt) {
        struct sched_param sp = { .sched_priority = 20 };
        if (sched_setscheduler(0, SCHED_FIFO, &sp) < 0)
            perror("sched_setscheduler (-R)");
        else
            printf("Real-time scheduling enabled (SCHED_FIFO 20)\n");
        if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0)
            perror("mlockall (-R)");
    }

    int r = libusb_init(NULL);
    if (r < 0) {
        fprintf(stderr, "libusb_init: %s\n", libusb_strerror(r));
        return 1;
    }
    libusb_set_pollfd_notifiers(NULL, pollfd_added, pollfd_removed, NULL);

    int announced_wait = 0;
    while (g_run) {
        int ur = usb_open_session();
        if (ur != 0) {
            if (g.devh)
                usb_close_session();
            if (first && !g.wait_dev) {
                if (ur == -1)
                    fprintf(stderr, "Device %04x:%04x not found or not accessible (check `lsusb`, run as root; -w waits for it).\n",
                            g.vid, g.pid);
                goto out;
            }
            if (!announced_wait) {
                fprintf(stderr, "Waiting for the controller...\n");
                announced_wait = 1;
            }
            wait_with_ff(500);
            continue;
        }
        announced_wait = 0;

        if (g.ufd < 0) {
            g.ufd = uinput_open();
            if (g.ufd < 0) {
                usb_close_session();
                goto out;
            }
        }
        first = 0;
        g.pfd_dirty = 1;
        run_session();
        usb_close_session();
        release_all();
        if (g_run)
            fprintf(stderr, "Controller disconnected; waiting for it to come back...\n");
    }
    rc = 0;

out:
    if (g.ufd >= 0) {
        ioctl(g.ufd, UI_DEV_DESTROY);
        close(g.ufd);
    }
    free(g.pfds);
    libusb_exit(NULL);
    return rc;
}
