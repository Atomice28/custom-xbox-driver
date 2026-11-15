# Flydigi Vader 5S userspace driver for Linux (Bazzite)

Gemini's Argon 4, along with Claude's Sonnet 5.5, created a small userspace driver that reads the Flydigi Vader 5S (USB `37d7:2801`, Xbox-licensed, GIP protocol)
over libusb and exposes it to Steam and games as a virtual gamepad through `uinput`, including rumble.

This code was made by them with the help of me.

> **Status: Experimentally Goddamn Volatile.** Built from public protocol notes and unit-tested with captured and synthetic
> packets. Verification on real hardware is in progress, so expect rough edges. sowwy

## What's new in v5

**Startup and protocol**
- Uses the startup sequence this pad needs: Hori-style ACK, LED on, auth done, power on. It also ACKs the
  pad's identify message. Earlier versions skipped these steps, which is the likely reason the pad stayed silent.
- Re-sends the startup sequence on a GIP announce packet (with a 300 ms guard) and retries up to 5 times while
  the pad is completely silent.
- Guide button (GIP command `0x07`) is handled; 1-byte dummy packets are ignored quietly.

**Performance**
- Pipelined asynchronous reads (4 transfers per endpoint) with proper stall recovery, driven by a `poll()` loop.
- Delta-only `uinput` writes: one `write()` per report, and none when nothing changed.
- No work in the hot path other than parsing; stats and hex dumps are optional.
- Optional real-time scheduling (`-R`).

**Rumble / force feedback**
- Honours effect duration, delay, gain and overlapping effects; stops motors on exit.
- Updates are limited to the pad's 4 ms output interval and sent only when the level changes.
- `-t` mirrors the game's rumble onto the two trigger (impulse) motors.

**Robustness**
- `-w` waits for the controller and survives unplugging; buttons are released on disconnect.
- `-r` resets the pad and re-opens it if it re-enumerates.
- Every USB / ioctl error is reported.

**Mapping fixes**
- Xbox X/Y use `BTN_X`/`BTN_Y` (they were swapped), the D-pad is a hat, and ACKs are only sent for GIP packets.
- Extra buttons (paddles, Share) are **not guessed**: use learn mode (`-L`) to find the byte/bit, then map it
  with `-m`.

## Build (Bazzite)

Build inside a distrobox that has `gcc` and `libusb-1.0-0-dev`, then run on the host:

```bash
distrobox enter ubuntu-driver -- gcc -O2 -Wall -Wextra -o vader5s_driver vader5s_driver.c -lusb-1.0
```

## Run

Quit Steam completely first, then:

```bash
sudo modprobe uinput
sudo ./vader5s_driver -L
```

| Flag | Meaning |
|------|---------|
| `-v` | hex-dump every packet in both directions (chatty) |
| `-n` | listen only: send nothing to the pad |
| `-e` | present as Xbox Elite Series 2 (`045e:0b00`) instead of Xbox 360 (`045e:028e`) |
| `-p` | `chmod 0666` the virtual pad's `/dev/input` nodes (a udev rule is cleaner) |
| `-t` | mirror game rumble onto the trigger motors |
| `-r` | USB-reset the controller before claiming it |
| `-R` | real-time scheduling + `mlockall` |
| `-w` | wait for the controller to appear / reappear |
| `-L` | learn mode: print which byte/bit changes when you press extra buttons |
| `-m O:M:N` | map bit mask `M` of byte `O` of the GIP input report to button `N` (`share`, `paddle1..4`, `extra1..8`, or a key code) |
| `-d V:P` | USB id to open (default `37d7:2801`) |

Example: run with `-L`, press a paddle, and if it prints `d[22] bit 0 PRESSED`, then use
`sudo ./vader5s_driver -m 22:0x01:paddle1`.

### Steam permissions

If Steam cannot open the virtual pad, either run with `-p` or add a udev rule in
`/etc/udev/rules.d/99-vader5s.rules`:

```
SUBSYSTEM=="input", ATTRS{name}=="Flydigi Vader 5S (userspace driver)", TAG+="uaccess"
```

## Reading the output

- `[stats]` shows per-endpoint packets, measured reports per second and the worst gap between reports.
  The rate is only meaningful while you move a stick.
- The descriptor dump prints the USB speed and each endpoint's polling interval.
  The 5S's input endpoint reported a 2 ms interval on a full-speed link in a public lsusb dump, which limits it to
  about 500 reports/s. Some reviews quote 1000 Hz, so check your own `[stats]` line.
- `[hint] No packets yet` means the handshake did not complete. Try `-r`, try `-n`, press the Guide button, and make
  sure Steam is fully closed.

## Known limitations

- The orange RGB light strip is **not** driven. It is controlled by Flydigi's vendor protocol, which is not
  documented; it needs a USB capture from Flydigi Space Station for Xbox on Windows.
- Paddles, C/Z and Share are not mapped by default; their report layout is unconfirmed for this pad.
- Rumble only drives the main motors and, with `-t`, a mirrored trigger level. Linux's rumble API has no separate
  trigger channel.
- My patience is revealing itself to be a now-known limitation.  
- Not yet verified end-to-end on hardware.

## References

- paroj/xpad issue [#363](https://github.com/paroj/xpad/issues/363) and PR [#368](https://github.com/paroj/xpad/pull/368): Vader 5S handshake behaviour and USB descriptors
- OpenJoystickDriver PR [#40](https://github.com/xsyetopz/OpenJoystickDriver/pull/40): the startup packet sequence for `37d7:2801`
