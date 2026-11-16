# Flydigi Vader 5S userspace driver for Linux (Bazzite)

Gemini's Argon 4, along with Claude's Sonnet 5.5, created a small userspace driver that reads the Flydigi Vader 5S (USB `37d7:2801`, Xbox-licensed, GIP protocol) over libusb and exposes it to Steam and games as a virtual gamepad through `uinput`, including rumble.

This code was made by them with the help of me.

> **Status: Miraculously Working (Formerly Experimentally Goddamn Volatile).** Built from public protocol notes, free Claude tokens, my axe and now **verified end-to-end on real hardware**. It actually works. yippee.

---

## Hardware Verification: What Works & What Doesn't

Tested on Asus ROG Ally X running Bazzite:

* **Standard Xbox Controls (A, B, X, Y, D-Pad, LB/RB, LT/RT, Thumbsticks, Start, Select, Guide):** 100% working.
* **Share Button:** Works! Mapped via `-m 18:0x01:share` (hardcoded directly in `xbox_driver.c`).
* **Back Paddles (M1–M4):** Work out of the box! They trigger whatever standard Xbox buttons they are mapped to in the controller's onboard hardware profile (and toggle a general activity flag at `d[4] bit 1`).
* **The `C` Button:** Surprisingly works out of the box!
* **The `Z` Button:** Does **not** work right now. Unlike `C`, `Z` has decided to go on strike and doesn't register an independent output yet.
* **Grip Rumble & Impulse Trigger Rumble (`-t`):** Works with a custom **quadratic ($x^2$) response curve**.
* **Polling Rate:** Runs at ~250 Hz (`246 Hz` measured) out of the box on default Xbox firmware; supports up to 500 Hz if unlocked via Flydigi Space Station on Windows.

---

## What's New in V5

**Startup and protocol**
- Uses the startup sequence this pad needs: Hori-style ACK, LED on, auth done, power on. It also ACKs the pad's identify message. Earlier versions skipped these steps, which is why the pad just sat there, staring at me.
- Re-sends the startup sequence on a GIP announce packet (with a 300 ms guard) and retries up to 5 times while the pad is completely silent.
- Guide button (GIP command `0x07`) is handled; 1-byte dummy packets are ignored quietly.

**Performance**
- Pipelined asynchronous reads (4 transfers per endpoint) with proper stall recovery, driven by a `poll()` loop.
- Delta-only `uinput` writes: one `write()` per report, and none when nothing changed.
- No work in the hot path other than parsing; stats and hex dumps are optional.
- Optional real-time scheduling (`-R`).

**Dynamic Rumble / Force Feedback**
- Honours effect duration, delay, gain and overlapping effects; stops motors on exit.
- Updates are limited to the pad's 4 ms output interval and sent only when the level changes.
- **Quadratic ($x^2$) Rumble Scaling:** Instead of a raw linear curve that spins the motors at 100% like an angry lawnmower, light rumbles are scaled gently (`55%` strong cap / `40%` weak cap) so subtle in-game feedback stays subtle.
- **Smart Trigger Impulse Gate (`-t`):** Mirrors game rumble onto the two trigger motors, gated behind a ~30% heavy-hit threshold (`strong > 20000`) and capped at `20%` so your fingers don't go numb while holding RT to web-swing.

**Robustness & Auto-Boot**
- `-w` waits for the controller and survives unplugging; buttons are released on disconnect.
- `-r` resets the pad and re-opens it if it re-enumerates.
- Includes `install.sh` to set up `udev` permissions and a `systemd` background service (`vader5s.service`) so you never have to touch Konsole again.

**Mapping fixes**
- Xbox X/Y use `BTN_X`/`BTN_Y` (they were swapped), the D-pad is a hat, and ACKs are only sent for GIP packets.
- Share button verified at byte `18`, mask `0x01` (`-m 18:0x01:share`).

---

## How to Install & Run on Bazzite (Step-by-Step)

If you want to use this driver on your own Bazzite machine, follow these steps. You only need to do this once!

### 1. Clone the repository
Open **Konsole** on Bazzite and download this repo:

```bash
git clone [https://github.com/Atomice28/flydigi-vader5s-bazzite.git](https://github.com/Atomice28/flydigi-vader5s-bazzite.git)
cd flydigi-vader5s-bazzite
```

### 2. Compile the C driver (inside Distrobox)
Bazzite is an immutable OS, so compile `xbox_driver.c` inside a Distrobox container that has `gcc` and `libusb` (`libusb-1.0-0-dev` on Ubuntu/Debian or `libusb1-devel` on Fedora), then exit back to Bazzite:

You can execute this in one fell swoop without ever actually stepping fully inside the container (which saves you from typing `exit` after `distrobox enter`).

```bash
distrobox enter <your-container-name> -- gcc -O2 -Wall -Wextra -o xbox_driver xbox_driver.c -lusb-1.0
```
*(DON'T COPY YOUR-CONTAINER-NAME. REPLACE IT WITH YOUR OWN CONTAINER'S NAME, LIKE UBUNTU-DRIVER etc. DON'T FORGET. If you want to, you can still type distrobox enter, run the gcc command manually, and then type exit. If your terminal window completely vanishes upon exiting, don't panic—just reopen Konsole and run cd ~/flydigi-vader5s-bazzite again.)*

### 3. Install as an automatic background service (Recommended)
Run the included installer script on the Bazzite host:

```bash
sudo bash install.sh
```

What `install.sh` does for you:
1. Copies your compiled `xbox_driver` binary to `/usr/local/bin/xbox_driver`.
2. Creates `/etc/udev/rules.d/99-vader5s.rules` so Steam and games have permission to use the virtual pad.
3. Creates and enables `vader5s.service` (`systemd`), running `/usr/local/bin/xbox_driver -t -p -w -R` silently in the background on every boot.

To check that the background service is happy and running:
```bash
systemctl status vader5s.service
```

---

## Manual Run & Command-Line Flags (For Debugging)

If you aren't using the background service (or stopped it with `sudo systemctl stop vader5s.service`) and want to run the binary manually, quit Steam completely first, then run:

```bash
sudo modprobe uinput
sudo ./xbox_driver -t -p -w -R
```

| Flag | Meaning |
|------|---------|
| `-v` | hex-dump every packet in both directions (chatty) |
| `-n` | listen only: send nothing to the pad |
| `-e` | present as Xbox Elite Series 2 (`045e:0b00`) instead of Xbox 360 (`045e:028e`) |
| `-p` | `chmod 0666` the virtual pad's `/dev/input` nodes (a udev rule is cleaner) |
| `-t` | mirror game rumble onto the trigger motors (with heavy-hit threshold) |
| `-r` | USB-reset the controller before claiming it |
| `-R` | real-time scheduling + `mlockall` |
| `-w` | wait for the controller to appear / reappear |
| `-L` | learn mode: print which byte/bit changes when you press extra buttons |
| `-m O:M:N` | map bit mask `M` of byte `O` of the GIP input report to button `N` (`share`, `paddle1..4`, `extra1..8`, or a key code) |
| `-d V:P` | USB id to open (default `37d7:2801`) |

---

## Reading the output

- `[stats]` shows per-endpoint packets, measured reports per second and the worst gap between reports. The rate is only meaningful while you move a stick.
- Out of the box on default firmware, moving the sticks continuously will read around **~246–250 Hz**. The USB endpoint descriptor supports up to **500 Hz** (2 ms interval on full-speed USB) once switched to 500 Hz in the Flydigi Space Station app on Windows.
- `[hint] No packets yet` means the handshake did not complete. Try `-r`, try `-n`, press the Guide button, and make sure Steam is fully closed.

---

## Known limitations

- **The `Z` button is MIA:** `C` works, paddles work, Share works, but `Z` currently refuses to report for duty.
- **The orange RGB light strip is NOT driven:** It is controlled by Flydigi's undocumented vendor protocol and would need a USB packet capture from Flydigi Space Station for Xbox on Windows.
- **Trigger rumble channel:** Linux's `uinput` rumble API has no dedicated left/right trigger channel, so `-t` derives trigger impulse rumble from the main motor signal.

---

## References

- paroj/xpad issue [#363](https://github.com/paroj/xpad/issues/363) and PR [#368](https://github.com/paroj/xpad/pull/368): Vader 5S handshake behaviour and USB descriptors
- OpenJoystickDriver PR [#40](https://github.com/xsyetopz/OpenJoystickDriver/pull/40): the startup packet sequence for `37d7:2801`
