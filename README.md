# Simple Handheld Thermal Imager — replacement firmware

Open replacement firmware for a cheap handheld thermal camera built around a
**GD32F103** MCU and an **MLX90640** 32×24 thermal sensor.

The stock firmware ran the display at **4 fps**. This firmware runs the same
hardware at **19 fps**, adds bilinear interpolation, per-pixel calibration,
temporal denoising, real temperature readouts in °C, and an on-screen display,
and drops the USB mass-storage / BMP snapshot feature that the stock firmware had.

Nothing about this hardware is documented by the vendor. Everything in
[`docs/HARDWARE.md`](docs/HARDWARE.md) was reverse-engineered from a SWD dump of
the stock firmware; [`docs/REVERSE-ENGINEERING.md`](docs/REVERSE-ENGINEERING.md)
describes how, and which approaches wasted time.

## Results

| | stock | this firmware |
|---|---|---|
| frame rate | 4 fps | **19 fps** |
| image | 32×24 nearest-neighbour, visible fixed-pattern noise | bilinear 10× upscale to 320×240, offset-calibrated, temporally denoised |
| temperature | none displayed | min / centre / max in °C |
| palette | fixed | 3 palettes + 3 gamma curves, switchable live |
| USB / snapshots | yes | removed |

The single biggest win was not code at all: **the stock firmware left the
MLX90640's refresh-rate register at 4 Hz.** Setting it to 64 Hz and fixing the
system clock (the stock image runs the PLL at 72 MHz; naive firmware boots at
8 MHz on the internal RC) accounts for most of the improvement. The remaining
gains came from the render loop.

Measured budget per frame at 19 fps: **I2C read 23.8 ms, render 28.6 ms.**
The bus still has headroom — it runs at 731 kHz against the MLX90640's 1 MHz
ceiling, worth roughly another 2 fps.

## Controls

| control | action |
|---|---|
| middle button | toggle the OSD (when hidden, the image expands to the full 240 rows) |
| wheel left / right | cycle palette: rainbow → ironbow → grayscale |
| wheel push | cycle gamma: 1.5 → 2.0 → 3.0 |

## Build and flash

Requires `gcc-arm-none-eabi`, `binutils-arm-none-eabi` and `openocd`, plus an
ST-Link V2 (a clone is fine).

```sh
cd firmware
make          # build
make flash    # flash over SWD
make debug    # dump the dbg[] telemetry array from a running target
```

Wire the ST-Link to the 4-pad debug header: **GND→GND, SWDIO→D1 pad,
SWCLK→D0 pad**. Leave the ST-Link's 3.3 V disconnected — the camera powers
itself, and you do not want two supplies fighting. If OpenOCD reports
`init mode failed`, drop the adapter speed (`adapter speed 100`) and check that
no stale `openocd` process is holding the port.

## Restoring the stock firmware

Take a dump **before** you flash anything:

```sh
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "init" -c "reset halt" \
  -c "dump_image backup.bin 0x08000000 0x10000" -c "shutdown"
```

and restore with `program backup.bin 0x08000000 verify`. The flash is 64 KiB and
is **not** read-protected.

The stock dump is deliberately **not** included in this repository — it is the
vendor's proprietary firmware and this repo is public. Keep your own copy
locally; `docs/REVERSE-ENGINEERING.md` explains how the tooling in `tools/`
uses it.

## Layout

```
firmware/     the firmware itself (main.c textually includes cal.h and gfx.h)
  main.c      clock, display driver, software I2C, render loop, controls
  cal.h       MLX90640 temperature calibration (Melexis reference math)
  gfx.h       font, status bar, crosshair
  startup.c   vector table and reset handler
  link.ld     64 KiB flash / 20 KiB RAM
tools/        Unicorn-based emulation harnesses used to read the stock firmware
docs/         hardware map and reverse-engineering notes
```

## Status and known gaps

- The **third switch is unidentified**. Two switches sit on the PA1 analog
  ladder and the wheel accounts for three more actions, but one switch was never
  located. The only unscanned pins left are the SPI-flash lines (PA4–PA7) and
  SWD (PA13/PA14), which can't be probed without losing the debug connection.
- Emissivity is fixed at 0.95. Skin is nearer 0.98, so readings of skin come out
  very slightly high — centre read 34.5 °C against a reference camera's 34 °C.
- The image is auto-ranged every frame between the 4th-coldest and 4th-hottest
  pixel, so contrast is relative, not absolute. There is no fixed-range mode yet.
- Only the centre, min and max pixels are converted to °C. The image itself is
  rendered from gain- and offset-corrected raw counts, which is monotonic in
  temperature but not calibrated per pixel. Converting all 768 pixels would cost
  most of the frame rate on a soft-float Cortex-M3.
- The SPI NOR flash (XT25F128F) is present but unused.
