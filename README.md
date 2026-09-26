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
| palette | fixed | 3 palettes + 4 gamma curves, switchable live |
| diagnostics | — | view modes that disable interpolation and/or filtering, to separate sensor behaviour from processing |
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
| wheel push | cycle gamma: 4.0 → 3.0 → 2.0 → 1.5 (default 4.0) |
| second button | cycle view mode: interpolation+filter (white crosshair) → neither, raw 10×10 blocks (yellow) → interpolation only (magenta) |

## Noise, and what actually helps

The sensor's residual noise is **temporal**, not fixed. Measured: ~6.4 counts
per frame against a displayed span that the auto-range had been shrinking to
~31 counts, so noise occupied up to 20% of the colour range. Two things mattered:

- **A minimum displayed span** (`SPAN_MIN`). Stretching a 31-count span across
  256 palette entries magnifies noise enormously; refusing to stretch below
  ~6 degC costs contrast only on scenes that have no detail to show anyway.
- **Sign-persistence in the temporal filter.** Noise is zero-mean and flips sign
  frame to frame; a real change does not. Tracking an EMA of the *signed* delta
  separates a small persistent change from noise of the same magnitude, which a
  per-frame magnitude threshold cannot. That allows 16-frame averaging on quiet
  pixels without the ~1 s settling it would otherwise cost.

Also fixed along the way: the magnitude thresholds were set at 2x/4x the mean
absolute deviation, but MAD ~ 0.8 sigma, so noise alone crossed the lower one
~11% of the time -- one pixel in nine escaped filtering every frame, which was
itself the visible sparkle.

A flat-field (NUC) correction was implemented and then **removed**: a
two-pass reproducibility test showed the table collapsing 64% on the second
capture, proving it was recording scene content rather than sensor structure.
Without a mechanical shutter there is no uniform reference, and the residual
fixed pattern turned out to be small anyway. It is in git history if wanted.

## Processing pipeline

```
read raw frame  ->  gain + per-pixel offset + flat-field  ->  adaptive temporal filter  ->  bilinear 10x upscale + gamma + palette
```

Every stage is there because a measurement said so, and three earlier stages were
removed once measured: a subpage equaliser (the difference was ~0.4 counts), the
Kta/Kv offset terms (a uniform 1.6-count shift), and a spatial blur (it only
softened random noise the temporal filter already handled). See the
characterisation table in [`docs/HARDWARE.md`](docs/HARDWARE.md).

Filtering happens on the 32x24 sensor data, before upscaling -- 768 pixels
instead of 76800, a 100x difference in cost.

## Build and flash

Requires `gcc-arm-none-eabi`, `binutils-arm-none-eabi` and `openocd`, plus an
ST-Link V2 (a clone is fine).

```sh
cd firmware
make          # build
make flash    # flash over SWD
make debug    # dump the dbg[] telemetry array from a running target
make diag     # flash the sensor-characterisation build (no display pipeline)
```

`tools/sweep_diag.sh` and `tools/analyse_diag.py` drive the `diag` build across
sensor configurations and report per-pixel temporal noise and fixed-pattern
noise. Note it force-rebuilds: `make` will not rebuild on a `-D` flag change
alone, which silently invalidated an entire measurement sweep once.

Wire the ST-Link to the 4-pad debug header: **GND→GND, SWDIO→D1 pad,
SWCLK→D0 pad**. Leave the ST-Link's 3.3 V disconnected — the camera powers
itself, and you do not want two supplies fighting. If OpenOCD reports
`init mode failed`, drop the adapter speed (`adapter speed 100`) and check that
no stale `openocd` process is holding the port.

## Restoring the stock firmware

The dump lives in [`stock-firmware/`](stock-firmware/) with its sha256 — but it is
**gitignored and therefore not on GitHub**, so it exists only on the local
machine. See that folder's README.

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
- **The flat-field table lives in RAM and is lost on every power cycle**, so the
  grid returns until the second button is pressed again. Since FPN measured
  26-40x larger than random noise, persisting this table to the unused SPI flash
  is probably the highest-value remaining improvement.
- The SPI NOR flash (XT25F128F) is present but unused.
- A rare full-device hang was seen once and not yet reproduced; when caught, the
  core showed **no fault** (CFSR/HFSR clear) and the loop was still advancing, so
  it is not a CPU exception. `dbg[8]`/`dbg[9]` count I2C failures and bus
  recoveries, which are the leading suspects.
