# Hardware map

Everything here was derived from a SWD dump of the stock firmware plus continuity
tracing on the board. None of it is vendor-documented. Package pin numbers refer
to the **LQFP48** package, whose pinout matches the STM32F103C8T6.

## MCU

**GD32F103**, LQFP48, Cortex-M3, 64 KiB flash, 20 KiB SRAM. Flash is not
read-protected, so SWD can both dump and program it.

Clock: **12 MHz HSE crystal → /2 → PLL ×12 = 72 MHz**, APB1 /2, APB2 /1.
This is exactly what the stock firmware does (`RCU_CFG0 = 0x082b0402`). The
crystal frequency is not marked anywhere; it was deduced from the USB prescaler
bit — `USBPRE=0` means PLL/1.5 must produce 48 MHz, so the PLL is 72 MHz and the
crystal is 12 MHz.

> Firmware that does not configure the PLL boots at 8 MHz on the internal RC and
> is **9× slower**. This is easy to miss.

## THE trap: PB3/PB4 are JTAG pins

After reset, **PA13, PA14, PA15, PB3 and PB4 belong to the JTAG peripheral**, not
to GPIO. PB3 (JTDO) and PB4 (NJTRST) are two of the display's data lines, and
PA15 is the sensor's SDA. Writing `GPIOB_CRL` has **no effect** on PB3/PB4 until
JTAG is released, so every byte on the display bus is silently corrupted and the
panel shows nothing at all.

```c
RCC_APB2ENR |= (1u << 0);                             /* AFIO clock */
AFIO_MAPR = (AFIO_MAPR & ~(7u << 24)) | (2u << 24);   /* SWJ_CFG = 010 */
```

`SWJ_CFG = 010` disables JTAG-DP but keeps SW-DP, so SWD still works and you can
still flash. Do **not** use `100` (disables both) — that costs you debug access.

## Display — ST7789, 240×320, 8-bit parallel (8080), bit-banged

| signal | pin |
|---|---|
| D0–D7 | PB0–PB7 |
| RS / DC | PA2 (0 = command, 1 = data) |
| CS | PA3 (pulses low around each byte) |
| WR | PC15 (low→high latches the byte) |
| RD | PC14 (parked high) |
| backlight / panel reset | PA8 |

Byte protocol: assert CS low, put the byte on PB0–7, pulse WR low→high, release
CS. Commands are identical but with RS low first.

CS and RS **do not** need toggling per byte during a bulk pixel blast — hoisting
them out of the loop roughly halves render time and the panel accepts it.

The panel is an ST7789: the init stream uses the ST7789-specific register set
(`0xB2`, `0xB7`, `0xBB`, `0xC0`–`0xD0`). The **stock init sequence is replayed
verbatim** by this firmware — see `init_seq[]` in `firmware/main.c`, extracted by
emulating the stock routine at `0x8006a5c`.

> Do not send `SWRESET` (`0x01`) without replaying the whole init. A soft reset
> returns the VCOM / VRH / gamma registers to defaults and the panel then shows
> **nothing**, which looks exactly like a wiring fault. This cost hours.

Two more panel notes:
- `MADCTL = 0xE8` gives the correct image orientation (landscape 320×240).
- The panel mirrors X relative to logical coordinates. The image happens to be
  fed in an order that cancels this, but **overlay coordinates must be mirrored**
  (`mx()` in `gfx.h`) and glyphs must be drawn with columns flipped (`g_fh=1`),
  or text appears reversed and right-to-left.
- `SWRESET` does not clear GRAM. After a reset the panel still displays whatever
  the previous firmware drew, which is misleading when debugging.

## Sensor — MLX90640, software I2C

| signal | pin |
|---|---|
| SCL | PA9 |
| SDA | PA15 |

These are **not** hardware-I2C-capable pins — I2C0 is PB6/PB7, which are display
data lines here. So the bus must be bit-banged; there is no way to use the
hardware peripheral without rewiring. Pull-ups are on the main PCB.

Current implementation runs at ~731 kHz (`I2C_HALF 3`). The MLX90640 tolerates up
to 1 MHz; pushing past that makes it NACK everything.

**The refresh rate register (`0x800D`, bits 9:7) is the real frame-rate limit.**
The stock firmware leaves it at `011` = 4 Hz. This firmware sets `111` = 64 Hz.
Note the register sets the *subpage* rate; a complete chess-pattern image needs
two subpages.

Measured: 23.8 ms to read 832 words (1668 bytes) at 731 kHz.

### The aux words get caught mid-update

The housekeeping words share the sensor's RAM with the pixels:

| index | address | meaning |
|---|---|---|
| 768 | `0x0700` | ptatArt |
| 778 | `0x070A` | gain |
| 800 | `0x0720` | ptat |
| 810 | `0x072A` | vdd |

At a 64 Hz subpage rate the sensor rewrites its RAM every 15.6 ms, but a full
read takes ~23.8 ms, so **every read straddles an update**. Reading all 832 words
in one burst puts these words at the *end* of the transfer, ~23 ms in, which is
exactly where the update lands — and they intermittently come back garbage, one
word at a time. Observed values against their normal readings:

| word | normal | seen when corrupt |
|---|---|---|
| `frame[778]` gain | ~6168 | 32767 |
| `frame[800]` ptat | ~1652 | −32 |
| `frame[768]` ptatArt | ~−21 | 19771 |

Gain and Ta feed **every** pixel, so a single bad word flashes the whole image
and throws the min/max readouts to absurd values (~80 °C, or −273 °C when
`mlx_to()` hits its `ac<=0` bail-out). The pixels themselves are fine — they are
read early and escape the update — which makes this look like an image bug when
it is not.

Mitigations used here, in `cal.h`/`main.c`:

1. Read the aux block (`0x0700`, 64 words) **first**, immediately after
   data-ready, when the sensor has just finished writing.
2. Validate the **computed** VDD / Ta / gain against physical ranges
   (3.0–3.6 V, −20–85 °C, 0.7–1.4) and fall back to the previous frame's values
   when impossible. Ta and VDD drift over seconds, so the previous values are the
   physically correct answer, not a fudge. `dbg[14]` counts how often this fires
   (a handful per thousand frames).
3. Do **not** re-read the gain separately in the pixel-correction path. Doing so
   is another chance to catch a mid-update word, and a `gain==0` fallback there
   multiplies every pixel by ~6200 and saturates the frame.

Dropping the subpage rate to 32 Hz would make reads fit inside one update period
and avoid this structurally, at the cost of roughly 3 fps.

### What the sensor actually gives you (measured)

Characterised with the `diag` build (see the Makefile), which reads raw frames
with the whole display pipeline out of the way and accumulates per-pixel
statistics on-device. Camera stationary, pointed at a uniform surface, ~1200
frames per configuration. Counts are raw sensor LSBs.

| config | temporal noise (median) | fixed pattern (high-pass) | FPN ÷ temporal |
|---|---|---|---|
| 18-bit / 64 Hz | 3.26 | **86.6** | 27x |
| 18-bit / 32 Hz | 2.03 | 85.7 | 42x |
| 19-bit / 64 Hz | 6.68 | **25.2** | 3.8x |
| 19-bit / 32 Hz | 4.01 | 24.9 | 6.2x |

Conclusions that drove the current design:

- **Fixed-pattern noise dominates random noise by 4x to 40x.** Any visible
  "grid" or "dots" is FPN, not noise, and no amount of filtering fixes it. This
  is the single most useful fact about this sensor.
- **The FPN is stable**: the same pixels are worst in every configuration --
  (19,25), (11,25), (19,27), (7,18). That is what makes a flat-field correction
  work, and it is why the firmware has one.
- **The per-pixel EEPROM offset removes most of it**: raw FPN ~21 counts
  high-passed, ~3.3 counts after `poff[]`. The flat-field pass removes the rest.
- **32 Hz halves temporal noise** (3.26 -> 2.03 and 6.68 -> 4.01, both ~1.6x,
  matching the sqrt(2) expected from doubled integration time). FPN is unaffected
  by refresh rate. 64 Hz is used anyway, because the extra latency at 32 Hz is
  more objectionable in the hand than the noise it removes.
- Absolute counts are **not comparable across ADC resolutions** -- the LSB scale
  differs and a uniform target gives no calibrated signal to normalise against.
  Do not read the 18-bit vs 19-bit rows as a like-for-like comparison.

## Controls

The switch common returns to **pin 47 (VSS)**, i.e. switches are active-low.

**Buttons — analog resistor ladder on PA1** (ADC channel 1). Several switches
share one pin, distinguished by voltage:

| level (12-bit) | meaning |
|---|---|
| 4095 | idle |
| 2045 | middle button -> toggles the OSD |
| 0 | second button -> cycles the view mode |

> The ADC's first conversions after calibration read **0**, which decodes
> identically to the 0-level button being held. Enough of them in a row satisfied
> the debounce and fired a button action at every boot. Ignore the button until
> the ADC is known good (`iters < 15`), or a mode silently toggles on startup --
> this went unnoticed for a while because the action it triggered was a
> calibration whose effect was hard to see.

PA0 is also analog but sits constant at ~3201 (~2.58 V) and never moves — it is
almost certainly a **battery monitor**, not an input.

The stock firmware reads both via ADC with DMA; that is how the ladder was found
(see the ADC setup around `0x8001340`).

**Wheel — 3-position rocker on PB8, PB9, PB15.** It is *not* a rotary encoder,
despite having three pins. Each position shorts one pin to the other two, and
because the contacts conduct transitively you **cannot** tell the positions apart
by driving a single pin. You must drive each of the three in turn and record
which others follow. `scan_wheel()` in `firmware/main.c` does this as a ~60 µs
burst between frames, so the pixel loop keeps a constant upper-bit pattern.

Pair bitmask: `1` = PB8–PB9, `2` = PB8–PB15, `4` = PB9–PB15.

| action | settled code |
|---|---|
| left | 3 |
| right | 6 |
| push | 5 |

These were measured on one unit and may vary. The transitional codes seen while
the wiper slides are meaningless — only the settled value counts, which is why
the decoder requires the code to be stable before acting.

> Gotcha: writing the display data bus writes `GPIOB_ODR`, which also sets the
> **pull direction** of PB8–PB15 when they are inputs. Write `0xFC00 | byte` so
> the upper bits keep a defined pull. A scan that ignores this reports every
> upper pin low and looks like all the buttons are stuck.

## Storage — XT25F128F (unused)

16 MB SPI NOR on hardware SPI0, CS on **PA4**. It backed the stock firmware's
USB mass-storage BMP snapshot feature (320×240 16-bit BMPs — which is how the
panel resolution was first confirmed). This firmware does not use it.
