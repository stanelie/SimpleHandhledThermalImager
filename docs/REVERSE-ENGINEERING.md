# How this was reverse-engineered

Notes for whoever picks this up next, including the approaches that wasted time.
The hardware facts themselves are in [HARDWARE.md](HARDWARE.md).

## Getting in

The GD32F103's flash is not read-protected, so SWD gives full access with a
cheap ST-Link V2 clone:

```sh
openocd -f interface/stlink.cfg -c "adapter speed 100" -f target/stm32f1x.cfg \
  -c "init" -c "reset halt" -c "dump_image backup.bin 0x08000000 0x10000" -c "shutdown"
```

The board's 4-pad header is GND, 3.3 V, SWDIO, SWCLK. Distinguishing SWD from
UART by voltmeter does not work — a multimeter averages transitions. A logic
analyser at ≥1 MHz during power-on settles it: a UART shows a framed burst, SWD
lines just sit at their idle levels.

## Emulate the firmware; do not read the disassembly by eye

**This is the single most important lesson here.** Hand-transcribing Thumb
disassembly produced three wrong conclusions in a row, each costing a
build/flash/observe cycle on hardware:

1. Misread which register a store targeted — `adds r1,r1,#4` came *after* the
   store, not before — so the decoded protocol drove RS/DC low mid-pixel, turning
   every pixel's low byte into a *command* byte.
2. Dropped a trailing instruction from a byte-send sequence.
3. Assumed a function was "draw one column" when it was really
   `set_window(x0,y0,x1,y1)`; a synthetic 3-argument call left `r3` uninitialised
   and produced a bogus fourth coordinate.

`GPIOx_BSRR` and `GPIOx_BRR` are especially easy to confuse: both mean "apply
this mask", their literal-pool entries sit adjacent, and they differ by one bit
in the offset.

The fix is to **execute the real bytes**. `tools/emu.py` and
`tools/decode_init.py` use [Unicorn](https://www.unicorn-engine.org/):

```sh
python3 -m venv venv && venv/bin/pip install unicorn
venv/bin/python tools/decode_init.py     # needs your own backup.bin
```

The technique:

- Map flash at `0x08000000`, RAM at `0x20000000`, and a peripheral region.
- Hook `UC_HOOK_MEM_WRITE` over the peripheral range.
- Call a function directly: set `r0`–`r3`, put a sentinel in `LR`, stop there.
- Stub out long delay loops by writing `bx lr` (`\x70\x47`) over them.
- Where a wait-loop polls a status bit, hook `UC_HOOK_MEM_READ` and force the
  ready bits set, or it spins forever.

Crucially, **decode the logical protocol inside the hook** rather than dumping
raw register writes: track the RS/DC pin and latch the bus value on each WR
edge. That turns an unreadable wall of `GPIOB_ODR <= 0x2a` into:

```
CMD 0x2A
    data: 0x00 0x00 0x00 0xEF
```

which is directly replayable. That is how the whole panel init sequence was
recovered in one shot after days of guessing.

Also emulate **your own** compiled binary the same way and compare it against the
original byte for byte before flashing. Regenerate the `.bin` first — a stale
`objcopy` output once had me "verifying" firmware from several revisions earlier
and drawing confident conclusions from it.

## Useful entry points in the stock firmware

Addresses are for the dump this work was based on; a different build will differ.

| address | what |
|---|---|
| `0x8003980` | send one command byte to the panel |
| `0x8003d4c` | `set_window(x0,y0,x1,y1)` — CASET/PASET/RAMWR |
| `0x8003a28` | fill a rectangle; contains the per-pixel write loop |
| `0x8005aac` | GPIO and peripheral clock init |
| `0x8006a5c` | **full ST7789 panel init sequence** |
| `0x80069cc` | `gpio_pin_remap_config()` — writes AFIO_MAPR (the JTAG release) |
| `0x80047ec` | `SystemInit` — clock/PLL configuration |
| `0x8001340` | ADC + DMA setup for the button ladder |

The stock firmware is built on GigaDevice's **Standard Peripheral Library**;
recognising `gpio_init(port, mode, speed, pin)` with `mode=0x10` (`OUT_PP`) and
`speed=0x03` (50 MHz) makes large stretches readable by inspection.

## Finding the inputs

Scanning spare GPIOs as pull-up inputs and latching anything that goes low finds
simple switches. It found exactly one pin (PA1) and missed everything else,
because:

- The **buttons are an analog ladder**, so most of them never cross the digital
  threshold. The giveaway was the stock firmware configuring ADC channels 0 and 1
  with DMA. Sample the ADC and bucket distinct levels instead.
- The **wheel contacts connect pins to each other**, not to ground, so with
  pull-ups on every pin nothing ever changes. And they conduct transitively, so
  driving one pin cannot separate the positions — drive each in turn.
- Driving the display bus rewrites `GPIOB_ODR`, which flips the pull direction of
  any GPIOB pin being used as an input. Mask the upper bits into every bus write
  or the scan is garbage.

When a control's function is ambiguous, put the decoded state **on the screen**
as a digit rather than reading memory over SWD once per press. One flash then
answers every question at human speed.

## Debugging technique

The firmware keeps a `volatile uint32_t dbg[]` array that OpenOCD reads out of a
running target (`make debug`). This avoids needing a console before the display
works, and gives exact cycle counts via `DWT_CYCCNT` for profiling.

One trap worth repeating: ending an OpenOCD invocation with `halt` and then
`shutdown` leaves the **core stopped**. That looks exactly like a firmware
freeze, and it sent me chasing a nonexistent I2C lockup bug for several rounds.
Always `resume` before `shutdown`.
