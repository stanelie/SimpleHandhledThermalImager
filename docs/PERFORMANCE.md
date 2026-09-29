# Performance: what worked, what didn't, and how to measure it

Stock firmware ran at **4 fps**. This firmware runs the same hardware at
**20–21 fps**, or **17 fps** with full per-pixel radiometry — which is the mode
that reproduces the RP2040 reference image, and which the reference camera
itself runs at 16 fps.

Everything here was measured on the device. Several of the changes below were
proposed on a plausible estimate and turned out to be worthless or actively
harmful; those are recorded too, because the estimate is the part that was
wrong, not the measurement.

## Measure this correctly or you will chase the wrong stage

**`dbg[5]` is the RENDER timer. `dbg[6]` is the I2C read.** Reading them the
other way round makes the render look like an I2C bottleneck. That mistake sent
a whole round of optimisation at the wrong stage.

**The stage timers do not sum to the frame period.** `correct_frame`,
`calc_frame_params`, the three `mlx_to` calls behind the OSD labels, the ADC and
wheel scan and the label averaging all fall outside them — about **9 ms a
frame**. Computing `1000/(render + I2C + convert)` overstated the frame rate by
roughly 20% for a long time.

**Use `fps_disp`**, shown at the right of the status bar. It counts whole loop
iterations against `DWT_CYCCNT` over a one-second window, so it measures the
thing itself. Note its resolution: at 20 fps one frame is 50 ms, so a change
smaller than ~2.5 ms will not move it.

## Soft-float on a Cortex-M3 with no FPU

Running the full Melexis `CalculateTo` on all 768 pixels first measured
**188.7 ms — 4.0 fps**, which is, notably, exactly the rate the stock vendor
firmware ran at. It ended at **19.8 ms**, a **9.5× speedup**, with the output
verified identical throughout (mean 0.053 °C, max 0.108 °C against `mlx_to()`,
which is entirely the 0.1 °C storage quantisation).

In order of what they were worth:

1. **The three-stage chain collapses to a function of one variable.** Writing
   `u = ir/ac`, the `ac` factors cancel out of every stage:
   `Sx = k1·ac·Tk`, so `ir/(ac·base + Sx) = u/(base + k1·Tk)`, and
   `acc2 = ac·W(u)` so `ir/acc2 = u/W(u)`. `To` therefore depends only on `u`,
   and `u ↔ Tk` is a bijection — so two of the three 4th roots, both divides
   after the first, and the band selection all become a table indexed by `Tk`.
2. **Precompute per-pixel constants as scaled integers.** `alpha`, `kta` and
   `kv` are fixed in EEPROM; the loop was re-decoding bitfields and doing three
   int→float conversions *per pixel per frame*.
3. **`fsqrtf` was running 8 Newton iterations containing a soft-float DIVIDE
   each** — 96 divides per pixel. Newton on the *reciprocal* square root is
   multiplies only. 3 iterations gives 1.65e-7 relative error, i.e. float
   epsilon.
4. **A direct 4th root** beats two nested square roots: split the float into
   exponent and mantissa, interpolate `m^(1/4)` from a 64-entry table, fold
   `2^(e/4)` back in. ~6 operations instead of ~30, 5.68e-6 relative error =
   0.0017 K on a 300 K target.
5. **`p2(n)` computed 2ⁿ by multiplying in a loop**, and `px_alpha()` called it
   with `alphaScale_ = 38` for every pixel. It is an exponent field.

**Verify these against the original, not just for speed.** `tools/` has the
pattern: dump a raw frame and the EEPROM off the device, compile the *real*
`cal.h` on the host, and run both implementations over all 768 pixels.

### Rebuild throttling, and two traps in it

Cached tables must be throttled on a **tolerance**, not an exact float compare.
`if(f_taTr != h_built_taTr)` rebuilt a 256-entry table on **100% of frames** —
measured, 130 rebuilds in 130 frames — because Ta jitters every frame. The
cache never hit once.

And a rebuild that costs ~18 ms lands as a **visible stutter**. Spread it: latch
the constants at the start of a pass, then build 96 entries per frame. Convert
time went from alternating 12.29/30.14 ms to a steady 14.22–14.34.

> **Never put a `float[768]` on the stack.** `build_px_tables` did, to compute a
> mean — 3072 bytes against ~2600 free, overflowing into bss on every rebuild.
> The scaling reference does not have to *be* the mean; the EEPROM's nominal
> alpha works and needs no temporary at all.

## The render loop

Measured split: **9.73 ms of per-pixel computation, 14.11 ms of bus.** Get this
split before optimising — it is what rules DMA out (see HARDWARE.md).

The blend `a*(256−w) + b*w`, all `>>8`, is algebraically `a + ((b−a)*w >> 8)` —
**one multiply instead of two**. Fully unrolling the 10-step inner loop with the
weights as literals removed a table load and all loop overhead. That was 9.6 ms
a frame, in the hottest loop in the firmware at 64,640 iterations.

## Things that did not work

| attempt | result |
|---|---|
| **DMA for the display bus** | would be ~2× SLOWER — three register writes per byte, and a DMA transfer costs more than a CPU store |
| **108 MHz / 96 MHz** | this chip will not run above 72 MHz; PLL locks, core executes garbage |
| **12-bit colour (RGB444)** | 93 distinct palette colours against RGB565's 226; visibly destroys gradation for ~1.4 ms, which did not move the frame counter |
| **`I2C_HALF 1`** | sensor ACKs but returns corrupted data — 147 rejects in 147 frames |

## Flat-field / FPN correction: tried twice, rejected twice

Do not implement this a third time without reading this section.

The second attempt was button-triggered (no automatic firing), averaged over 32
frames (so the table's own temporal noise was ~0.7 counts rather than ~4.2), and
captured against surfaces the user chose. It was still rejected, on a two-pass
test — capture two different uniform surfaces, correlate the tables:

| | |
|---|---|
| r over all 768 pixels | **+0.514** |
| r on the **high-pass** part | **+0.436** |

At r ≈ 0.5 the table is about half real structure and half noise, so applying it
removes ~1.4 counts of genuine fixed error while writing in ~1.4 counts of new
fixed error. A wash.

**The decisive detail is that the high-pass correlation is LOWER than the
overall one.** Fixed-pattern noise is by definition pixel-to-pixel, so a genuine
FPN table would be *more* reproducible in its high-frequency part, not less. The
reproducible component here is smooth — i.e. lens vignetting, which is
multiplicative and would need a gain correction, not an offset one.

Confirmed visually: no stuck or static pixels on this sensor, only random noise.
`poff[]` already removes the fixed pattern; what remains is temporal.

> **A warning about the metric.** `noise_spatial()` measures the median
> |horizontal difference|, which counts **real scene detail** as well as noise —
> and scene detail is exactly as constant under temporal averaging as FPN is.
> Decomposing a B-sweep as "temporal vs fixed" therefore overstates the fixed
> part, and I used that decomposition to justify retrying the flat-field. Do not
> read a non-averaging-away residual as evidence of correctable sensor error.

## The hard floor

I2C is **18.1 ms** and cannot be improved: the sensor corrupts data above this
rate, silently. With render at ~23 ms that caps the simple pipeline near 24 fps
before anything else is counted.
