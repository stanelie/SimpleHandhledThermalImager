# Comparison against the RP2040 reference camera

A second camera built by André Weinand (<https://github.com/weinand/thermal-imaging-camera>)
uses the **same MLX90640** with an RP2040 and a 128×128 SSD1351 OLED. Its image
was visibly better than ours — sharper edges, more uniform interiors — and
closing that gap drove a long investigation. This file records what was found,
including the many hypotheses that were wrong, so nobody repeats them.

## Its configuration, for reference

| | RP2040 camera | this firmware (after the work below) |
|---|---|---|
| ADC | 19-bit | 19-bit |
| refresh | 32 Hz subpage | 32 Hz subpage |
| frame rate | 16 fps | 16.2 fps |
| display | SSD1351 OLED 128×128, image 128×96 | ST7789 IPS LCD 320×240 |
| magnification | 4× | 10× |
| calibration | full `CalculateTo` on all 768 px | gain + offset + **alpha** on 768; full Melexis on 3 |
| temporal filter | **none** | adaptive, sign-persistence |
| upscale | integer bilinear | bilinear, smoothstep weights |

Note its `display_smoothed_frame()` *looks* like a 5-frame box average but is
**dead code** — it never writes back into `dto->values`, and its only draw call
is commented out. The camera does no temporal filtering at all.

## What actually mattered

**1. ADC resolution.** We were at 18-bit; it uses 19-bit. Our own table in
[HARDWARE.md](HARDWARE.md) measures fixed-pattern noise at **86.6 counts at
18-bit against 25.2 at 19-bit** — 3.4× worse — for only 3.26 vs 6.68 counts of
temporal noise. FPN is fixed, so no filter can remove it.

**2. Per-pixel sensitivity (`alpha`) was never applied to the image.** Its
`CalculateTo` divides by `alpha[i]` for every pixel; we only did gain + offset.
Measured on this sensor's EEPROM, alpha has **sigma 9.93%, peak-to-peak 42.8%**.
It is *multiplicative*, so the error scales with signal.

Correlation of the uncorrected image against alpha, on real dumps:

| scene | r(image, alpha) |
|---|---|
| flat surface (colder than reflected reference) | **−0.690** |
| hand (warmer than reflected reference) | **+0.549** |
| after normalisation | +0.127 |

The sign reverses between a cold and a warm target — the signature of a
multiplicative error, and the reason an early test of alpha on a near-ambient
scene correctly found "no effect" and sent the investigation the wrong way.
Fixing it needs no `CalculateTo`: precompute `1/alpha[i]` normalised to the
array mean (`build_alpha_rcp()`), then one multiply and shift per pixel.

**3. Refresh rate.** 64 → 32 Hz halves temporal noise (6.68 → 4.01 counts, the
√2 from doubled integration) and leaves FPN untouched. Costs ~3 fps, landing at
16.2 — the same rate the reference camera runs at.

It also **structurally fixed the corrupted aux words**: at 64 Hz the sensor
rewrote its RAM every 15.6 ms while a full read took 23.8 ms, so every read
straddled an update. At 32 Hz the read fits inside one 31.2 ms period.
Measured over 442 frames: `aux rejects 0, i2c fails 0, bus recoveries 0`,
against a handful per thousand frames before.

**4. Palette and range.** Its ramp is 7 anchors (black→blue→cyan→green→yellow→
red→white) interpolated linearly with **no gamma**. Its range is
`step = (ceil(max+1) − floor(min−1))/255`, `index = (value − min)/step` — note it
subtracts the **unpadded** min but divides by the **padded** span, so the pad is
spent entirely at the top: the coldest pixel is index 0 (pure black) and the
hottest never reaches white. Padding symmetrically instead lifts the coldest
pixel to index ~33, a strong blue, which is visibly wrong at the dark end.

This is reproduced as **palette 4**, the default. Its ramp is bit-exact against
the original generator (verified, 0/256 mismatches). Its `SSD1351_color()` is the
same truncating RGB565 as ours, so colour quantisation is not a factor.

## Deviations we kept

- **Black anchored at the 5th percentile, not the absolute minimum.** Its black
  background is really its *noise*: an unfiltered, noisier image repeatedly
  touches the frame floor, which is index 0 by definition. Ours is clean enough
  that nothing reached bottom and the background floated at index ~75 (dark
  blue) with 2 pixels black. Anchoring 38 pixels up gives the same look from a
  cleaner signal — and 106 black pixels instead of 2.
- **Hot end trimmed to the 4th-hottest, not the absolute max.** Matching its
  untrimmed max reintroduced the old "blue flash": one spiked pixel blows the
  span up and collapses every real pixel to index ~0, so the thermal image goes
  black for a frame while the separately-drawn info bar survives.
- **Counts per degC is measured live** (`cpd100`, `dbg[20]`). It is *not* a
  constant: ~8.6 at 18-bit, ~5.7–6.9 at 19-bit, and it drifts with gain and Ta.
  A hard-coded value mis-sizes the pad, which is what made the hand render
  green instead of orange.

## Hypotheses that were measured and rejected

Each of these was plausible, and each cost a cycle of work:

| hypothesis | measurement | verdict |
|---|---|---|
| temporal filter blurs it | tested with the filter off — no change | wrong |
| gamma 4.0 crushes contrast | set to 1.5 — no change | wrong |
| TGC / compensation pixel | `TGC = 0.0000` on this sensor | no-op |
| Kta/Kv per-pixel | sigma 0.49, high-pass **0.28** counts vs a 4.07 noise floor | negligible |
| °C vs raw counts (T⁴) | max **3.8** palette steps of 255 over a 22–34 °C scene | negligible |
| broken/outlier pixels | **0 broken, 0 outlier** on this sensor | its correction is a no-op here |
| red dots are fixed-pattern | user observed they *move* | temporal spikes, not FPN |
| 10× vs 4× magnification | 10/320 vs 4.13/128 — same *fractional* blur | not a code difference |
| alpha causes the mottle | alpha high-pass barely changes: 3.54 → 3.49 | it is smooth shading, not mottle |

The last row is worth dwelling on: alpha is a **real and large** error, but it
varies smoothly across the array, so it produces shading rather than
pixel-to-pixel texture. Overall sigma dropped 5.05 → 3.79 when corrected while
the high-pass component was untouched.

## What is genuinely left

Ranked, with the measurements that justify them:

1. **No spatial processing exists at all.** Every pixel is filtered in isolation
   in time. Scenes are spatially correlated; noise is not.
2. **DDE** (base/detail decomposition, compress base, boost detail) — the
   standard technique in commercial thermal imagers, and the one that gives
   uniform interiors and crisp edges simultaneously.
3. **Motion-compensated temporal averaging.** Handheld tremor defeats the
   current filter. A global ±3 px integer search is 49 × 768 SAD ≈ 37k ops.
4. **Local contrast** (tiled histogram equalisation) — the auto-range is global,
   so one hot object flattens everything else.
5. **Edge-directed interpolation** at the 10× upscale.
6. **Super-resolution from tremor** — needs the motion estimation from (3).

There is budget: the I2C phase measures 37.4 ms but the bus transfer is only
~20 ms, so roughly **1.2M cycles per frame are spent spinning on data-ready** —
about 1500 cycles per sensor pixel.

The remaining differences that firmware cannot fix are the **OLED vs IPS LCD**
black level (an IPS panel emits ~0.125% of white at "black") and the fact that
our image is the same blur shown physically larger.
