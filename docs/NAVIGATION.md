# Navigation Layer

Module:

```text
src/navigation/
```

## Overview

The navigation layer decodes **satellite navigation messages** from the
demodulated bitstream produced by the tracking layer. The output consists of
structured satellite orbit and clock data required for pseudorange computation.

## Input and Output Data

```text
Tracking (prompt I-component, 1 bit / 20 ms)
    │
    ▼
[Navigation Layer]
    │
    ├── Ephemeris  { a, e, i, Ω, ω, M₀, … }
    ├── ClockParams { af0, af1, af2, toc }
    ├── IonosphericModel { α[], β[] }
    └── Almanac    { orbital approx for all PRN }
    │
    ▼
Observables (pseudorange calculator)
```

## GPS L1 C/A Navigation Message Structure

### Frame Format

```text
1 frame = 5 subframes × 300 bits = 1500 bits = 30 seconds

Subframe 1 (300 бит):
  ┌─ TLM (30 bits) ─ HOW (30 bits) ─ words 3-10 (240 bits) ─┐
  │  preamble      TOW, subframe ID   clock parameters      │
  └─────────────────────────────────────────────────────────┘

Subframe 2: orbital params (part 1)
Subframe 3: orbital params (part 2)
Subframe 4: almanac + ionosphere (pages 1-25, cyclic)
Subframe 5: almanac (PRN 1-24)
```

### TLM and HOW

| Field           | Size    | Description                                     |
| --------------- | ------- | ----------------------------------------------- |
| TLM preamble    | 8 bits  | `0b10001011` = `0x8B` — subframe start marker   |
| TLM message     | 14 bits | reserved                                        |
| HOW TOW         | 17 bits | start time of next subframe (in 6-second units) |
| HOW subframe ID | 3 bits  | 1–5 — current subframe number                   |

### Parity Check

Each 30-bit word contains a 6-bit parity field (Hamming):

```text
bits 1-24: data
bits 25-30: parity
```

Decoding: verify each of the 6 parity equations. On failure, wait for the next
subframe.

## Components

### FrameDecoder (`frame_decoder.rs`)

**Responsibilities:**

1. Detect TLM preamble (`0x8B`)
2. Verify parity of each word
3. Extract TOW and subframe ID from HOW
4. Assemble complete subframe (10 × 30 bits)
5. Dispatch to Subframe 1/2/3 parsers

```rust
pub struct FrameDecoder {
    bit_buffer: VecDeque<u8>,
    state: DecodeState,
}

pub enum DecodeState {
    SearchingPreamble,
    VerifyingAlignment { offset: usize },
    Collecting { subframe_id: u8, bits_collected: usize },
}
```

**Preamble Detection Algorithm:**

```text
for offset in 0..30:
    if bit_buffer[offset..offset+8] == 0b10001011:
        if parity(word_at(offset)) == OK:
            locked = true
```

### EphemerisParser (`ephemeris.rs`)

Decodes Subframe 1, 2, 3 according to GPS ICD-200.

#### Subframe 1 — Clock Parameters

| Parameter     | Bits    | Scale     | Description                 |
| ------------- | ------- | --------- | --------------------------- |
| `week_number` | 10 bits | 1         | GPS week number             |
| `ura_index`   | 4 bits  | —         | User Range Accuracy         |
| `sv_health`   | 6 bits  | —         | 0 = healthy                 |
| `iodc`        | 10 bits | 1         | Issue of Data Clock         |
| `toc`         | 16 bits | 2⁴ s      | Clock reference time        |
| `af2`         | 8 bits  | 2⁻⁵⁵ s/s² | Quadratic clock coefficient |
| `af1`         | 16 bits | 2⁻⁴³ s/s  | Linear clock coefficient    |
| `af0`         | 22 bits | 2⁻³¹ s    | Constant clock coefficient  |

#### Subframe 2 — Orbit (Part 1)

| Parameter | Bits    | Scale      | Description                    |
| --------- | ------- | ---------- | ------------------------------ |
| `iode`    | 8 bits  | 1          | Issue of Data Ephemeris        |
| `crs`     | 16 bits | 2⁻⁵ m      | Radius sine correction         |
| `delta_n` | 16 bits | 2⁻⁴³ rad/s | Mean motion correction         |
| `m0`      | 32 bits | 2⁻³¹ π rad | Mean anomaly                   |
| `cuc`     | 16 bits | 2⁻²⁹ rad   | Latitude correction (cos)      |
| `e`       | 32 bits | 2⁻³³       | Eccentricity                   |
| `cus`     | 16 bits | 2⁻²⁹ rad   | Latitude correction (sin)      |
| `sqrt_a`  | 32 bits | 2⁻¹⁹ m½    | Square root of semi-major axis |
| `toe`     | 16 bits | 2⁴ s       | Ephemeris reference time       |

#### Subframe 3 — Orbit (Part 2)

| Parameter   | Bits    | Scale      | Description                  |
| ----------- | ------- | ---------- | ---------------------------- |
| `cic`       | 16 bits | 2⁻²⁹ rad   | Inclination correction (cos) |
| `omega0`    | 32 bits | 2⁻³¹ π rad | Longitude of ascending node  |
| `cis`       | 16 bits | 2⁻²⁹ rad   | Inclination correction (sin) |
| `i0`        | 32 bits | 2⁻³¹ π rad | Inclination                  |
| `crc`       | 16 bits | 2⁻⁵ m      | Radius correction (cos)      |
| `omega`     | 32 bits | 2⁻³¹ π rad | Argument of perigee          |
| `omega_dot` | 24 bits | 2⁻⁴³ rad/s | Rate of right ascension      |
| `idot`      | 14 bits | 2⁻⁴³ rad/s | Inclination rate             |

### Satellite Position Computation (ECEF)

Using ephemeris data for time `t`:

```text
1. Compute mean motion:
   n₀ = √(μ / a³),   μ = 3.986005×10¹⁴ m³/s²
   n  = n₀ + Δn

2. Time since TOE:
   tk = t − toe  (with week rollover correction)

3. Mean anomaly:
   Mk = M₀ + n·tk

4. Eccentric anomaly Ek (Kepler iterations):
   Ek = Mk + e·sin(Ek)  (≈5 iterations)

5. True anomaly:
   νk = atan2(√(1−e²)·sin(Ek), cos(Ek)−e)

6. Argument of latitude:
   Φk = νk + ω

7. Corrections:
   δuk = cus·sin(2Φk) + cuc·cos(2Φk)
   δrk = crs·sin(2Φk) + crc·cos(2Φk)
   δik = cis·sin(2Φk) + cic·cos(2Φk)

8. Corrected values:
   uk = Φk + δuk
   rk = a·(1 − e·cos(Ek)) + δrk
   ik = i₀ + δik + idot·tk

9. Position in orbital plane:
   xk' = rk·cos(uk)
   yk' = rk·sin(uk)

10. Longitude of ascending node:
    Ωk = Ω₀ + (Ω̇ − Ω̇e)·tk − Ω̇e·toe
    Ω̇e = 7.2921151467×10⁻⁵ rad/s

11. ECEF coordinates:
    x = xk'·cos(Ωk) − yk'·cos(ik)·sin(Ωk)
    y = xk'·sin(Ωk) + yk'·cos(ik)·cos(Ωk)
    z = yk'·sin(ik)
```

### Ionospheric Model (`nav_data.rs`)

Klobuchar model from Subframe 4 (page 18):

```text
Parameters: α₀ α₁ α₂ α₃ (amplitude)
            β₀ β₁ β₂ β₃ (period)

Correction (seconds):
  T_iono = F × (5×10⁻⁹ + A·cos(2π(t−50400)/P))
  F = 1 + 16(0.53−El)³

where El is satellite elevation in semicircles,
      A = Σαₙ·φₙ  (clamped to zero if negative),
      P = Σβₙ·φₙ  (minimum 72000 s)
```

### NavData (`nav_data.rs`)

Storage for current navigation data state:

```rust
pub struct NavData {
    /// Ephemerides for each PRN (1-32 for GPS)
    pub ephemeris: HashMap<u8, Ephemeris>,
    /// Ionospheric model parameters
    pub iono: Option<IonosphericModel>,
    /// Almanac (approximate orbits for all satellites)
    pub almanac: HashMap<u8, AlmanacEntry>,
    /// GPS–UTC correction (leap seconds)
    pub utc_correction: Option<UtcCorrection>,
}
```

**Ephemeris Validation:**

- Check `sv_health == 0` (health flag)
- Check `IODE == IODC` (data consistency)
- Check data age: `|t − toe| < 2 hours`

## Decoding Flow (1 epoch = 20 ms)

```text
tracking: I_prompt (navigation message bit)
    │
    ▼  I sign → bit (> 0 → 1, < 0 → 0)
bit_buffer.push(bit)
    │
    ▼  every 30 bits
check_parity(word)
    │ OK
    ▼  every 10 words (300 bits)
decode_subframe(bits[0..300])
    │
    ├── subframe_id == 1 → parse_clock_params()    → NavData.clock
    ├── subframe_id == 2 → parse_orbit_part1()     ┐
    ├── subframe_id == 3 → parse_orbit_part2()     ┘→ NavData.ephemeris[prn]
    └── subframe_id == 4 → parse_iono_or_almanac() → NavData.iono / .almanac
```

**Time to First Fix:**

- Subframes 1–3 decoded in 18 s (3 × 6 s)
- Full ephemeris set: ~30 s
- Warm start with valid almanac: ~6 s

## File Structure

```text
src/navigation/
├── mod.rs              — exports, NavigationState
├── frame_decoder.rs    — bit synchronization, parity, subframe assembly
├── ephemeris.rs        — SF1/SF2/SF3 decoding, satellite position
└── nav_data.rs         — NavData, IonosphericModel, Almanac, UtcCorrection
```

## Integration with Other Modules

| From     | Receives              | Produces         | To                            |
| -------- | --------------------- | ---------------- | ----------------------------- |
| Tracking | I_prompt (bit, 20 ms) | —                | —                             |
| —        | —                     | Ephemeris        | Observables (pseudorange)     |
| —        | —                     | IonosphericModel | Observables (iono correction) |
| —        | —                     | AlmanacEntry     | Acquisition (fast search)     |

## Target Metrics

| Metric                     | Target                         |
| -------------------------- | ------------------------------ |
| Bit sync                   | < 40 ms (2 bits)               |
| Frame sync (preamble lock) | < 6 s                          |
| Subframe 1–3 decode        | < 30 s                         |
| Parity error rate          | < 0.1% at CN0 > 35 dB-Hz       |
| Satellite position error   | < 1 m with ephemeris age < 2 h |
