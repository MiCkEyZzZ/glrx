# Tracking Layer — Satellite Signal Tracking

Module:

```text
src/tracking/
```

## Purpose

The tracking layer provides continuous synchronization with satellite signals
after the acquisition stage. Each satellite is processed in an independent tracking
channel.

---

## Tracking Channel

```rust
struct TrackingChannel {
    prn: u8,
    pll: Pll,              // carrier phase lock
    dll: Dll,              // code delay lock
    fll: Fll,              // frequency lock (assists PLL during pull-in)
    cn0_estimator: Cn0Estimator,
    state: ChannelState,
    prompt_history: Vec<Complex32>, // for C/N₀ estimation
}
```

### Channel States

```text
IDLE
  │ ← AcquisitionResult
  ▼
FLL_LOCK   (frequency locked, phase not yet locked)
  │ (transition when FLL is stable)
  ▼
PLL_LOCK   (phase locked, data readable)
  │ (transition when navigation bits are synchronized)
  ▼
BIT_SYNC   (navigation bits are being decoded)
```

---

## Tracking Loops

### DLL — Code Delay Lock Loop

Tracks PRN code phase.

```text
correlator_epl(signal, early, prompt, late)
    │
    ▼
dll_nelp() = (|E|² − |L|²) / (|E|² + |L|²)
    │
    ▼
Loop filter (2nd order)
    │
    ▼
Code NCO correction (samples/s)
```

**Parameters:**

- `chip_spacing`: 0.1–1.0 chips (early-late spacing)
- `bandwidth`: 1–5 Hz (loop bandwidth)
- `order`: 2 (position + velocity)

### PLL — Phase Lock Loop

Tracks carrier phase.

```text
epl.pll_dd_atan() = atan(Q_P / |I_P|)
    │
    ▼
Loop filter (3rd order for high dynamics)
    │
    ▼
Carrier NCO correction (Hz)
```

**Parameters:**

- `bandwidth`: 10–25 Hz
- `order`: 3 (phase + frequency + frequency rate)
- Uses DD-atan to remove bit ambiguity

### FLL — Frequency Lock Loop

Assists PLL during initial acquisition.

```text
cross_product_discriminator(P_prev, P_curr)
    │
    ▼
Loop filter (1st order)
    │
    ▼
Carrier NCO correction (frequency only, no phase)
```

**FLL → PLL switching:**

- FLL is active until phase lock is achieved
- After PLL lock, FLL is disabled
- On lock loss: fallback to FLL (or full reacquisition)

---

## C/N₀ Estimation

```rust
// Accumulate 20 prompt samples (20 ms)
prompt_history.push(epl.prompt);
if prompt_history.len() >= 20 {
    let cn0 = cn0_estimate(&prompt_history, 0.001);
    // Typical values: 35–50 dB-Hz
}
```

**Thresholds:**

- `> 40 дБ-Гц` — reliable PLL lock
- `35–40 дБ-Гц` — unstable lock, possible loss
- `< 35 дБ-Гц` — lock loss, reacquisition required

---

## Multi-Channel

The receiver supports N parallel channels (8/16/32).

```text
IqBlock (2048 samples, 1 ms)
    │
    ├─→ Channel[G01]: correlator_epl → DLL/PLL
    ├─→ Channel[G05]: correlator_epl → DLL/PLL
    ├─→ Channel[G12]: correlator_epl → DLL/PLL
    └─→ Channel[G20]: correlator_epl → DLL/PLL
```

Parallel processing via Rayon or tokio::task::spawn_blocking.

---

## Loop Filter Design

Typical 2nd-order DLL filter:

```text
bandwidth = 2 Hz, damping = 0.707

ω_n = bandwidth * 8 * damping / (4 * damping² + 1)
τ_1 = 1 / ω_n²
τ_2 = 2 * damping / ω_n

y[k] = y[k-1] + (τ_2/τ_1 + T/τ_1) * e[k] - τ_2/τ_1 * e[k-1]
```

where `T = 0.001 с` (integration period), `e[k]` is discriminator output.

---

## Planned File Structure

```text
src/tracking/
├── mod.rs          — exports, TrackingState
├── channel.rs      — TrackingChannel, ChannelState
├── dll.rs          — Dll struct, loop filter
├── pll.rs          — Pll struct, loop filter
└── fll.rs          — Fll struct, cross-product discriminator
```
