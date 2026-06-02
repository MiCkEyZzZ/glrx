# Конвейер приёма — GLRX Pipeline

Module:

```text
src/pipeline/
```

## Общая схема

GGLRX processes signals through a multi-stage pipeline.

```text
┌─────────────┐
│   IQ Source │  RF frontend: file / SDR / stream
│  (src/rf/)  │
└──────┬──────┘
       │ IqBlock (raw IQ, normalized ±1.0)
       ▼
┌──────────────────┐
│ Signal Processing│  mixer, filter, resampler, FFT, correlator
│ (src/signal/)    │
└────────┬─────────┘
         │ SignalBlock (baseband IQ after downconversion)
         ▼
┌──────────────────┐
│  Acquisition     │  PCPS search: PRN × Doppler grid
│(src/acquisition/)│
└──────┬───────────┘
       │ AcquisitionResult { prn, doppler_hz, code_phase }
       ▼
┌───────────────┐
│   Tracking    │  DLL + PLL + FLL per channel
│(src/tracking/)│
└──────┬────────┘
       │ TrackingState { epl, cn0, lock }
       ▼
┌──────────────────┐
│   Navigation     │  Bit sync, frame decode, ephemeris
│(src/navigation/) │
└──────┬───────────┘
       │ Ephemeris, NavData
       ▼
┌──────────────────┐
│  Observables     │  Pseudorange, Doppler, CN0
│(src/observables/)│
└──────┬───────────┘
       │ Observable { prn, pseudorange, doppler, cn0 }
       ▼
┌──────────────┐
│    Solver    │  WLS / Kalman → ECEF → LLA
│ (src/solver/)│
└──────┬───────┘
       │ PositionSolution { lat, lon, alt, clock_bias, dop }
       ▼
┌──────────────┐
│    Output    │  NMEA, UBX, telemetry
│ (src/output/)│
└──────────────┘
```

## Данные на каждом этапе

### IqBlock (RF -> Signal)

```rust
IqBlock {
    samples: Vec<Complex32>,  // normalized IQ samples in range ±1.0
    config: Arc<RfConfig>,    // fs, center_freq, format
    start_sample: u64,        // monotonic sample counter
}
```

### AcquisitionResult (Acquisition → Tracking)

```rust
AcquisitionResult {
    prn: u8,           // GPS PRN 1–32
    doppler_hz: f64,   // estimated Doppler shift
    code_phase: usize, // code phase in samples
    cn0_db: f32,       // estimated C/N₀
}
```

### Observable (Tracking → Solver)

```rust
Observable {
    prn: u8,
    pseudorange: f64,   // meters (corrected)
    doppler: f64,       // Hz
    cn0: f32,           // dB-Hz
    timestamp: f64,     // GPS time (seconds)
}
```

### PositionSolution (Solver → Output)

```rust
PositionSolution {
    lat: f64,           // degrees
    lon: f64,
    alt: f64,           // meters above ellipsoid
    clock_bias: f64,    // receiver clock offset (meters)
    hdop: f32,
    vdop: f32,
    num_satellites: u8,
}
```

## Состояния приёмника

```text
COLD_START
    │
    ▼ (IQ данные доступны)
ACQUIRING
    │
    ▼ (найдено ≥ 4 спутников)
TRACKING
    │
    ▼ (декодированы эфемериды)
NAVIGATING
    │
    ▼ (≥ 4 observables)
FIXED (position solution available)
```

## Временной бюджет (GPS L1 C/A, 2.048 Msps)

| Этап                | Единица времени     | Типичная задержка     |
| ------------------- | ------------------- | --------------------- |
| IQ capture          | 1 ms (2048 samples) | real-time             |
| Signal processing   | 1 ms                | < 1 ms                |
| Acquisition (1 PRN) | 1–100 ms            | < 500 ms (all 32 PRN) |
| Tracking lock       | 1–20 ms             | 5–20 ms               |
| Nav bit sync        | 20 ms               | ~100 ms               |
| Subframe decode     | 6 s                 | 6–30 s                |
| Position fix        | —                   | TTFF 30–90 s (cold)   |
