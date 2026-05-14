# GLRX — GNSS Receiver

![Rust](https://img.shields.io/badge/language-Rust-orange)
![Status](https://img.shields.io/badge/status-pre--alpha-yellow)

**GLRX** is a modular GNSS receiver written in Rust, designed for research and
engineering experiments with satellite navigation signals such as GLONASS, GPS,
and other GNSS constellations.

The project focuses on reproducibility, modular DSP pipelines, and integration
with external telemetry and analysis tools.

## GLRX – GNSS Receiver Pipeline Overview

```mermaid
flowchart TD

A[IQ Source<br/>FileSource / SDR] --> B[Signal Layer]

B --> C[Acquisition]

C --> D[Tracking]

D --> E[Navigation]

E --> F[Solver]

F --> G[Output]

%% Signal layer internals
subgraph Signal
B1[Mixer / NCO]
B2[Filters]
B3[Resampler]
B4[Correlation Utils]

B --> B1
B1 --> B2
B2 --> B3
B3 --> B4
end

%% Acquisition internals
subgraph AcquisitionLayer
C1[PRN Generator]
C2[FFT Correlator]
C3[Peak Detector]

C --> C1
C1 --> C2
C2 --> C3
end

%% Tracking internals
subgraph TrackingLayer
D1[DLL]
D2[PLL]
D3[FLL]
D4[Channel Manager]

D --> D4
D4 --> D1
D4 --> D2
D4 --> D3
end

%% Navigation
subgraph NavigationLayer
E1[Frame Decoder]
E2[Ephemeris Parser]
E3[Navigation Data]

E --> E1
E1 --> E2
E2 --> E3
end

%% Solver
subgraph SolverLayer
F1[Least Squares]
F2[Kalman Filter]

F --> F1
F1 --> F2
end

%% Output
subgraph OutputLayer
G1[NMEA]
G2[UBX]
G3[Telemetry]

G --> G1
G --> G2
G --> G3
end
```

## Features

- Read IQ data from files or SDR devices.
- Basic signal processing primitives: mixing, filtering, resampling.
- Satellite acquisition using FFT-based search.
- Tracking loops: DLL, PLL, FLL.
- Navigation message and ephemeris decoding.
- Pseudorange computation and position estimation (Least Squares, Kalman filtering).
- Multi-channel satellite tracking.
- Output in standard formats (NMEA / UBX).
- Integration with external tools: **GLOS**, **GLINT**, **USMET**.
- Built-in support for testing, benchmarking, and observability.

## Quick Start

### Requirements

- Rust (stable toolchain)
- Optional SDR hardware:
  - SoapySDR
  - RTL-SDR
  - HackRF

### Build

```bash
cargo build --workspace --release
```

### Run tests

```bash
cargo test --workspace
```

### Run (simulator mode)

```bash
cargo run --release --bin glrx -- \
  --device sim \
  --freq 1602MHz \
  --rate 2MHz \
  --gain 40 \
  --duration 5
```

## Development Roadmap

1. IQ reader and DSP primitives
2. FFT-based satellite acquisition
3. Tracking loops (DLL / PLL / FLL) and multi-channel tracking
4. Navigation message and ephemeris decoding
5. Pseudorange computation and position solver
6. NMEA / UBX output and integration with GLINT / USMET
7. High-level processing pipeline and time synchronization
8. Performance optimization (SIMD, latency)
9. Observability, testing, and validation

## Goals

- Build a modular GNSS receiver implemented entirely in Rust.
- Ensure experiment reproducibility and system extensibility.
- Enable seamless integration with telemetry analysis and storage systems.

## License

This project is licensed under either of

- [Apache License, Version 2.0](LICENSE.APACHE)
- [MIT License](LICENSE.MIT)
