# GLRX — GNSS Layered Receiver

[![CI](https://github.com/MiCkEyZzZ/glrx/actions/workflows/ci.yml/badge.svg)](https://github.com/MiCkEyZzZ/glrx/actions)
[![Rust](https://img.shields.io/badge/rust-1.96+-orange.svg)](https://www.rust-lang.org/)

**GLRX** is a modular GNSS SDR receiver implemented in Rust, focused on
layered DSP pipelines, satellite tracking, navigation message decoding,
and precise positioning.

## Architecture

The diagram below represents the current processing flow implemented in
GLRX.

```mermaid
flowchart TD

A[Raw I/O Data] --> B[Signal Separation]

B --> B1[GPS]
B --> B2[GLONASS]
B --> B3[Galileo]
B --> B4[BeiDou]

B1 --> C[Acquisition]
B2 --> C
B3 --> C
B4 --> C

C --> D[Acquisition Results]
D --> E[USMET]

D --> F[Tracking]

F --> G[Tracking Results]
G --> H[USMET]

G --> I[Multi-GNSS Navigation]

I --> J[PVT Solution]
```

## Planned Workspace Architecture

As GLRX evolves, the project is planned to transition into a layered
multi-crate workspace. This architecture separates hardware interfaces,
DSP algorithms, navigation logic, shared types, and user-facing
applications while keeping each component independently testable.

```mermaid
flowchart TD

A[glrx-cli] --> B[glrx-receiver]

S[glrx-sdr] --> B

B --> C[Signal Orchestration Layer]

C --> D[glrx-dsp]

D --> D1[Acquisition Algorithms]
D --> D2[Tracking Loops]

D1 --> E[glrx-core]
D2 --> E

E --> E1[Navigation Engine]
E --> E2[Time / State Estimation]

E1 --> F[PVT Solution]

E2 --> F

E --> G[glrx-types]

D --> G

E --> G

E --> H[glrx-error]

B --> H
D --> H
E --> H
```

## Features

- Read I/Q data from files or SDR devices.
- FFT-based satellite acquisition.
- DLL, PLL and FLL tracking loops.
- Navigation message decoding.
- GPS ephemeris decoding.
- Multi-GNSS navigation pipeline.
- Pseudorange computation.
- Position estimation (Least Squares / Kalman).
- Multi-channel receiver architecture.
- Export to NMEA / UBX.
- Integration with **USMET**, **GLINT**, and **GLOS**.
- Extensive unit tests, benchmarks and documentation.

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

### Run (simulator)

```bash
cargo run --release --bin glrx -- \
  --device sim \
  --freq 1602MHz \
  --rate 2MHz \
  --gain 40 \
  --duration 5
```

## Development Roadmap

- DSP primitives
- Satellite acquisition
- Tracking loops
- Navigation message decoding
- Ephemeris decoding
- Pseudorange computation
- PVT solver
- Multi-GNSS support
- NMEA / UBX output
- SDR device integration
- Workspace modularization
- Performance optimization
- Observability and validation

## Goals

GLRX aims to provide a modern, modular GNSS SDR receiver written entirely
in Rust with a strong emphasis on correctness, maintainability,
reproducibility, and extensibility.

The long-term objective is to build a reusable ecosystem of crates for
GNSS signal processing, navigation, and positioning that can be used in
both research and production environments.

## License

This project is licensed under either of

- [Apache License, Version 2.0](LICENSE.APACHE)
- [MIT License](LICENSE.MIT)
