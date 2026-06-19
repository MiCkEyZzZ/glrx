# Benchmarks for GLRX

This is directory contains benchmarks used to verify zero-cost abstractions and
the performance of time conversions.

## Running

Run all benchmarks:

```bash
cargo bench
```

Run a specific benchmark:

```bash
cargo bench --bench dsp_benchmark
```

## Results

### Mixer

| Operation                      | Time      | Throughput     |
| ------------------------------ | --------- | -------------- |
| `Mixer::mix (2048)`            | ~14.84 µs | ~137.0 Melem/s |
| `Mixer::mix_inplace (2048)`    | ~14.61 µs | ~140.1 Melem/s |
| `mix_shift / stateless (2048)` | ~14.88 µs | ~137.6 Melem/s |
| `generate_carrier (2048)`      | ~40.03 µs | ~50.9 Melem/s  |

### FIR Filter

| Operation (taps)      | Time       | Throughput     |
| --------------------- | ---------- | -------------- |
| `apply (15)`          | ~29.61 µs  | ~69.15 Melem/s |
| `apply_inplace (15)`  | ~28.57 µs  | ~71.6 Melem/s  |
| `apply (31)`          | ~54.74 µs  | ~37.4 Melem/s  |
| `apply_inplace (31)`  | ~53.33 µs  | ~38.4 Melem/s  |
| `apply (63)`          | ~113.75 µs | ~18.0 Melem/s  |
| `apply_inplace (63)`  | ~112.96 µs | ~18.1 Melem/s  |
| `apply (127)`         | ~248.62 µs | ~8.23 Melem/s  |
| `apply_inplace (127)` | ~248.50 µs | ~8.24 Melem/s  |

### Decimation

| Operation                 | Time       | Throughput    |
| ------------------------- | ---------- | ------------- |
| `Decimator::decimate (2)` | ~116.97 µs | ~8.75 Melem/s |
| `Decimator::decimate (4)` | ~115.27 µs | ~4.44 Melem/s |
| `Decimator::decimate (8)` | ~115.44 µs | ~2.71 Melem/s |

### FFT

| Operation        | Time       | Throughput   |
| ---------------- | ---------- | ------------ |
| `forward (512)`  | ~517.79 ns | ~988 Melem/s |
| `inverse (512)`  | ~621.89 ns | ~823 Melem/s |
| `forward (1024)` | ~1.33 µs   | ~770 Melem/s |
| `inverse (1024)` | ~1.38 µs   | ~742 Melem/s |
| `forward (2048)` | ~3.12 µs   | ~656 Melem/s |
| `inverse (2048)` | ~3.05 µs   | ~671 Melem/s |
| `forward (4096)` | ~7.53 µs   | ~544 Melem/s |
| `inverse (4096)` | ~6.85 µs   | ~598 Melem/s |

### Cross-correlation

| Operation                      | Time      | Throughput   |
| ------------------------------ | --------- | ------------ |
| `cross_correlate_power (512)`  | ~2.12 µs  | ~241 Melem/s |
| `cross_correlate_power (1024)` | ~4.73 µs  | ~216 Melem/s |
| `cross_correlate_power (2048)` | ~10.82 µs | ~189 Melem/s |
| `cross_correlate_power (4096)` | ~24.48 µs | ~167 Melem/s |

### Correlator

| Operation                 | Time      | Throughput   |
| ------------------------- | --------- | ------------ |
| `correlate_single (2048)` | ~2.07 µs  | ~990 Melem/s |
| `correlator_epl (2048)`   | ~2.09 µs  | ~981 Melem/s |
| `shift_code (frac=0.5)`   | ~13.46 µs | ~152 Melem/s |

### Power & Normalization

| Operation              | Time     | Throughput   |
| ---------------------- | -------- | ------------ |
| `compute_power (2048)` | ~2.08 µs | ~985 Melem/s |
| `normalize (2048)`     | ~2.37 µs | ~864 Melem/s |
