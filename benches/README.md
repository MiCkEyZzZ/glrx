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
| `mix_shift / stateless (2048)` | ~15.00 µs | ~136.4 Melem/s |
| `generate_carrier (2048)`      | ~40.03 µs | ~51.1 Melem/s  |

### FIR Filter

| Operation (taps)      | Time       | Throughput    |
| --------------------- | ---------- | ------------- |
| `apply (15)`          | ~32.27 µs  | ~63.5 Melem/s |
| `apply_inplace (15)`  | ~30.23 µs  | ~67.7 Melem/s |
| `apply (31)`          | ~58.77 µs  | ~34.8 Melem/s |
| `apply_inplace (31)`  | ~57.72 µs  | ~35.5 Melem/s |
| `apply (63)`          | ~119.79 µs | ~17.1 Melem/s |
| `apply_inplace (63)`  | ~117.06 µs | ~17.5 Melem/s |
| `apply (127)`         | ~252.49 µs | ~8.11 Melem/s |
| `apply_inplace (127)` | ~257.52 µs | ~7.95 Melem/s |

### Decimation

| Operation                 | Time       | Throughput    |
| ------------------------- | ---------- | ------------- |
| `Decimator::decimate (2)` | ~119.50 µs | ~8.57 Melem/s |
| `Decimator::decimate (4)` | ~119.76 µs | ~4.28 Melem/s |
| `Decimator::decimate (8)` | ~120.70 µs | ~2.12 Melem/s |

### FFT

| Operation        | Time     | Throughput   |
| ---------------- | -------- | ------------ |
| `forward (512)`  | ~573 ns  | ~893 Melem/s |
| `inverse (512)`  | ~636 ns  | ~804 Melem/s |
| `forward (1024)` | ~1.33 µs | ~770 Melem/s |
| `inverse (1024)` | ~1.38 µs | ~742 Melem/s |
| `forward (2048)` | ~3.12 µs | ~656 Melem/s |
| `inverse (2048)` | ~3.05 µs | ~671 Melem/s |
| `forward (4096)` | ~7.53 µs | ~544 Melem/s |
| `inverse (4096)` | ~6.85 µs | ~598 Melem/s |

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
