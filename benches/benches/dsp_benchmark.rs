//! DSP throughput benchmarks.
//!
//! Run with: `cargo bench --bench dsp_benchmark`
//!
//! Benchmarks use realistic GNSS block sizes:
//! * 2048 samples = 1 ms at 2.048 Msps (GPS L1 C/A standard)
//! * 4096 samples = 2 ms integration
//! * 8192 samples = wider acquisition window

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use glrx::signal::{
    correlator::{
        base::{correlate, correlator_epl},
        code_utilities::shift_code,
        normalisation::{compute_power, normalize},
    },
    fft::FftEngine,
    filter::{FirFilter, Window},
    mixer::{generate_carrier, mix_shift, Mixer},
    resampler::Decimator,
};
use num_complex::Complex32;
use std::hint::black_box;

const FS: f64 = 2_048_000.0;
const BLOCK: usize = 2048; // 1 ms at 2.048 Msps

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

fn make_iq_block(n: usize) -> Vec<Complex32> {
    let mut nco_f = 0.0_f32;
    let step = std::f32::consts::TAU * 10_000.0 / FS as f32;
    (0..n)
        .map(|_| {
            let (s, c) = nco_f.sin_cos();
            nco_f += step;
            Complex32::new(c, s)
        })
        .collect()
}

fn make_bpsk_code(n: usize) -> Vec<f32> {
    (0..n)
        .map(|i| {
            if (i * 7 + 3) % 17 < 9 {
                1.0_f32
            } else {
                -1.0_f32
            }
        })
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Mixer
// ─────────────────────────────────────────────────────────────────────────────

fn bench_mixer(c: &mut Criterion) {
    let input = make_iq_block(BLOCK);

    let mut group = c.benchmark_group("mixer");
    group.throughput(Throughput::Elements(BLOCK as u64));

    group.bench_function("Mixer::mix (2048)", |b| {
        let mut mixer = Mixer::new(10_000.0, FS);
        b.iter(|| mixer.mix(black_box(&input)));
    });

    group.bench_function("Mixer::mix_inplace (2048)", |b| {
        let mut mixer = Mixer::new(10_000.0, FS);
        let mut buf = input.clone();
        b.iter(|| {
            mixer.mix_inplace(black_box(&mut buf));
        });
    });

    group.bench_function("mix_shift / stateless (2048)", |b| {
        b.iter(|| mix_shift(black_box(&input), 10_000.0, FS));
    });

    group.bench_function("generate_carrier (2048)", |b| {
        b.iter(|| generate_carrier(black_box(1_575_420_000.0), FS, BLOCK));
    });

    group.finish();
}

// ─────────────────────────────────────────────────────────────────────────────
// FIR filter
// ─────────────────────────────────────────────────────────────────────────────

fn bench_filter(c: &mut Criterion) {
    let input = make_iq_block(BLOCK);

    let mut group = c.benchmark_group("fir_filter");
    group.throughput(Throughput::Elements(BLOCK as u64));

    for &taps in &[15usize, 31, 63, 127] {
        group.bench_with_input(BenchmarkId::new("apply", taps), &taps, |b, &taps| {
            let mut f = FirFilter::low_pass(500_000.0, FS, taps, Window::Hamming);
            b.iter(|| f.apply(black_box(&input)));
        });

        group.bench_with_input(
            BenchmarkId::new("apply_inplace", taps),
            &taps,
            |b, &taps| {
                let mut f = FirFilter::low_pass(500_000.0, FS, taps, Window::Hamming);
                let mut buf = input.clone();
                b.iter(|| f.apply_inplace(black_box(&mut buf)));
            },
        );
    }

    group.finish();
}

// ─────────────────────────────────────────────────────────────────────────────
// Decimation
// ─────────────────────────────────────────────────────────────────────────────

fn bench_decimation(c: &mut Criterion) {
    let input = make_iq_block(BLOCK);

    let mut group = c.benchmark_group("decimation");

    for &factor in &[2usize, 4, 8] {
        group.throughput(Throughput::Elements((BLOCK / factor) as u64));
        group.bench_with_input(
            BenchmarkId::new("Decimator::decimate", factor),
            &factor,
            |b, &factor| {
                let mut dec = Decimator::new(factor);
                b.iter(|| dec.decimate(black_box(&input)));
            },
        );
    }

    group.finish();
}

// ─────────────────────────────────────────────────────────────────────────────
// FFT
// ─────────────────────────────────────────────────────────────────────────────

fn bench_fft(c: &mut Criterion) {
    let mut group = c.benchmark_group("fft");

    for &size in &[512usize, 1024, 2048, 4096] {
        let input = make_iq_block(size);
        group.throughput(Throughput::Elements(size as u64));

        group.bench_with_input(BenchmarkId::new("forward", size), &size, |b, &size| {
            let mut engine = FftEngine::new(size);
            b.iter(|| engine.fft(black_box(&input)));
        });

        group.bench_with_input(BenchmarkId::new("inverse", size), &size, |b, &size| {
            let mut engine = FftEngine::new(size);
            let spectrum = engine.fft(&input);
            b.iter(|| engine.ifft(black_box(&spectrum)));
        });

        group.bench_with_input(
            BenchmarkId::new("cross_correlate_power", size),
            &size,
            |b, &size| {
                let mut engine = FftEngine::new(size);
                let signal = make_iq_block(size);
                let template: Vec<Complex32> = make_bpsk_code(size)
                    .into_iter()
                    .map(|c| Complex32::new(c, 0.0))
                    .collect();
                b.iter(|| engine.cross_correlate_power(black_box(&signal), black_box(&template)));
            },
        );
    }

    group.finish();
}

// ─────────────────────────────────────────────────────────────────────────────
// Correlator
// ─────────────────────────────────────────────────────────────────────────────

fn bench_correlator(c: &mut Criterion) {
    let signal = make_iq_block(BLOCK);
    let code = make_bpsk_code(BLOCK);
    let code_e = shift_code(&code, -1.0); // early
    let code_l = shift_code(&code, 1.0); // late

    let mut group = c.benchmark_group("correlator");
    group.throughput(Throughput::Elements(BLOCK as u64));

    group.bench_function("correlate_single (2048)", |b| {
        b.iter(|| correlate(black_box(&signal), black_box(&code)));
    });

    group.bench_function("correlator_epl (2048)", |b| {
        b.iter(|| {
            correlator_epl(
                black_box(&signal),
                black_box(&code_e),
                black_box(&code),
                black_box(&code_l),
            )
        });
    });

    group.bench_function("shift_code (2048, frac=0.5)", |b| {
        b.iter(|| shift_code(black_box(&code), 0.5));
    });

    group.finish();
}

// ─────────────────────────────────────────────────────────────────────────────
// Power & normalisation
// ─────────────────────────────────────────────────────────────────────────────

fn bench_power_and_norm(c: &mut Criterion) {
    let signal = make_iq_block(BLOCK);

    let mut group = c.benchmark_group("power_norm");
    group.throughput(Throughput::Elements(BLOCK as u64));

    group.bench_function("compute_power (2048)", |b| {
        b.iter(|| compute_power(black_box(&signal)));
    });

    group.bench_function("normalize (2048)", |b| {
        let mut buf = signal.clone();
        b.iter(|| normalize(black_box(&mut buf)));
    });

    group.finish();
}

// ─────────────────────────────────────────────────────────────────────────────
// Registration
// ─────────────────────────────────────────────────────────────────────────────

criterion_group!(
    dsp_benches,
    bench_mixer,
    bench_filter,
    bench_decimation,
    bench_fft,
    bench_correlator,
    bench_power_and_norm,
);
criterion_main!(dsp_benches);
