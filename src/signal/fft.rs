//! FFT/IFFT и операции в частотной области.
//!
//! Модуль является обёрткой над [`rustfft`] и использует **кэшированный план
//! преобразования** и **scratch-буферы**, чтобы избежать повторных
//! выделений памяти.
//!
//! Предполагается, что **один экземпляр [`FftEngine`] используется для
//! фиксированного размера FFT** и переиспользуется многократно.
//!
//! Это особенно важно для DSP-задач реального времени
//! (например GNSS-корреляторов или спектрального анализа).

use std::sync::Arc;

use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

/// Кэшированный движок FFT/IFFT для фиксированного размера преобразования.
pub struct FftEngine {
    fft_plan: Arc<dyn Fft<f32>>,
    ifft_plan: Arc<dyn Fft<f32>>,
    scratch_fwd: Vec<Complex32>,
    scratch_inv: Vec<Complex32>,
    size: usize,
}

impl FftEngine {
    /// Создаёт новый FFT-движок для преобразований длины `size`.
    #[must_use]
    pub fn new(size: usize) -> Self {
        assert!(size > 0, "FFT size must be positive");

        let mut planner = FftPlanner::<f32>::new();
        let fft_plan = planner.plan_fft_forward(size);
        let ifft_plan = planner.plan_fft_inverse(size);
        let scratch_fwd = vec![Complex32::default(); fft_plan.get_inplace_scratch_len()];
        let scratch_inv = vec![Complex32::default(); ifft_plan.get_inplace_scratch_len()];

        Self {
            fft_plan,
            ifft_plan,
            scratch_fwd,
            scratch_inv,
            size,
        }
    }

    /// Размер преобразования, для которого был создан этот движок.
    #[must_use]
    pub const fn size(&self) -> usize {
        self.size
    }

    /// Выполняет **прямое FFT** над буфером *in-place*.
    pub fn fft_inplace(
        &mut self,
        buf: &mut [Complex32],
    ) {
        assert_eq!(buf.len(), self.size, "buffer length must FFT size");

        self.fft_plan
            .process_with_scratch(buf, &mut self.scratch_fwd);
    }

    /// Выполняет **обратное FFT** *in-place*.
    pub fn ifft_inplace(
        &mut self,
        buf: &mut [Complex32],
    ) {
        assert_eq!(buf.len(), self.size, "buffer length must match FFT size");

        self.ifft_plan
            .process_with_scratch(buf, &mut self.scratch_inv);

        let scale = 1.0 / self.size as f32;

        for s in buf.iter_mut() {
            *s *= scale;
        }
    }

    /// Выполняет прямое FFT и возвращает новый `Vec`.
    pub fn fft(
        &mut self,
        input: &[Complex32],
    ) -> Vec<Complex32> {
        assert_eq!(input.len(), self.size);

        let mut buf = input.to_vec();

        let () = self.fft_inplace(&mut buf);

        buf
    }

    /// Выполняет обратное FFT и возвращает новый `Vec`.
    pub fn ifft(
        &mut self,
        input: &[Complex32],
    ) -> Vec<Complex32> {
        assert_eq!(input.len(), self.size);

        let mut buf = input.to_vec();

        self.ifft_inplace(&mut buf);

        buf
    }

    /// Вычисляет **мощностной спектр** сигнала.
    pub fn power_spectrum(
        &mut self,
        input: &[Complex32],
    ) -> Vec<f32> {
        self.fft(input).into_iter().map(|s| s.norm_sqr()).collect()
    }

    /// Вычисляет мощностной спектр в **децибелах относительно полной шкалы
    /// (dBFS)**.
    pub fn power_spectrum_db(
        &mut self,
        input: &[Complex32],
    ) -> Vec<f32> {
        self.power_spectrum(input)
            .into_iter()
            .map(|p| 10.0 * p.max(1e-12_f32).log10())
            .collect()
    }

    /// Возвращает индекс бина с **максимальной мощностью**.
    pub fn peak_bin(
        &mut self,
        input: &[Complex32],
    ) -> usize {
        self.power_spectrum(input)
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map_or(0, |(i, _)| i)
    }

    /// Преобразует индекс FFT-бина в частоту (Гц).
    #[must_use]
    pub fn bin_to_freq(
        &self,
        bin: usize,
        sample_rate_hz: f64,
    ) -> f64 {
        let k = bin.cast_signed();
        let n = self.size.cast_signed();
        let shifted = if k > n / 2 { k - n } else { k };

        shifted as f64 * sample_rate_hz / n as f64
    }

    /// Циклическая взаимная корреляция через FFT.
    pub fn cross_correlate_power(
        &mut self,
        signal: &[Complex32],
        template: &[Complex32],
    ) -> Vec<f32> {
        assert_eq!(signal.len(), self.size);
        assert_eq!(template.len(), self.size);

        let mut sig = signal.to_vec();
        let mut tmpl = template.to_vec();

        let () = self.fft_inplace(&mut sig);
        let () = self.fft_inplace(&mut tmpl);

        let mut product: Vec<Complex32> = sig
            .iter()
            .zip(tmpl.iter())
            .map(|(s, t)| s * t.conj())
            .collect();

        self.ifft_inplace(&mut product);

        product.into_iter().map(|s| s.norm_sqr()).collect()
    }

    /// То же самое, что `cross_correlate_power`, но возвращает **комплексный
    /// результат IFFT**.
    ///
    /// Это позволяет сохранить **фазовую информацию**
    /// корреляции.
    pub fn cross_correlate(
        &mut self,
        signal: &[Complex32],
        template: &[Complex32],
    ) -> Vec<Complex32> {
        assert_eq!(signal.len(), self.size);
        assert_eq!(template.len(), self.size);

        let mut sig = signal.to_vec();
        let mut tmpl = template.to_vec();

        let () = self.fft_inplace(&mut sig);
        let () = self.fft_inplace(&mut tmpl);

        let mut product: Vec<Complex32> = sig
            .iter()
            .zip(tmpl.iter())
            .map(|(s, t)| s * t.conj())
            .collect();

        self.ifft_inplace(&mut product);

        product
    }

    /// Представляет элементы спектра так, чтобы **DC-компонента оказалась в
    /// центре массива**.
    #[must_use]
    pub fn fftshift(input: &[Complex32]) -> Vec<Complex32> {
        let n = input.len();
        let half = n / 2;
        let mut out = Vec::with_capacity(n);

        out.extend_from_slice(&input[half..]);
        out.extend_from_slice(&input[..half]);

        out
    }
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use std::f64::consts::TAU;

    use super::*;

    const FS: f64 = 2_048_000.0;

    #[test]
    fn test_fft_roundtrip() {
        let n = 256;
        let mut engine = FftEngine::new(n);
        let original: Vec<Complex32> = (0..n)
            .map(|i| Complex32::new((i as f32 * 0.1).sin(), (i as f32 * 0.07).cos()))
            .collect();
        let spectrum = engine.fft(&original);
        let recovered = engine.ifft(&spectrum);

        for (x, y) in original.iter().zip(recovered.iter()) {
            assert!((x.re - y.re).abs() < 1e-4, "re: {} vs {}", x.re, y.re);
            assert!((x.im - y.im).abs() < 1e-4, "im: {} vs {}", x.im, y.im);
        }
    }

    #[test]
    fn test_fft_of_dc_has_single_peak_at_bin_zero() {
        let n = 64;
        let mut engine = FftEngine::new(n);
        let dc = vec![Complex32::new(1.0, 0.0); n];
        let spectrum = engine.power_spectrum(&dc);
        let peal_bin = spectrum
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)
            .unwrap();

        assert_eq!(peal_bin, 0);

        for (k, &p) in spectrum.iter().enumerate().skip(1) {
            assert!(p < 1e-6, "bin {k} power={p}");
        }
    }

    #[test]
    fn test_fft_single_tone_peak_at_correct_bin() {
        let n = 1024;
        let fs = 1024.0_f64;
        let mut engine = FftEngine::new(n);
        let f = 10.0_f64;
        let signal: Vec<Complex32> = (0..n)
            .map(|i| {
                let phase = TAU * f * i as f64 / fs;

                Complex32::new(phase.cos() as f32, phase.sin() as f32)
            })
            .collect();

        let peak = engine.peak_bin(&signal);

        assert_eq!(peak, 10, "expected peak at bin 10, got {peak}");
    }

    #[test]
    fn test_ifft_normalises_by_n() {
        let n = 64;
        let mut engine = FftEngine::new(n);
        let dc = vec![Complex32::new(1.0, 0.0); n];
        let spectrum = engine.fft(&dc);

        assert!((spectrum[0].re - n as f32).abs() < 0.1);

        let recovered = engine.ifft(&spectrum);

        for s in &recovered {
            assert!((s.re - 1.0).abs() < 1e-4, "re={}", s.re);
            assert!(s.im.abs() < 1e-4, "im={}", s.im);
        }
    }

    #[test]
    fn test_cross_correlate_power_peak_at_zero_for_identical_signals() {
        let n = 512;
        let mut engine = FftEngine::new(n);
        let signal: Vec<Complex32> = (0..n)
            .map(|i| {
                let v = if i % 7 < 3 { 1.0f32 } else { -1.0f32 };

                Complex32::new(v, 0.0)
            })
            .collect();
        let power = engine.cross_correlate_power(&signal, &signal);
        let peak_idx = power
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)
            .unwrap();

        assert_eq!(peak_idx, 0, "expected peak at lag=0, got {peak_idx}");
    }

    #[test]
    fn test_cross_correlate_power_detects_shift() {
        let n = 512;
        let shift = 42;
        let mut engine = FftEngine::new(n);
        let signal: Vec<Complex32> = (0..n)
            .map(|i| Complex32::new(if i % 11 < 5 { 1.0 } else { -1.0 }, 0.0))
            .collect();
        let mut template = vec![Complex32::default(); n];

        for i in 0..n {
            template[(i + shift) % n] = signal[i];
        }

        let power = engine.cross_correlate_power(&signal, &template);
        let peak_idx = power
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)
            .unwrap();

        assert!(
            peak_idx == n - shift || peak_idx == shift,
            "expected peak near {} or {}, got {}",
            n - shift,
            shift,
            peak_idx
        );
    }

    #[test]
    fn test_bin_freq_dc_is_zero() {
        let e = FftEngine::new(1024);

        assert!((e.bin_to_freq(0, FS) - 0.0).abs() < 1e-9, "n={}", 1024);
    }

    #[test]
    fn test_bin_to_freq_nyquist() {
        let n = 1024;
        let e = FftEngine::new(n);
        let f_nyq = FS / 2.0;

        assert!((e.bin_to_freq(n / 2, FS) - f_nyq).abs() < 1.0);
    }

    #[test]
    fn test_bin_to_freq_negative_frequencies() {
        let n = 1024;
        let e = FftEngine::new(n);
        let f_neg = -(FS / n as f64);

        assert!((e.bin_to_freq(n - 1, FS) - f_neg).abs() < 1.0);
    }

    #[test]
    fn test_fftshift_moves_dc_to_centre() {
        let n = 8;
        let input: Vec<Complex32> = (0..n).map(|k| Complex32::new(k as f32, 0.0)).collect();
        let shifted = FftEngine::fftshift(&input);

        assert!((shifted[n / 2].re - 0.0).abs() < 1e-9, "n={n}");
    }

    #[test]
    fn test_fftshift_double_shift_is_identity() {
        let n = 16;
        let input: Vec<Complex32> = (0..n).map(|k| Complex32::new(k as f32, 0.0)).collect();
        let shifted_twice = FftEngine::fftshift(&FftEngine::fftshift(&input));

        for (a, b) in input.iter().zip(shifted_twice.iter()) {
            assert_eq!(a, b);
        }
    }

    #[test]
    fn test_power_spectrum_db_length_matches_input() {
        let mut e = FftEngine::new(256);
        let input = vec![Complex32::new(0.5, 0.3); 256];

        assert_eq!(e.power_spectrum_db(&input).len(), 256);
    }

    #[test]
    fn test_fft_inplace_same_as_fft() {
        let n = 64;
        let mut e = FftEngine::new(n);
        let input: Vec<Complex32> = (0..n)
            .map(|i| Complex32::new((i as f32).sin(), 0.0))
            .collect();
        let out_alloc = e.fft(&input);
        let mut out_inplace = input.clone();

        let () = e.fft_inplace(&mut out_inplace);

        for (a, b) in out_alloc.iter().zip(out_inplace.iter()) {
            assert!((a.re - b.re).abs() < 1e-4);
            assert!((a.im - b.im).abs() < 1e-4);
        }
    }
}
