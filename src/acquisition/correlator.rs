//! FFT-базированный коррелятор захвата (Parallel Code Search - PCPS).
//!
//! Этот модуль объединяет [`PrnCodeCache`] с [`FftEngine`] для реализации
//! основного алгоритма GNSS-захвата: поиска PRN-кода спутника по всем
//! фазам кода одновременно с использованием FFT.
//!
//! # Алгоритм — Parallel Code Search (PCPS)
//!
//! Для каждой пробной доплеровской частоты `f_d`:
//!
//! ```text
//! 1. Смешивание IQ-блока с exp(−j·2π·f_d·t)     → подавление несущей
//! 2. FFT(смешанный_сигнал)                      → S[k]
//! 3. FFT(ресэмплированный_PRN_код)              → C[k]   (предвычисленный)
//! 4. product[k] = S[k] × conj(C[k])             → частотная корреляция
//! 5. power[n] = |IFFT(product)|²               → корреляционная поверхность
//! 6. peak = max(power)                         → (фаза_кода, мощность)
//! ```
//!
//! Положение пика даёт **фазу кода** (в чипах), а доплеровская
//! проба, давшая максимальный пик, даёт **оценку доплера**.
//!
//! # Использование
//!
//! ```no_run
//! use glrx::acquisition::correlator::{AcquisitionCorrelator, AcquisitionResult};
//! use glrx::signal::prn_code::PrnCodeCache;
//!
//! let cache = PrnCodeCache::new();
//! let mut acq = AcquisitionCorrelator::new(2048, 2_048_000.0);
//!
//! // Предвычисление FFT PRN для всех спутников
//! acq.precompute_all(&cache);
//!
//! // Поиск PRN 1 в диапазоне ±5 кГц доплера с шагом 500 Гц
//! // (здесь сигнал — просто заглушка IQ данных)
//! let signal = vec![num_complex::Complex32::new(0.0, 0.0); 2048];
//! if let Some(result) = acq.search(&signal, 1, -5000.0, 5000.0, 500.0) {
//!     println!("PRN 1: доплер={:.0} Гц, фаза_кода={}", result.doppler_hz, result.code_phase_samples);
//! }
//! ```

use std::collections::HashMap;

use num_complex::Complex32;

use crate::signal::{
    fft::FftEngine,
    mixer::Nco,
    prn_code::{GPS_CODE_LENGTH, PrnCodeCache},
};

/// Результат успешного захвата PRN (поиска спутника).
#[derive(Debug, Clone)]
pub struct AcquisitionResult {
    /// Номер PRN, который был найден (1–32 для GPS)
    pub prn: u8,

    /// Оценка доплеровского сдвига частоты в Гц
    pub doppler_hz: f64,

    /// Фаза кода корреляционного пика в сэмплах (`0..block_size`)
    pub code_phase_samples: usize,

    /// Фаза кода, преобразованная в чипы (0.0..1023.0)
    pub code_phase_chips: f64,

    /// Пиковая мощность корреляции (линейная, не дБ)
    pub peak_power: f32,

    /// Отношение пиковой мощности к средней — чем выше, тем лучше.
    /// Значения > 2.5 обычно указывают на уверенное обнаружение
    pub peak_to_mean_ratio: f32,
}

/// FFT-based PCPS acquisition engine.
///
/// Maintains:
/// - A single [`FftEngine`] of size `block_size` (reused across calls).
/// - Pre-computed FFTs of all PRN codes at the configured sample rate.
pub struct AcquisitionCorrelator {
    /// FFT Engine
    fft: FftEngine,

    /// `block_size` = number of IQ samples per code period (1ms)
    block_size: usize,

    /// Receiver sample rate in Hz
    sample_rate_hz: f64,

    /// Precomputed `FFT(prn_code)` for each PRN
    /// Key: PRN 1-32, Value: complex spectrum of the resampled code.
    prn_ffts: HashMap<u8, Vec<Complex32>>,
}

impl AcquisitionCorrelator {
    /// Crate a new correlator.
    ///
    /// # Arguments
    ///
    /// - `block_size` - number of IQ samples per code period (e.g. 2048 for
    ///   2.048 Msps GPS L1 C/A at 1ms integration).
    /// - `sample_rate_hz` - IQ sample rate in Hz.
    #[must_use]
    pub fn new(
        block_size: usize,
        sample_rate_hz: f64,
    ) -> Self {
        Self {
            fft: FftEngine::new(block_size),
            block_size,
            sample_rate_hz,
            prn_ffts: HashMap::new(),
        }
    }

    /// Precompute FFT of the resampled PRN code for a single satellite.
    ///
    /// Call this once per PRN before starting acquisition searches.
    ///
    /// # Panics
    ///
    /// Panics if `prn` is outside the valid GPS L1 C/A range `1..=32`.
    pub fn precompute_prn(
        &mut self,
        prn: u8,
        cache: &PrnCodeCache,
    ) {
        let resampled = cache
            .resample_gps(prn, self.block_size)
            .expect("PRN must be 1..=32");
        // Convert f32 code to Complex32 (real-valued, imaginary = 0)
        let mut code_complex: Vec<Complex32> = resampled
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        self.fft.fft_inplace(&mut code_complex);
        self.prn_ffts.insert(prn, code_complex);
    }

    /// Precompute FFTs for all GPS PRN 1-32.
    pub fn precompute_all(
        &mut self,
        cache: &PrnCodeCache,
    ) {
        for prn in 1u8..=32 {
            self.precompute_prn(prn, cache);
        }
    }

    /// Compute the FFT-based circular cross-correlation power surface
    /// between `signal` and the pre-computed PRN code.
    ///
    /// Returns `None` if the PRN has not been precomputed.
    ///
    /// The returned `Vec<f32>` has length `block_size`. Index `k` is the
    /// correlation power at code phase offset `k` samples.
    ///
    /// # Panics
    ///
    /// Panics if `signal.len() != block_size`.
    pub fn correlate_power(
        &mut self,
        signal: &[Complex32],
        prn: u8,
    ) -> Option<Vec<f32>> {
        let code_fft = self.prn_ffts.get(&prn)?.clone();

        assert_eq!(signal.len(), self.block_size);

        let mut sig = signal.to_vec();

        self.fft.fft_inplace(&mut sig);

        let mut product: Vec<Complex32> = sig
            .iter()
            .zip(code_fft.iter())
            .map(|(s, c)| s * c.conj())
            .collect();

        self.fft.ifft_inplace(&mut product);

        Some(product.into_iter().map(|s| s.norm_sqr()).collect())
    }

    /// Search for `prn` across a Doppler grid.
    ///
    /// # Arguments
    ///
    /// * `signal` — one code-period of IQ samples (length must match `block_size`).
    /// * `prn` — GPS PRN to search (1–32). Must have been precomputed.
    /// * `doppler_min_hz` — start of Doppler search range in Hz (e.g. −5000).
    /// * `doppler_max_hz` — end of Doppler search range in Hz (e.g. +5000).
    /// * `doppler_step_hz` — frequency resolution of the grid (e.g. 500).
    ///
    /// # Returns
    ///
    /// `None` if the PRN was not precomputed.
    /// `Some(AcquisitionResult)` with the Doppler + code phase of the maximum
    /// peak found across the entire grid.
    ///
    /// # Panics
    ///
    /// Panics if floating-point comparison fails due to NaN values
    /// encountered during peak search (`partial_cmp(...).unwrap()`).
    ///
    /// Panics if internal power computation produces an empty surface
    /// (should not happen in normal operation).
    pub fn search(
        &mut self,
        signal: &[Complex32],
        prn: u8,
        doppler_min_hz: f64,
        doppler_max_hz: f64,
        doppler_step_hz: f64,
    ) -> Option<AcquisitionResult> {
        if !self.prn_ffts.contains_key(&prn) {
            return None;
        }

        let mut best_power = 0.0f32;
        let mut best_doppler = doppler_min_hz;
        let mut best_phase = 0usize;
        let mut best_surface: Vec<f32> = Vec::new();

        let mut f = doppler_min_hz;

        while f <= doppler_max_hz + doppler_step_hz * 0.5 {
            // Carrier wipe-off at trial Doppler
            let wiped = apply_doppler(signal, -f, self.sample_rate_hz);
            // Cross-correlate with precomputed PRN
            let power = self.correlate_power(&wiped, prn)?;
            // Find peak in this Doppler slice
            let (peak_idx, &peak_val) = power
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())?;

            if peak_val > best_power {
                best_power = peak_val;
                best_doppler = f;
                best_phase = peak_idx;
                best_surface = power;
            }

            f += doppler_step_hz;
        }

        let mean_power = best_surface.iter().sum::<f32>() / best_surface.len() as f32;
        let peak_to_mean = if mean_power > 0.0 {
            best_power / mean_power
        } else {
            0.0
        };

        // Convert sample phase to chip phase
        let code_phase_chips = best_phase as f64 * GPS_CODE_LENGTH as f64 / self.block_size as f64;

        Some(AcquisitionResult {
            prn,
            doppler_hz: best_doppler,
            code_phase_samples: best_phase,
            code_phase_chips,
            peak_power: best_power,
            peak_to_mean_ratio: peak_to_mean,
        })
    }

    /// Block size this correlator was built for.
    #[must_use]
    pub const fn block_size(&self) -> usize {
        self.block_size
    }

    /// Sample rate this correlator was built for.
    #[must_use]
    pub const fn sample_rate_hz(&self) -> f64 {
        self.sample_rate_hz
    }

    /// The number of PRNs currently precomputed.
    #[must_use]
    pub fn precomputed_count(&self) -> usize {
        self.prn_ffts.len()
    }
}

/// Apply Doppler frequency shift to a signal block.
fn apply_doppler(
    signal: &[Complex32],
    doppler_hz: f64,
    sample_rate_hz: f64,
) -> Vec<Complex32> {
    let mut nco = Nco::new(doppler_hz, sample_rate_hz);

    signal.iter().map(|&s| s * nco.advance()).collect()
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 2_048_000.0;
    const N: usize = 2048;

    fn make_correlator() -> AcquisitionCorrelator {
        AcquisitionCorrelator::new(N, FS)
    }

    #[test]
    fn test_precompute_single_prn() {
        let cache = PrnCodeCache::new();
        let mut acq = make_correlator();

        acq.precompute_prn(1, &cache);

        assert_eq!(acq.precomputed_count(), 1);
    }

    #[test]
    fn test_precompute_all_32_prns() {
        let cache = PrnCodeCache::new();
        let mut acq = make_correlator();

        acq.precompute_all(&cache);

        assert_eq!(acq.precomputed_count(), 32);
    }

    #[test]
    fn test_correlate_power_returns_correct_length() {
        let cache = PrnCodeCache::new();
        let mut acq = make_correlator();

        acq.precompute_prn(1, &cache);

        let signal = vec![Complex32::new(0.0, 0.0); N];
        let power = acq.correlate_power(&signal, 1).unwrap();

        assert_eq!(power.len(), N);
    }

    #[test]
    fn test_correlate_power_zero_signal_produces_zeros() {
        let cache = PrnCodeCache::new();
        let mut acq = make_correlator();

        acq.precompute_prn(1, &cache);

        let signal = vec![Complex32::new(0.0, 0.0); N];
        let power = acq.correlate_power(&signal, 1).unwrap();

        for &p in &power {
            assert!(p.abs() < 1e-6, "expected zero power, got {p}");
        }
    }

    #[test]
    fn correlate_power_unprecomputed_prn_returns_none() {
        let mut acq = make_correlator();
        let signal = vec![Complex32::new(1.0, 0.0); N];

        assert!(acq.correlate_power(&signal, 5).is_none());
    }

    #[test]
    fn correlate_power_peak_at_zero_lag_for_aligned_signal() {
        // If signal = resampled PRN code (no Doppler, no delay),
        // the correlation peak should be at lag 0.
        let cache = PrnCodeCache::new();
        let mut acq = make_correlator();

        acq.precompute_prn(1, &cache);

        // Signal IS the code (perfect alignment, no Doppler)
        let signal: Vec<Complex32> = cache
            .resample_gps(1, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        let power = acq.correlate_power(&signal, 1).unwrap();
        let peak_idx = power
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)
            .unwrap();

        assert_eq!(peak_idx, 0, "expected peak at lag 0, got {peak_idx}");
    }

    #[test]
    fn correlate_power_peak_at_known_delay() {
        // Delay signal by D samples → correlation peak should be at lag D.
        let cache = PrnCodeCache::new();
        let mut acq = make_correlator();

        acq.precompute_prn(3, &cache);

        let delay = 42usize;
        let base: Vec<Complex32> = cache
            .resample_gps(3, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        // Circularly shift the signal by `delay` samples
        let mut delayed = vec![Complex32::default(); N];

        for i in 0..N {
            delayed[(i + delay) % N] = base[i];
        }

        let power = acq.correlate_power(&delayed, 3).unwrap();
        let peak_idx = power
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.total_cmp(b))
            .map(|(i, _)| i)
            .unwrap();

        assert_eq!(peak_idx, delay, "expected peak at {delay}, got {peak_idx}");
    }

    #[test]
    fn correlate_power_different_prns_low_cross_peak() {
        // Correlation of PRN 1 signal against PRN 2 code should be low
        let cache = PrnCodeCache::new();
        let mut acq = make_correlator();

        acq.precompute_prn(1, &cache);
        acq.precompute_prn(2, &cache);

        // Strong PRN 1 signal
        let signal: Vec<Complex32> = cache
                .resample_gps(1, N)
                .unwrap()
                .into_iter()
                .map(|c| Complex32::new(c * 100.0, 0.0)) // high amplitude
                .collect();

        let power_matched = acq.correlate_power(&signal, 1).unwrap();
        let power_wrong = acq.correlate_power(&signal, 2).unwrap();

        let peak_matched = power_matched.iter().copied().fold(0.0f32, f32::max);
        let peak_wrong = power_wrong.iter().copied().fold(0.0f32, f32::max);

        assert!(
            peak_matched > peak_wrong * 10.0,
            "matched peak ({peak_matched}) should be >> wrong PRN peak ({peak_wrong})",
        );
    }

    #[test]
    fn search_returns_none_without_precompute() {
        let mut acq = make_correlator();
        let signal = vec![Complex32::new(0.0, 0.0); N];

        assert!(acq.search(&signal, 7, -1000.0, 1000.0, 500.0).is_none());
    }

    #[test]
    fn search_finds_zero_doppler_aligned_signal() {
        let cache = PrnCodeCache::new();
        let mut acq = make_correlator();

        acq.precompute_prn(5, &cache);

        // Perfect signal: PRN 5, no Doppler, no delay
        let signal: Vec<Complex32> = cache
            .resample_gps(5, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        let result = acq
            .search(&signal, 5, -1000.0, 1000.0, 500.0)
            .expect("search should succeed");

        // Doppler should be near 0 Hz
        assert!(
            result.doppler_hz.abs() <= 500.0,
            "expected doppler ≈ 0, got {}",
            result.doppler_hz
        );
        // Code phase should be near 0 samples
        assert!(
            result.code_phase_samples <= 2,
            "expected code_phase ≈ 0, got {}",
            result.code_phase_samples
        );
        // Peak-to-mean ratio should be high for a clean signal
        assert!(
            result.peak_to_mean_ratio > 2.0,
            "expected high ratio, got {}",
            result.peak_to_mean_ratio
        );
    }

    #[test]
    fn search_finds_known_doppler() {
        // Inject 1000 Hz Doppler → search should find it
        let cache = PrnCodeCache::new();
        let mut acq = make_correlator();

        acq.precompute_prn(2, &cache);

        let true_doppler = 1000.0_f64;
        let base: Vec<Complex32> = cache
            .resample_gps(2, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        // Apply +1000 Hz Doppler to simulate received signal
        let signal = apply_doppler(&base, true_doppler, FS);

        let result = acq
            .search(&signal, 2, -2000.0, 2000.0, 500.0)
            .expect("search should find signal");

        // Best Doppler trial should be nearest to 1000 Hz (within one step)
        assert!(
            (result.doppler_hz - true_doppler).abs() <= 500.0,
            "expected doppler ≈ 1000 Hz, got {} Hz",
            result.doppler_hz
        );
    }

    #[test]
    fn search_code_phase_to_chips_in_range() {
        let cache = PrnCodeCache::new();
        let mut acq = make_correlator();

        acq.precompute_prn(1, &cache);

        let signal = vec![Complex32::new(0.01, 0.0); N]; // weak noise

        if let Some(result) = acq.search(&signal, 1, 0.0, 0.0, 500.0) {
            assert!(
                result.code_phase_chips < GPS_CODE_LENGTH as f64,
                "chip phase out of range: {}",
                result.code_phase_chips
            );
        }
    }

    #[test]
    fn apply_doppler_zero_is_identity() {
        let signal: Vec<Complex32> = (0..64)
            .map(|i| Complex32::new((i as f32).cos(), (i as f32).sin()))
            .collect();
        let result = apply_doppler(&signal, 0.0, FS);

        for (a, b) in signal.iter().zip(result.iter()) {
            assert!((a.re - b.re).abs() < 1e-5);
            assert!((a.im - b.im).abs() < 1e-5);
        }
    }

    #[test]
    fn apply_doppler_preserves_amplitude() {
        let signal: Vec<Complex32> = (0..64)
            .map(|i| Complex32::new((i as f32 * 0.1).cos(), 0.0))
            .collect();
        let result = apply_doppler(&signal, 10_000.0, FS);

        for (a, b) in signal.iter().zip(result.iter()) {
            let mag_a = (a.re * a.re + a.im * a.im).sqrt();
            let mag_b = (b.re * b.re + b.im * b.im).sqrt();

            assert!(
                (mag_a - mag_b).abs() < 1e-5,
                "amplitude changed: {mag_a} vs {mag_b}",
            );
        }
    }

    #[test]
    fn block_size_and_sample_rate_accessible() {
        let acq = AcquisitionCorrelator::new(4096, 4_096_000.0);
        let expected = 4_096_000.0;

        assert_eq!(acq.block_size(), 4096);
        assert!((acq.sample_rate_hz() - expected).abs() < 1e-9);
    }
}
