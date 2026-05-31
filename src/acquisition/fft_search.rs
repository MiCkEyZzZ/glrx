//! FFT-based GNSS acquisition: Parallel Code Search (PCPS).
//!
//! # Algorithm
//!
//! For each Doppler trial frequency `f_d` in the search grid:
//!
//! ```text
//! 1. wiped[n] = signal[n] × exp(−j·2π·f_d·n/fs)   carrier wipe-off
//! 2. S[k]     = FFT(wiped)
//! 3. C[k]     = FFT(prn_code)                       pre-computed
//! 4. R[k]     = S[k] × conj(C[k])
//! 5. power[n] = |IFFT(R)|²                          correlation surface
//!  6. peak     = argmax(power)  →  code_phase
//! ```
//!
//! The 2-D surface (Doppler * code_phase) is scanned for each PRN.
//! A detection is declared when `peak_power / noise+floor > cfar_threshold`.

use std::collections::HashMap;

use num_complex::Complex32;

use crate::signal::{
    fft::FftEngine,
    mixer::Nco,
    prn_code::{PrnCodeCache, GPS_CODE_LENGTH},
};

/// Configuration for the acquisition frequency/Doppler search grid.
#[derive(Debug, Clone)]
pub struct SearchConfig {
    /// Minimum Doppler to search in Hz (e.g. -10_000)
    pub doppler_min_hz: f64,

    /// Maximum Doppler to search in Hz (e.g. +10_000)
    pub doppler_max_hz: f64,

    /// Step between Doppler trials in Hz (e.g. 500)
    pub doppler_step_hz: f64,

    /// CFAR detection threshold: peak/noise_floor ratio.
    /// Typical values: 2.5 (loose)..4.0 (strict).
    pub cfar_threshold: f32,
}

/// Coarse PCPS acquisition result for one PRN.
#[derive(Debug, Clone)]
pub struct SearchResult {
    /// PRN number (GPS: 1-32)
    pub prn: u8,

    /// Coarse Doppler frequency from the grid in Hz
    pub doppler_coarse_hz: f64,

    /// Fine Doppler estimate (after sub-bin interpolation) in Hz.
    pub doppler_fine_hz: f64,

    /// Code phase in samples (0..block_size).
    pub code_phase_samples: usize,

    /// Code phase in chips (0.0..1023.0).
    pub code_phase_chips: f64,

    /// Peak correlation power (linear).
    pub peak_power: f32,

    /// Estimated noise floor power (mean of the surface).
    pub noise_floor: f32,

    /// Peak-to-noise-floor ratio. Used for CFAR detection.
    pub peak_to_noise: f32,

    /// Whether this result exceeds the CFAR detection threshold.
    pub detected: bool,
}

/// 2-D search surface (internal)
struct SearchSurface {
    /// Row = Doppler bin index, Col = code-phase sample
    data: Vec<Vec<f32>>,
    doppler_trials: Vec<f64>,
}

/// Full PCPS acquisition engine.
///
/// Holds pre-computed PRN FFTs and an [`FftEngine`] reused across all
/// Doppler trials. Call [`PcpsSearch::precompute_all`] once, then
/// [`PcpsSearch::search_prn`] or [`PcpsSearch::search_all`] per acquisition
/// attempt.
pub struct PcpsSearch {
    fft: FftEngine,
    block_size: usize,
    sample_rate_hz: f64,
    prn_ffts: HashMap<u8, Vec<Complex32>>,
    config: SearchConfig,
}

impl SearchConfig {
    /// The Number of Doppler trials in this configuration.
    pub fn num_doppler_bins(&self) -> usize {
        let span = self.doppler_max_hz - self.doppler_min_hz;

        (span / self.doppler_step_hz).round() as usize + 1
    }

    /// Iterator over all Doppler trial frequencies.
    pub fn doppler_trials(&self) -> impl Iterator<Item = f64> {
        let min = self.doppler_min_hz;
        let step = self.doppler_step_hz;
        let n = self.num_doppler_bins();

        (0..n).map(move |i| min + i as f64 * step)
    }
}

impl SearchSurface {
    fn new(
        doppler_trials: Vec<f64>,
        block_size: usize,
    ) -> Self {
        let data = vec![vec![0.0f32; block_size]; doppler_trials.len()];

        SearchSurface {
            data,
            doppler_trials,
        }
    }

    fn set_row(
        &mut self,
        doppler_idx: usize,
        power: Vec<f32>,
    ) {
        self.data[doppler_idx] = power;
    }

    /// Find the global maximum over the entire 2-D surface.
    /// Returns (doppler_idx, code_phase_samples, peak_power).
    fn global_peak(&self) -> (usize, usize, f32) {
        let mut best_d = 0usize;
        let mut best_c = 0usize;
        let mut best_p = 0.0f32;

        for (d, row) in self.data.iter().enumerate() {
            for (c, &p) in row.iter().enumerate() {
                if p > best_p {
                    best_p = p;
                    best_d = d;
                    best_c = c
                }
            }
        }

        (best_d, best_c, best_p)
    }

    /// Mean power across the entire surface (noise floor estimate).
    fn mean_power(&self) -> f32 {
        let total: f32 = self.data.iter().flatten().sum();
        let count = self.data.len() * self.data[0].len();

        if count == 0 {
            0.0
        } else {
            total / count as f32
        }
    }
}

impl PcpsSearch {
    /// Create a new PCPS engine.
    ///
    /// # Arguments
    ///
    /// - `block_size` — samples per code period (e.g. 2048 at 2.048 Msps).
    /// - `sample_rate_hz` — IQ sample rate in Hz.
    /// - `config` — Doppler grid and CFAR configuration.
    pub fn new(
        block_size: usize,
        sample_rate_hz: f64,
        config: SearchConfig,
    ) -> Self {
        Self {
            fft: FftEngine::new(block_size),
            block_size,
            sample_rate_hz,
            prn_ffts: HashMap::new(),
            config,
        }
    }

    /// Create engine default search config (±10 kHz, 500 Hz step, CFAR=3).
    pub fn with_defaults(
        block_size: usize,
        sample_rate_hz: f64,
    ) -> Self {
        Self::new(block_size, sample_rate_hz, SearchConfig::default())
    }

    /// Pre-compute the FFT of the resampled code for a signle PRN.
    pub fn precompute_prn(
        &mut self,
        prn: u8,
        cache: &PrnCodeCache,
    ) {
        let resampled = match cache.resample_gps(prn, self.block_size) {
            Some(v) => v,
            None => return,
        };
        let mut code: Vec<Complex32> = resampled
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        self.fft.fft_inplace(&mut code);
        self.prn_ffts.insert(prn, code);
    }

    /// Pre-compute FFTs for all GPS PRN 1-32.
    pub fn precompute_all(
        &mut self,
        cache: &PrnCodeCache,
    ) {
        for prn in 1u8..=32 {
            self.precompute_prn(prn, cache);
        }
    }

    /// Run PCPS acquisition for a single PRN.
    ///
    /// Returns `None` if the PRN has not been pre-computed.
    pub fn search_prn(
        &mut self,
        signal: &[Complex32],
        prn: u8,
    ) -> Option<SearchResult> {
        assert_eq!(signal.len(), self.block_size);

        if !self.prn_ffts.contains_key(&prn) {
            return None;
        }

        let trials: Vec<f64> = self.config.doppler_trials().collect();
        let mut surface = SearchSurface::new(trials.clone(), self.block_size);

        for (d_idx, &f_d) in trials.iter().enumerate() {
            let wiped = apply_doppler_shift(signal, -f_d, self.sample_rate_hz);
            let power = self.correlate_with_prn(&wiped, prn)?;
            surface.set_row(d_idx, power);
        }

        let (best_d, best_c, peak_power) = surface.global_peak();
        let noise_floor = surface.mean_power();
        let peak_to_noise = if noise_floor > 0.0 {
            peak_power / noise_floor
        } else {
            0.0
        };
        let detected = peak_to_noise >= self.config.cfar_threshold;
        let doppler_coarse = trials[best_d];
        let doppler_fine = self.fine_doppler_estimate(&surface, best_d, best_c);
        let code_phase_chips = best_c as f64 * GPS_CODE_LENGTH as f64 / self.block_size as f64;

        Some(SearchResult {
            prn,
            doppler_coarse_hz: doppler_coarse,
            doppler_fine_hz: doppler_fine,
            code_phase_samples: best_c,
            code_phase_chips,
            peak_power,
            noise_floor,
            peak_to_noise,
            detected,
        })
    }

    /// Search all 32 GPS PRNs and return only detected satellites.
    ///
    /// Results are sorted by `peak_to_noise` descending (strongest first).
    pub fn search_all(
        &mut self,
        signal: &[Complex32],
    ) -> Vec<SearchResult> {
        let prns: Vec<u8> = self.prn_ffts.keys().cloned().collect();
        let mut detected = Vec::new();
        for prn in prns {
            if let Some(result) = self.search_prn(signal, prn) {
                if result.detected {
                    detected.push(result);
                }
            }
        }
        detected.sort_by(|a, b| b.peak_to_noise.partial_cmp(&a.peak_to_noise).unwrap());
        detected
    }

    /// Sample rate this engine was built for.
    pub fn sample_rate_hz(&self) -> f64 {
        self.sample_rate_hz
    }

    /// Block size (samples per code period).
    pub fn block_size(&self) -> usize {
        self.block_size
    }

    /// The Number of per-computed PRNs.
    pub fn precomputed_count(&self) -> usize {
        self.prn_ffts.len()
    }

    /// Current search configuration.
    pub fn config(&self) -> &SearchConfig {
        &self.config
    }

    /// FFT-based circular correlation of `signal` against pre-computed PRN.
    fn correlate_with_prn(
        &mut self,
        signal: &[Complex32],
        prn: u8,
    ) -> Option<Vec<f32>> {
        let code_fft = self.prn_ffts.get(&prn)?.to_vec();
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

    /// Parabolic interpolation in the Doppler axis to get sub-bin frequency estimate.
    ///
    /// Uses the peak value and its two neighbours in the Doppler dimension.
    /// Returns the coarse Doppler if the peak is at the boundary.
    fn fine_doppler_estimate(
        &self,
        surface: &SearchSurface,
        best_d: usize,
        best_c: usize,
    ) -> f64 {
        let n = surface.data.len();

        if best_d == 0 || best_d >= n - 1 {
            return surface.doppler_trials[best_d];
        }

        let y_m1 = surface.data[best_d - 1][best_c];
        let y_0 = surface.data[best_d][best_c];
        let y_p1 = surface.data[best_d + 1][best_c];
        let denom = 2.0 * y_0 - y_m1 - y_p1;

        if denom.abs() < f32::EPSILON {
            return surface.doppler_trials[best_d];
        }

        let delta = (y_p1 - y_m1) / (2.0 * denom);

        surface.doppler_trials[best_d] + delta as f64 * self.config.doppler_step_hz
    }
}

/// Apply a frequency shift of `doppler_hz` to `signal`.
pub(crate) fn apply_doppler_shift(
    signal: &[Complex32],
    doppler_hz: f64,
    sample_rate_hz: f64,
) -> Vec<Complex32> {
    let mut nco = Nco::new(doppler_hz, sample_rate_hz);

    signal.iter().map(|&s| s * nco.advance()).collect()
}

impl Default for SearchConfig {
    /// GPS L1 C/A default: ±10 kHz, 500 Hz step, CFAR = 3.0.
    fn default() -> Self {
        SearchConfig {
            doppler_min_hz: -10_000.0,
            doppler_max_hz: 10_000.0,
            doppler_step_hz: 500.0,
            cfar_threshold: 3.0,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 2_048_000.0;
    const N: usize = 2048;

    fn make_engine() -> PcpsSearch {
        PcpsSearch::with_defaults(N, FS)
    }

    #[test]
    fn test_search_config_default_num_bins() {
        let cfg = SearchConfig::default();

        // (-10000 .. +10000) / 500 + 1 = 41
        assert_eq!(cfg.num_doppler_bins(), 41);
    }

    #[test]
    fn test_search_config_doppler_trials_range() {
        let cfg = SearchConfig::default();
        let trials: Vec<f64> = cfg.doppler_trials().collect();

        assert!((trials.first().unwrap() - cfg.doppler_min_hz).abs() < 1.0);
        assert!((trials.last().unwrap() - cfg.doppler_max_hz).abs() < 1.0);
    }

    #[test]
    fn test_search_config_custom() {
        let cfg = SearchConfig {
            doppler_min_hz: -1000.0,
            doppler_max_hz: 1000.0,
            doppler_step_hz: 500.0,
            cfar_threshold: 2.5,
        };

        assert_eq!(cfg.num_doppler_bins(), 5);
    }

    #[test]
    fn test_precompute_single_prn() {
        let cache = PrnCodeCache::new();
        let mut eng = make_engine();

        eng.precompute_prn(1, &cache);

        assert_eq!(eng.precomputed_count(), 1);
    }

    #[test]
    fn test_precompute_all_gps() {
        let cache = PrnCodeCache::new();
        let mut eng = make_engine();

        eng.precompute_all(&cache);

        assert_eq!(eng.precomputed_count(), 32);
    }

    #[test]
    fn test_search_prn_unprecomputed_returns_none() {
        let mut eng = make_engine();
        let signal = vec![Complex32::new(0.0, 0.0); N];

        assert!(eng.search_prn(&signal, 7).is_none());
    }

    #[test]
    fn test_search_prn_finds_aligned_signal_no_doppler() {
        let cache = PrnCodeCache::new();
        let mut eng = PcpsSearch::new(
            N,
            FS,
            SearchConfig {
                doppler_min_hz: -1000.0,
                doppler_max_hz: 1000.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 2.0,
            },
        );

        eng.precompute_prn(1, &cache);

        let signal: Vec<Complex32> = cache
            .resample_gps(1, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        let res = eng.search_prn(&signal, 1).unwrap();

        assert!(res.detected, "should detect clean aligned signal");
        assert!(
            res.doppler_coarse_hz.abs() <= 500.0,
            "doppler = {}",
            res.doppler_coarse_hz
        );
        assert!(
            res.code_phase_samples <= 2,
            "code_phase = {}",
            res.code_phase_samples
        );
        assert!(
            res.peak_to_noise > 2.0,
            "peak_to_noise = {}",
            res.peak_to_noise
        );
    }

    #[test]
    fn test_search_prn_finds_known_code_phase() {
        let cache = PrnCodeCache::new();
        let mut eng = PcpsSearch::new(
            N,
            FS,
            SearchConfig {
                doppler_min_hz: 0.0,
                doppler_max_hz: 0.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 2.0,
            },
        );

        eng.precompute_prn(2, &cache);

        let delay = 100usize;
        let base: Vec<Complex32> = cache
            .resample_gps(2, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();
        let mut delayed = vec![Complex32::default(); N];

        for i in 0..N {
            delayed[(i + delay) % N] = base[i];
        }

        let res = eng.search_prn(&delayed, 2).unwrap();

        assert_eq!(
            res.code_phase_samples, delay,
            "expected code phase {}, got {}",
            delay, res.code_phase_samples
        );
    }

    #[test]
    fn test_search_prn_finds_known_doppler() {
        let cache = PrnCodeCache::new();
        let mut eng = PcpsSearch::new(
            N,
            FS,
            SearchConfig {
                doppler_min_hz: -2000.0,
                doppler_max_hz: 2000.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 2.0,
            },
        );

        eng.precompute_prn(3, &cache);

        let true_doppler = 1000.0_f64;
        let base: Vec<Complex32> = cache
            .resample_gps(3, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();
        let signal = apply_doppler_shift(&base, true_doppler, FS);

        let res = eng.search_prn(&signal, 3).unwrap();

        assert!(
            (res.doppler_coarse_hz - true_doppler).abs() <= 500.0,
            "expected ≈ {} Hz, got {} Hz",
            true_doppler,
            res.doppler_coarse_hz
        );
    }

    #[test]
    fn test_search_prn_zero_signal_not_detected() {
        let cache = PrnCodeCache::new();
        let mut eng = PcpsSearch::new(
            N,
            FS,
            SearchConfig {
                doppler_min_hz: 0.0,
                doppler_max_hz: 0.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 3.0,
            },
        );

        eng.precompute_prn(1, &cache);

        let signal = vec![Complex32::new(0.0, 0.0); N];
        let res = eng.search_prn(&signal, 1).unwrap();

        assert!(!res.detected, "zero signal should not be detected");
    }

    #[test]
    fn test_cfar_threshold_respected() {
        let cache = PrnCodeCache::new();
        // Use a very high threshold so even a real signal is "not detected"
        let mut eng_strict = PcpsSearch::new(
            N,
            FS,
            SearchConfig {
                doppler_min_hz: 0.0,
                doppler_max_hz: 0.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 1000.0,
            },
        );

        eng_strict.precompute_prn(1, &cache);

        let signal: Vec<Complex32> = cache
            .resample_gps(1, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();
        let res = eng_strict.search_prn(&signal, 1).unwrap();

        assert!(!res.detected, "ultra-strict threshold should not detect");
    }

    #[test]
    fn test_fine_doppler_within_step_of_coarse() {
        let cache = PrnCodeCache::new();
        let mut eng = PcpsSearch::new(
            N,
            FS,
            SearchConfig {
                doppler_min_hz: -2000.0,
                doppler_max_hz: 2000.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 2.0,
            },
        );

        eng.precompute_prn(4, &cache);

        let signal: Vec<Complex32> = cache
            .resample_gps(4, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();
        let res = eng.search_prn(&signal, 4).unwrap();

        // Fine estimate should be within one step of coarse
        assert!(
            (res.doppler_fine_hz - res.doppler_coarse_hz).abs() <= 500.0,
            "fine={} coarse={}",
            res.doppler_fine_hz,
            res.doppler_coarse_hz
        );
    }

    #[test]
    fn test_search_all_no_signal_returns_empty() {
        let cache = PrnCodeCache::new();
        let mut eng = PcpsSearch::new(
            N,
            FS,
            SearchConfig {
                doppler_min_hz: 0.0,
                doppler_max_hz: 0.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 3.0,
            },
        );

        eng.precompute_all(&cache);

        let signal = vec![Complex32::new(0.0, 0.0); N];
        let results = eng.search_all(&signal);

        assert!(results.is_empty(), "noise should not produce detections");
    }

    #[test]
    fn test_search_all_detects_injected_prn() {
        let cache = PrnCodeCache::new();
        let mut eng = PcpsSearch::new(
            N,
            FS,
            SearchConfig {
                doppler_min_hz: -500.0,
                doppler_max_hz: 500.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 2.0,
            },
        );

        eng.precompute_all(&cache);

        // Inject PRN 7 at zero Doppler
        let signal: Vec<Complex32> = cache
            .resample_gps(7, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        let results = eng.search_all(&signal);

        assert!(!results.is_empty(), "should detect at least one satellite");
        assert_eq!(results[0].prn, 7, "strongest detection should be PRN 7");
    }

    #[test]
    fn tst_search_all_sorted_by_peak_to_noise() {
        let cache = PrnCodeCache::new();
        let mut eng = PcpsSearch::new(
            N,
            FS,
            SearchConfig {
                doppler_min_hz: 0.0,
                doppler_max_hz: 0.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 2.0,
            },
        );

        eng.precompute_all(&cache);

        let signal: Vec<Complex32> = cache
            .resample_gps(5, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        let results = eng.search_all(&signal);

        for i in 1..results.len() {
            assert!(
                results[i - 1].peak_to_noise >= results[i].peak_to_noise,
                "results not sorted at positions {} and {}",
                i - 1,
                i
            );
        }
    }

    #[test]
    fn test_code_phase_chips_in_valid_range() {
        let cache = PrnCodeCache::new();
        let mut eng = PcpsSearch::new(
            N,
            FS,
            SearchConfig {
                doppler_min_hz: 0.0,
                doppler_max_hz: 0.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 0.0,
            },
        );

        eng.precompute_prn(1, &cache);

        let signal = vec![Complex32::new(0.01, 0.0); N];

        if let Some(res) = eng.search_prn(&signal, 1) {
            assert!(
                res.code_phase_chips < GPS_CODE_LENGTH as f64,
                "chip phase out of range: {}",
                res.code_phase_chips
            );
        }
    }

    #[test]
    fn test_apply_doppler_zero_is_identity() {
        let signal: Vec<Complex32> = (0..64)
            .map(|i| Complex32::new((i as f32).cos(), (i as f32).sin()))
            .collect();
        let result = apply_doppler_shift(&signal, 0.0, FS);

        for (a, b) in signal.iter().zip(result.iter()) {
            assert!((a.re - b.re).abs() < 1e-5);
            assert!((a.im - b.im).abs() < 1e-5);
        }
    }

    #[test]
    fn test_apply_doppler_preserves_amplitude() {
        let signal: Vec<Complex32> = (0..64)
            .map(|i| Complex32::new((i as f32 * 0.1).cos(), 0.0))
            .collect();
        let result = apply_doppler_shift(&signal, 5000.0, FS);

        for (a, b) in signal.iter().zip(result.iter()) {
            let ma = (a.re * a.re + a.im * a.im).sqrt();
            let mb = (b.re * b.re + b.im * b.im).sqrt();

            assert!((ma - mb).abs() < 1e-5);
        }
    }

    #[test]
    fn accessors_return_configured_values() {
        let eng = PcpsSearch::with_defaults(4096, 4_096_000.0);

        assert_eq!(eng.block_size(), 4096);
        assert_eq!(eng.sample_rate_hz(), 4_096_000.0);
        assert_eq!(eng.precomputed_count(), 0);
    }
}
