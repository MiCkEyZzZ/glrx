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

use crate::signal::{fft::FftEngine, mixer::Nco, prn_code::PrnCodeCache};

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
    pub prm: u8,

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
    /// Returns (doppler_idx, code_phase_sampler, peak_power).
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
    pub fn with_default(
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
        PcpsSearch::with_default(N, FS)
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
}
