//! SDR (Software-Defined Radio) IQ source.
//!
//! This module provides two imple,entations:
//!
//! * [`MockSdrSource`] — always available, generates a configurable test tone.
//!   Use this in tests and CI where no hardware is present.
//! * `SoapySource` — hardware SDR via `SoapySDR` (RTL-SDR, `HackRF`, USRP, …).
//!   Compiled only when the `sdr` feature is enabled **and** the system
//!   `SoapySDR` library is installed.
//!
//! # Enabling hardware SDR
//!
//! ```toml
//! [dependencies]
//! glrx = { features = ["sdr"] }
//! ```
//!
//! Then install `SoapySDR` + your device driver:
//!
//! ```sh
//! # Debian / Ubuntu
//! sudo apt install libsoapysdr-dev soapysdr-module-rtlsdr
//! ```
//!
//! # Architecture note
//!
//! Real-time SDR capture runs in a dedicated OS thread that fills a ring
//! buffer (see [`stream`](super::stream)). `read_block` on [`SoapySource`]
//! is a non-blocking drain of that buffer.

use std::f32::consts::TAU;

use num_complex::Complex32;

use super::{IqBlock, IqSource, RfConfig, RfResult, SourceMetrics};

/// A sunthetic IQ source that generates a single-tone complex sinusoid.
///
/// Produces `exp(j·2π·tone_hz·t)` at the configured sample rate with
/// optional additive white Gaussian noise.
///
/// # Example
///
/// ```
/// use glrx::rf::{sdr::MockSdrSource, IqSource, RfConfig};
///
/// // 10 kHz tone (simulates a ±10 kHz Doppler offset)
/// let mut src = MockSdrSource::new(RfConfig::default(), 10_000.0, 0.0);
/// let block = src.read_block(2048).unwrap();
/// assert_eq!(block.samples.len(), 2048);
/// ```
pub struct MockSdrSource {
    config: RfConfig,

    /// Tone frequency relative to centre frequency in Hz.
    tone_hz: f64,

    /// RMS noise amplitude (0.0 = noiseless).
    noise_amplitude: f32,

    /// Current phase accumulator in radians.
    phase: f32,

    /// Phase increment per sample.
    phase_step: f32,
    next_sample: u64,
    metrics: SourceMetrics,
}

/// Hardware SDR source via the `SoapySDR` abstraction layer.
///
/// Supports any device with a `SoapySDR` driver: RTL-SDR, `HackRF`, USRP,
/// `LimeSDR`, `PlutoSDR`, etc.
///
/// # Compile-time requirement
///
/// Only available when the `sdr` Cargo feature is enabled and the
/// `SoapySDR` C++ library is installed on the build host.
///
/// # Thread model
///
/// A background thread drives the `SoapySDR` streaming API and writes
/// samples into a ring buffer.  `read_block` drains from that buffer,
/// blocking for at most `timeout` if insufficient data is available.
///
/// # Usage
///
/// ```no_run
/// # #[cfg(feature = "sdr")]
/// # {
/// use glrx::rf::{sdr::SoapySource, RfConfig};
///
/// let config = RfConfig::default(); // GPS L1, 2.048 Msps
/// let src = SoapySource::open("driver=rtlsdr", config).unwrap();
/// # }
/// ```
#[cfg(feature = "sdr")]
pub struct SoapySource {
    #[allow(dead_code)]
    config: RfConfig,
    #[allow(dead_code)]
    driver_args: String,
    #[allow(dead_code)]
    metrics: SourceMetrics,
    // When SoapySDR bindings are added, the actual device handle goes here.
    // _device: soapysdr::Device,
    // _stream: soapysdr::RxStream<Complex32>,
}

impl MockSdrSource {
    /// Create a new mock source.
    ///
    /// * `tone_hz` — frequency of the complex sinusoid in Hz (relative to
    ///   centre).
    /// * `noise_amplitude` — RMS noise amplitude added to each sample (0 =
    ///   none).
    #[must_use]
    pub fn new(
        config: RfConfig,
        tone_hz: f64,
        noise_amplitude: f32,
    ) -> Self {
        let phase_step = (f64::from(TAU) * tone_hz / config.sample_rate_hz) as f32;
        Self {
            config,
            tone_hz,
            noise_amplitude,
            phase: 0.0,
            phase_step,
            next_sample: 0,
            metrics: SourceMetrics::default(),
        }
    }

    /// Tone frequency this source was configured with.
    #[must_use]
    pub const fn tone_hz(&self) -> f64 {
        self.tone_hz
    }
}

impl IqSource for MockSdrSource {
    fn config(&self) -> &RfConfig {
        &self.config
    }

    fn name(&self) -> &'static str {
        "mock_sdr"
    }

    fn read_block(
        &mut self,
        n: usize,
    ) -> RfResult<IqBlock> {
        let start_sample = self.next_sample;
        let mut samples = Vec::with_capacity(n);

        for _ in 0..n {
            let (sin, cos) = self.phase.sin_cos();
            let mut s = Complex32::new(cos, sin);

            // Simple deterministic pseudo-noise (xorshift on sample index)
            if self.noise_amplitude > 0.0 {
                let idx = self.next_sample + samples.len() as u64;
                let noise = pseudo_noise(idx) * self.noise_amplitude;
                s += Complex32::new(
                    noise,
                    pseudo_noise(idx ^ 0xDEAD_BEEF) * self.noise_amplitude,
                );
            }

            samples.push(s);
            self.phase = (self.phase + self.phase_step) % TAU;
        }

        let len = samples.len() as u64;
        self.next_sample += len;
        self.metrics.total_samples += len;
        self.metrics.measured_rate_hz = Some(self.config.sample_rate_hz);

        Ok(IqBlock {
            samples,
            config: self.config.clone(),
            start_sample,
        })
    }

    fn metrics(&self) -> SourceMetrics {
        self.metrics.clone()
    }
}

#[cfg(feature = "sdr")]
impl SoapySource {
    /// Open a `SoapySDR` device.
    ///
    /// * `driver_args` — `SoapySDR` device filter string, e.g. `"driver=rtlsdr"`,
    ///   `"driver=hackrf"`, or `""` to use the first available device.
    ///
    /// # Errors
    ///
    /// Returns:
    /// - `RfError::Config` if the RF configuration is invalid
    /// - `RfError::Sdr` if the SDR backend fails to initialize
    /// - Underlying I/O or driver-specific errors if device creation fails
    pub fn open(
        driver_args: &str,
        config: RfConfig,
    ) -> RfResult<Self> {
        config.validate()?;

        // Future implementation outline
        //
        // use soapysdr::{Device, Direction::Rx};
        //
        // let device = Device::new(driver_args)
        //     .map_err(|e| RfError::Sdr(e.to_string()))?;
        //
        // device.set_sample_rate(Rx, 0, config.sample_rate_hz)
        //     .map_err(|e| RfError::Sdr(e.to_string()))?;
        //
        // device.set_frequency(Rx, 0, config.center_freq_hz, ())
        //     .map_err(|e| RfError::Sdr(e.to_string()))?;
        //
        // if let Some(gain) = config.gain_db {
        //     device.set_gain(Rx, 0, gain)
        //         .map_err(|e| RfError::Sdr(e.to_string()))?;
        // } else {
        //     device.set_gain_mode(Rx, 0, true)   // AGC
        //         .map_err(|e| RfError::Sdr(e.to_string()))?;
        // }
        //
        // let stream = device.rx_stream::<Complex32>(&[0])
        //     .map_err(|e| RfError::Sdr(e.to_string()))?;
        //

        log::info!(
            "SoapySource: opening device '{}' @ {:.3} MHz, {:.3} Msps",
            driver_args,
            config.center_freq_hz / 1e6,
            config.sample_rate_hz / 1e6,
        );

        Ok(Self {
            config,
            driver_args: driver_args.to_owned(),
            metrics: SourceMetrics::default(),
        })
    }

    /// List all `SoapySDR` devices currently visible on the system.
    #[must_use]
    pub fn enumerate() -> Vec<String> {
        // Future: soapysdr::enumerate("").map(|kw| kw.to_string()).collect()
        vec!["<SoapySDR enumeration not yet implemented>".to_string()]
    }
}

#[cfg(feature = "sdr")]
impl IqSource for SoapySource {
    fn config(&self) -> &RfConfig {
        &self.config
    }

    fn name(&self) -> &str {
        &self.driver_args
    }

    fn read_block(
        &mut self,
        _n: usize,
    ) -> RfResult<IqBlock> {
        // TODO: drain ring buffer filled by background streaming thread.

        use crate::rf::RfError;

        Err(RfError::Sdr(
            "SoapySDR hardware streaming not yet implemented; \
             use MockSdrSource for testing"
                .into(),
        ))
    }

    fn metrics(&self) -> SourceMetrics {
        self.metrics.clone()
    }
}

/// Very simple deterministic pseudo-noise in [-1, 1] based on xorshift64.
fn pseudo_noise(mut x: u64) -> f32 {
    // xorshift64
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;

    // Use lower 32 bits, map to [-1, 1] via i32 range
    let lo = (x & 0xFFFF_FFFF) as i32;

    lo as f32 / i32::MAX as f32
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use super::*;

    fn default_mock(tone_hz: f64) -> MockSdrSource {
        MockSdrSource::new(RfConfig::default(), tone_hz, 0.0)
    }

    #[test]
    fn mock_produces_correct_block_size() {
        let mut src = default_mock(0.0);
        let block = src.read_block(2048).unwrap();

        assert_eq!(block.samples.len(), 2048);
    }

    #[test]
    fn mock_zero_tone_is_constant_one() {
        // 0 Hz tone → exp(j·0) = 1+0j for all samples
        let mut src = default_mock(0.0);
        let block = src.read_block(8).unwrap();

        for s in &block.samples {
            assert!((s.re - 1.0).abs() < 1e-5, "re={}", s.re);
            assert!(s.im.abs() < 1e-5, "im={}", s.im);
        }
    }

    #[test]
    fn mock_tone_at_quarter_nyquist_rotates_correctly() {
        // tone = fs/4 → phase step = π/2 per sample
        let fs = 2_048_000.0_f64;
        let mut src = default_mock(fs / 4.0);
        let block = src.read_block(4).unwrap();
        // Expected: (1+0j), (0+1j), (-1+0j), (0-1j)
        let expected = [
            Complex32::new(1.0, 0.0),
            Complex32::new(0.0, 1.0),
            Complex32::new(-1.0, 0.0),
            Complex32::new(0.0, -1.0),
        ];

        for (s, e) in block.samples.iter().zip(expected.iter()) {
            assert!(
                (s.re - e.re).abs() < 1e-5,
                "re mismatch {} vs {}",
                s.re,
                e.re
            );
            assert!(
                (s.im - e.im).abs() < 1e-5,
                "im mismatch {} vs {}",
                s.im,
                e.im
            );
        }
    }

    #[test]
    fn mock_unit_amplitude() {
        let mut src = default_mock(50_000.0);
        let block = src.read_block(4096).unwrap();

        for s in &block.samples {
            let mag = (s.re * s.re + s.im * s.im).sqrt();

            assert!((mag - 1.0).abs() < 1e-4, "magnitude={mag}");
        }
    }

    #[test]
    fn mock_metrics_count_samples() {
        let mut src = default_mock(0.0);
        src.read_block(1000).unwrap();
        src.read_block(1000).unwrap();

        assert_eq!(src.metrics().total_samples, 2000);
    }

    #[test]
    fn mock_start_sample_increases() {
        let mut src = default_mock(0.0);
        let b1 = src.read_block(100).unwrap();
        let b2 = src.read_block(100).unwrap();

        assert_eq!(b1.start_sample, 0);
        assert_eq!(b2.start_sample, 100);
    }

    #[test]
    fn mock_with_noise_varies_samples() {
        let mut src = MockSdrSource::new(RfConfig::default(), 0.0, 0.1);
        let block = src.read_block(64).unwrap();
        // At least some samples should differ from pure tone (1+0j) due to noise
        let max_re_dev = block
            .samples
            .iter()
            .map(|s| (s.re - 1.0_f32).abs())
            .fold(0.0_f32, f32::max);

        assert!(max_re_dev > 0.0, "noise should perturb the signal");
    }

    #[test]
    fn mock_phase_continuous_across_blocks() {
        // Phase at end of block N should be the start of block N+1.
        let fs = 2_048_000.0_f64;
        let f = fs / 8.0; // π/4 per sample
        let mut src = default_mock(f);
        let _b1 = src.read_block(4).unwrap();
        let b2 = src.read_block(4).unwrap();
        // Last sample of b1 has phase = 3 * π/4. Next should be 4 * π/4 = π.
        let expected_phase = 4.0 * PI / 4.0;
        let got_re = b2.samples[0].re;
        let got_im = b2.samples[0].im;

        assert!((got_re - expected_phase.cos()).abs() < 1e-4);
        assert!((got_im - expected_phase.sin()).abs() < 1e-4);
    }
}
