use std::{f32::consts::TAU, sync::Arc};

use num_complex::Complex32;

use crate::{IqBlock, IqSource, RfConfig, RfResult, SourceMetrics};

/// Mock SDR - always available, useful for unit tests and CI.
pub struct MockSdrSource {
    config: Arc<RfConfig>,
    tone_hz: f64,
    noise_amplitude: f32,
    phase: f32,
    phase_step: f32,
    next_sample: u64,
    metrics: SourceMetrics,
}

/// Аппартный SDR через абстракцию SoapySDR.
///
/// Поддерживает любые устройства с драйвером SoapySDR: RTL-SDR, HackRF, USRP,
/// LimeSDR, PlutoSDR и др.
///
/// # Требования на этапе компиляции
///
/// Доступно только при включенном Cargo-фиче `sdr` и установленной
/// C++ библиотеке `SoapySDR` на машине сборки.
///
/// # Модель потоков
///
/// Фоновый поток управляет API потоковой передачи SoapySDR и записывает
/// сэмплы в кольцевой буфер. `read_block` считывает данные из буфера,
/// блокируясь максимум на `timeout`, если данных недостаточно.
///
/// # Использование
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
    config: Arc<RfConfig>,
    driver_args: String,
    metrics: SourceMetrics,
    // Когда будут добавлены привязки SoapySDR, сюда поместится реальный дескриптор устройства.
    // _device: soapysdr::Device,
    // _stream: soapysdr::RxStream<Complex32>,
}

impl MockSdrSource {
    /// Create a new mock source.
    pub fn new(
        config: Arc<RfConfig>,
        tone_hz: f64,
        noise_amplitude: f32,
    ) -> Self {
        let phase_step = (TAU as f64 * tone_hz / config.sample_rate_hz) as f32;

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
    pub fn tone_hz(&self) -> f64 {
        self.tone_hz
    }
}

#[cfg(feature = "sdr")]
impl SoapySource {
    /// Открывает устройство SoapySDR.
    ///
    /// * `driver_args` — строка фильтра устройства SoapySDR, например
    ///   `"driver=rtlsdr"`, `"driver=hackrf"`, или `""`, чтобы использовать
    ///   первое доступное устройство.
    pub fn open(
        driver_args: &str,
        config: Arc<RfConfig>,
    ) -> RfResult<Self> {
        config.validate()?;

        // План будущей реализации
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

    /// Возвращает список всех устройств SoapySDR, видимых в системе.
    pub fn enumerate() -> Vec<String> {
        // В будущем: soapysdr::enumerate("").map(|kw| kw.to_string()).collect()
        vec!["<SoapySDR enumeration not yet implemented>".to_string()]
    }
}

impl IqSource for MockSdrSource {
    fn config(&self) -> &RfConfig {
        &self.config.as_ref()
    }

    fn name(&self) -> &str {
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
            self.phase += self.phase_step;

            if self.phase >= TAU {
                self.phase -= TAU
            }
        }

        let len = samples.len() as u64;
        self.next_sample += len;
        self.metrics.total_samples += len;
        self.metrics.measured_rate_hz = Some(self.config.sample_rate_hz);

        Ok(IqBlock {
            samples,
            config: Arc::clone(&self.config),
            start_sample,
        })
    }

    fn metrics(&self) -> SourceMetrics {
        self.metrics.clone()
    }
}

/// Very simple deterministic pseudo-noise in [-1, 1] based in xorshift64.
fn pseudo_noise(mut x: u64) -> f32 {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;

    let lo = (x & 0xFFFF_FFFF) as i32;
    lo as f32 / i32::MAX as f32
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use super::*;

    fn default_mock(tone_hz: f64) -> MockSdrSource {
        MockSdrSource::new(Arc::new(RfConfig::default()), tone_hz, 0.0)
    }

    #[test]
    fn test_mock_produces_correct_block_size() {
        let mut src = default_mock(0.0);
        let block = src.read_block(2048).unwrap();

        assert_eq!(block.samples.len(), 2048);
    }

    #[test]
    fn test_mock_zero_tone_is_constant_one() {
        // 0 Hz tone → exp(j*0) = 1+0j for all samples
        let mut src = default_mock(0.0);
        let block = src.read_block(8).unwrap();

        for s in &block.samples {
            assert!((s.re - 1.0).abs() < 1e-5, "re={}", s.re);
            assert!(s.im.abs() < 1e-5, "im={}", s.im);
        }
    }

    #[test]
    fn test_mock_tone_at_quarter_nyquist_rotates_correctly() {
        // tone = fs/4 → phase step = PI/2 per sample
        let fs = 2_048_000.0_f64;
        let mut src = default_mock(fs / 4.0);
        let block = src.read_block(4).unwrap();

        // Expected: (1 + 0j), (0 + 1j), (-1 + 0j), (0 - 1j)
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
    fn test_mock_unit_amplitude() {
        let mut src = default_mock(50_000.0);
        let block = src.read_block(4096).unwrap();

        for s in &block.samples {
            let mag = (s.re * s.re + s.im * s.im).sqrt();

            assert!((mag - 1.0).abs() < 1e-4, "magnitude={}", mag);
        }
    }

    #[test]
    fn test_mock_metrics_count_samples() {
        let mut src = default_mock(0.0);

        src.read_block(1000).unwrap();
        src.read_block(1000).unwrap();

        assert_eq!(src.metrics().total_samples, 2000);
    }

    #[test]
    fn test_mock_start_sample_increases() {
        let mut src = default_mock(0.0);
        let b1 = src.read_block(100).unwrap();
        let b2 = src.read_block(100).unwrap();

        assert_eq!(b1.start_sample, 0);
        assert_eq!(b2.start_sample, 100);
    }

    #[test]
    fn test_mock_with_noise_varies_samples() {
        let mut src = MockSdrSource::new(Arc::new(RfConfig::default()), 0.0, 0.1);
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
    fn test_mock_phase_continuous_across_blocks() {
        // Phase at end of block N should be the start of block N+1.
        let fs = 2_048_000.0_f64;
        let f = fs / 8.0; // PI/4 per sample
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

    #[test]
    fn test_noise_deterministic() {
        let mut src1 = MockSdrSource::new(Arc::new(RfConfig::default()), 0.0, 0.1);
        let mut src2 = MockSdrSource::new(Arc::new(RfConfig::default()), 0.0, 0.1);

        let b1 = src1.read_block(128).unwrap();
        let b2 = src2.read_block(128).unwrap();

        for (a, b) in b1.samples.iter().zip(b2.samples.iter()) {
            assert!((a.re - b.re).abs() < 1e-6);
            assert!((a.im - b.im).abs() < 1e-6);
        }
    }

    #[test]
    fn test_read_zero_samples() {
        let mut src = default_mock(0.0);
        let block = src.read_block(0).unwrap();

        assert_eq!(block.samples.len(), 0);
        assert_eq!(block.start_sample, 0);
        assert_eq!(src.metrics().total_samples, 0);
    }

    #[test]
    fn test_tone_accessor() {
        let src = default_mock(12345.0);

        assert_eq!(src.tone_hz(), 12345.0);
    }

    #[test]
    fn test_metrics_rate_reported() {
        let mut src = default_mock(0.0);

        src.read_block(10).unwrap();

        let metrics = src.metrics();

        assert_eq!(metrics.measured_rate_hz, Some(src.config().sample_rate_hz));
    }

    #[test]
    fn test_phase_wraparound() {
        let mut src = default_mock(1_000_000.0);
        let _ = src.read_block(10_000).unwrap();

        assert!(src.phase >= 0.0);
        assert!(src.phase < TAU);
    }
}
