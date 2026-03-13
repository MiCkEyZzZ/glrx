use std::f32::consts::TAU;

use num_complex::Complex32;

/// Численно управляемый осциллятор (NCO).
///
/// Генерирует `exp(j·φ)`, где `φ` увеличивается на `Δφ = 2π·f/fₛ` на каждый
/// сэмпл. Фаза непрерывна между вызовами.
#[derive(Debug, Clone)]
pub struct Nco {
    phase: f32,
    phase_step: f32,
}

impl Nco {
    /// Создаёт генератор случайных чисел (NCO) на частоте `freq_hz` для
    /// заданной частоты дискретизации.
    pub fn new(
        freq_hz: f64,
        sample_rate_hz: f64,
    ) -> Self {
        Self {
            phase: 0.0,
            phase_step: (TAU as f64 * freq_hz / sample_rate_hz) as f32,
        }
    }

    pub fn set_frequency(
        &mut self,
        freq_hz: f64,
        sample_rate_hz: f64,
    ) {
        self.phase_step = (TAU as f64 * freq_hz / sample_rate_hz) as f32;
    }

    /// Возвращает текущий выходной сигнал NCO и сдвигает фазовый аккумулятор.
    #[inline(always)]
    pub fn advance(&mut self) -> Complex32 {
        let (sin, cos) = self.phase.sin_cos();

        self.phase += self.phase_step;

        if self.phase >= TAU {
            self.phase -= TAU;
        } else {
            self.phase += TAU;
        };

        Complex32::new(cos, sin)
    }

    #[inline]
    pub fn phase_rad(&self) -> f32 {
        self.phase
    }

    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// Генерирует `n` последовательных выборок NCO.
    pub fn generate(
        &mut self,
        n: usize,
    ) -> Vec<Complex32> {
        (0..n).map(|_| self.advance()).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 2_048_000.0;

    #[test]
    fn test_nco_zero_freq_is_constant_one() {
        let mut nco = Nco::new(0.0, FS);

        for _ in 0..8 {
            let s = nco.advance();

            assert!((s.re - 1.0).abs() < 1e-5, "re={}", s.re);
            assert!(s.im.abs() < 1e-5, "im={}", s.im);
        }
    }

    #[test]
    fn test_nco_quater_nyquist_four_phase_cycle() {
        let mut nco = Nco::new(FS / 4.0, FS);
        let expected = [
            Complex32::new(1.0, 0.0),
            Complex32::new(0.0, 1.0),
            Complex32::new(-1.0, 0.0),
            Complex32::new(0.0, -1.0),
        ];

        for e in &expected {
            let s = nco.advance();

            assert!((s.re - e.re).abs() < 1e-5, "re: got={} exp={}", s.re, e.re);
            assert!((s.im - e.im).abs() < 1e-5, "im: got={} exp={}", s.im, e.im);
        }
    }

    #[test]
    fn test_nco_phase_continuous_across_windows() {
        let mut nco = Nco::new(FS / 8.0, FS);

        for _ in 0..4 {
            nco.advance();
        }

        let s = nco.advance(); // sample 4: phase = π

        assert!((s.re + 1.0).abs() < 1e-4, "re={}", s.re);
        assert!(s.im.abs() < 1e-4, "im={}", s.im);
    }

    #[test]
    fn test_nco_unit_amplitude_long_run() {
        let mut nco = Nco::new(12_345.6, FS);

        for i in 0..8192 {
            let s = nco.advance();
            let mag = (s.re * s.re + s.im * s.im).sqrt();

            assert!((mag - 1.0).abs() < 1e-4, "sample {}: mag={}", i, mag);
        }
    }

    #[test]
    fn test_nco_generate_returns_correct_length() {
        let mut nco = Nco::new(1000.0, FS);

        assert_eq!(nco.generate(128).len(), 128);
    }

    #[test]
    fn test_nco_set_frequency_no_phase_jump() {
        let mut nco = Nco::new(1000.0, FS);

        for _ in 0..10 {
            nco.advance();
        }

        let phase_before = nco.phase_rad();

        nco.set_frequency(5000.0, FS);

        assert_eq!(nco.phase_rad(), phase_before);
    }

    #[test]
    fn test_nco_reset_restarts_from_zero() {
        let mut nco = Nco::new(1000.0, FS);

        for _ in 0..100 {
            nco.advance();
        }

        nco.reset();

        let s = nco.advance();

        assert!((s.re - 1.0).abs() < 1e-5);
        assert!(s.im.abs() < 1e-5);
    }
}
