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

/// Микшер частот — умножает IQ-сигнал на `exp(j * 2π * f * t)`.
///
/// Установка отрицательного значения `freq_hz` сдвигает сигнал вниз (понижающее
/// преобразование). Фаза сохраняется при вызовах `mix` для использования в
/// потоковом режиме.
#[derive(Debug, Clone)]
pub struct Mixer {
    nco: Nco,
    sample_rate_hz: f64,
}

impl Nco {
    /// Создаёт NCO на частоте `freq_hz` для заданной частоты дискретизации
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
        } else if self.phase < 0.0 {
            self.phase += TAU;
        }

        Complex32::new(cos, sin)
    }

    #[inline]
    pub fn phase_rad(&self) -> f32 {
        self.phase
    }

    #[inline]
    pub fn phase_step_rad(&self) -> f32 {
        self.phase_step
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

impl Mixer {
    /// Создаёт новый микшер на частоте `freq_hz`.
    pub fn new(
        freq_hz: f64,
        sample_rate_hz: f64,
    ) -> Self {
        Self {
            nco: Nco::new(freq_hz, sample_rate_hz),
            sample_rate_hz,
        }
    }

    /// Обновляет частоту смешивания без разрыва фазы.
    pub fn set_frequency(
        &mut self,
        freq_hz: f64,
    ) {
        self.nco.set_frequency(freq_hz, self.sample_rate_hz);
    }

    /// Регулирует частоту на величину дельты.
    pub fn adjust_frequency(
        &mut self,
        delta_hz: f64,
    ) {
        let step = self.nco.phase_step_rad() as f64;
        let new_freq = (step / (TAU as f64)) * self.sample_rate_hz + delta_hz;

        self.nco.set_frequency(new_freq, self.sample_rate_hz);
    }

    /// Настраивает частоту дискретизации.
    pub fn sample_rate_hz(&self) -> f64 {
        self.sample_rate_hz
    }

    /// Текущая фаза NCO в радианах.
    pub fn phase_rad(&self) -> f32 {
        self.nco.phase_rad()
    }

    /// Смешиваем `input` с NCO, возвращая новый выделенный `Vec`.
    pub fn mix(
        &mut self,
        input: &[Complex32],
    ) -> Vec<Complex32> {
        input.iter().map(|&s| s * self.nco.advance()).collect()
    }

    /// Смешиваем `input` с NCO, возвращая новый выделенный `Vec`.
    pub fn mix_inplace(
        &mut self,
        samples: &mut [Complex32],
    ) {
        for s in samples.iter_mut() {
            *s *= self.nco.advance();
        }
    }

    /// Сброс фазового аккумулятора до нуля.
    pub fn reset(&mut self) {
        self.nco.reset();
    }
}

/// Генерирует комплексный несущий тон на частоте `freq_hz`.
pub fn generate_carrier(
    freq_hz: f64,
    sample_rate_hz: f64,
    n: usize,
) -> Vec<Complex32> {
    Nco::new(freq_hz, sample_rate_hz).generate(n)
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

    #[test]
    fn test_nco_phase_step_matches_frequency() {
        let freq = 10_000.0;
        let nco = Nco::new(freq, FS);
        let expected = (TAU as f64 * freq / FS) as f32;

        assert!((nco.phase_step_rad() - expected).abs() < 1e-6);
    }

    #[test]
    fn test_nco_negative_frequency_rotates_clockwise() {
        let mut nco = Nco::new(-FS / 4.0, FS);
        let expected = [
            Complex32::new(1.0, 0.0),
            Complex32::new(0.0, -1.0),
            Complex32::new(-1.0, 0.0),
            Complex32::new(0.0, 1.0),
        ];

        for e in &expected {
            let s = nco.advance();

            assert!((s.re - e.re).abs() < 1e-5);
            assert!((s.im - e.im).abs() < 1e-5);
        }
    }

    #[test]
    fn test_nco_phase_wrap_range() {
        let mut nco = Nco::new(12345.0, FS);

        for _ in 0..10000 {
            nco.advance();

            let phase = nco.phase_rad();

            assert!(phase >= 0.0 && phase < TAU);
        }
    }

    #[test]
    fn test_mixer_downconverts_tone_to_dc() {
        // 10 kHz tone mixed with -10 kHz → DC (all 1+0j)
        let tone = generate_carrier(10_000.0, FS, 2048);
        let mut mixer = Mixer::new(-10_000.0, FS);
        let out = mixer.mix(&tone);

        for s in &out {
            assert!((s.re - 1.0).abs() < 1e-3, "re={}", s.re);
            assert!(s.im.abs() < 1e-3, "im={}", s.im);
        }
    }

    #[test]
    fn test_mixer_phase_continuous_across_blocks() {
        // f = fs/4, two 4-simple blocks
        let ones = vec![Complex32::new(1.0, 0.0); 4];
        let mut mixer = Mixer::new(FS / 4.0, FS);
        let b1 = mixer.mix(&ones);
        let b2 = mixer.mix(&ones);

        // b1[3]: phase = 3 * (π/2) = 3π/2 → (0, -1)
        assert!((b1[3].im + 1.0).abs() < 1e-4, "b1[3].im={}", b1[3].im);

        // b2[0]: phase = 4·(π/2) = 2π = 0 → (1, 0)
        assert!((b2[0].re - 1.0).abs() < 1e-4, "b2[0].re={}", b2[0].re);
    }

    #[test]
    fn test_mixer_inplace_equals_alloc() {
        let input = generate_carrier(3_000.0, FS, 64);
        let mut m1 = Mixer::new(5_000.0, FS);
        let out_alloc = m1.mix(&input);
        let mut m2 = Mixer::new(5_000.0, FS);
        let mut out_inplace = input.clone();

        m2.mix_inplace(&mut out_inplace);

        for (a, b) in out_alloc.iter().zip(out_inplace.iter()) {
            assert!((a.re - b.re).abs() < 1e-6);
            assert!((a.im - b.im).abs() < 1e-6);
        }
    }

    #[test]
    fn test_mixer_reset_restarts_phase() {
        let input = vec![Complex32::new(1.0, 0.0); 32];
        let mut m = Mixer::new(FS / 4.0, FS);
        let out1 = m.mix(&input);

        m.reset();

        let out2 = m.mix(&input);

        for (a, b) in out1.iter().zip(out2.iter()) {
            assert!((a.re - b.re).abs() < 1e-6);
        }
    }

    #[test]
    fn test_nco_matches_formula() {
        let freq = 10_000.0;
        let mut nco = Nco::new(freq, FS);

        for n in 0..128 {
            let s = nco.advance();
            let phase = (TAU as f64 * freq * n as f64 / FS) as f32;
            let expected = Complex32::new(phase.cos(), phase.sin());

            assert!((s.re - expected.re).abs() < 1e-4);
            assert!((s.im - expected.im).abs() < 1e-4);
        }
    }

    #[test]
    fn test_generate_equals_manual_advance() {
        let mut nco1 = Nco::new(1234.0, FS);
        let mut nco2 = nco1.clone();

        let a = nco1.generate(128);
        let b: Vec<_> = (0..128).map(|_| nco2.advance()).collect();

        assert_eq!(a, b);
    }

    #[test]
    fn test_mixer_zero_freq_is_identity() {
        let input = generate_carrier(10_000.0, FS, 256);
        let mut mixer = Mixer::new(0.0, FS);
        let out = mixer.mix(&input);

        for (a, b) in input.iter().zip(out.iter()) {
            assert!((a.re - b.re).abs() < 1e-6);
            assert!((a.im - b.im).abs() < 1e-6);
        }
    }

    #[test]
    fn test_mixer_upconvert() {
        let tone = generate_carrier(5_000.0, FS, 256);
        let mut mixer = Mixer::new(10_000.0, FS);
        let out = mixer.mix(&tone);

        let expected = generate_carrier(15_000.0, FS, 256);

        for (a, b) in out.iter().zip(expected.iter()) {
            assert!((a.re - b.re).abs() < 1e-3);
            assert!((a.im - b.im).abs() < 1e-3);
        }
    }

    #[test]
    fn test_adjust_frequency_changes_step() {
        let mut mixer = Mixer::new(10_000.0, FS);

        let before = mixer.nco.phase_step_rad();
        mixer.adjust_frequency(1_000.0);
        let after = mixer.nco.phase_step_rad();

        assert!(after > before);
    }
}
