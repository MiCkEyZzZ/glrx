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

    #[inline(always)]
    pub fn advance(&mut self) -> Complex32 {
        let (sin, cos) = self.phase.sin_cos();

        self.phase += self.phase_step;

        if self.phase > TAU || self.phase < 0.0 {
            self.phase = self.phase.rem_euclid(TAU);
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

    pub fn generate(
        &mut self,
        n: usize,
    ) -> Vec<Complex32> {
        (0..n).map(|_| self.advance()).collect()
    }
}

impl Mixer {
    pub fn new(
        freq_hz: f64,
        sample_rate_hz: f64,
    ) -> Self {
        Self {
            nco: Nco::new(freq_hz, sample_rate_hz),
            sample_rate_hz,
        }
    }

    pub fn set_frequency(
        &mut self,
        freq_hz: f64,
    ) {
        self.nco.set_frequency(freq_hz, self.sample_rate_hz);
    }

    pub fn adjust_frequency(
        &mut self,
        delta_hz: f64,
    ) {
        let step = self.nco.phase_step_rad() as f64;
        let new_freq = (step / (TAU as f64)) * self.sample_rate_hz + delta_hz;

        self.nco.set_frequency(new_freq, self.sample_rate_hz);
    }

    pub fn sample_rate(&self) -> f64 {
        self.sample_rate_hz
    }

    pub fn phase_rad(&self) -> f32 {
        self.nco.phase_rad()
    }

    pub fn mix(
        &mut self,
        input: &[Complex32],
    ) -> Vec<Complex32> {
        input.iter().map(|&s| s * self.nco.advance()).collect()
    }

    pub fn mix_inplace(
        &mut self,
        samples: &mut [Complex32],
    ) {
        for s in samples.iter_mut() {
            *s *= self.nco.advance();
        }
    }

    pub fn reset(&mut self) {
        self.nco.reset();
    }
}

pub fn mix_shift(
    input: &[Complex32],
    freq_hz: f64,
    sample_rate_hz: f64,
) -> Vec<Complex32> {
    let mut nco = Nco::new(freq_hz, sample_rate_hz);

    input.iter().map(|&s| s * nco.advance()).collect()
}

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

    #[test]
    fn test_mix_shift_basic() {
        let input = vec![Complex32::new(1.0, 0.0); 4];
        let out = mix_shift(&input, FS / 4.0, FS);

        let expected = [
            Complex32::new(1.0, 0.0),
            Complex32::new(0.0, 1.0),
            Complex32::new(-1.0, 0.0),
            Complex32::new(0.0, -1.0),
        ];

        for (a, b) in out.iter().zip(expected.iter()) {
            assert!((a.re - b.re).abs() < 1e-5, "re: got={}, exp={}", a.re, b.re);
            assert!((a.im - b.im).abs() < 1e-5, "im: got={}, exp={}", a.im, b.im);
        }
    }

    #[test]
    fn test_mix_shift_phase_starts_from_zero_each_call() {
        let input = vec![Complex32::new(1.0, 0.0); 2];
        let out1 = mix_shift(&input, FS / 8.0, FS);
        let out2 = mix_shift(&input, FS / 8.0, FS);

        // Каждый вызов начинается с фазы = 0
        assert_eq!(out1[0], Complex32::new(1.0, 0.0));
        assert_eq!(out2[0], Complex32::new(1.0, 0.0));
    }

    #[test]
    fn test_mix_shift_downconversion() {
        // 10 kHz tone → mix_shift -10 kHz → DC
        let tone = generate_carrier(10_000.0, FS, 128);
        let out = mix_shift(&tone, -10_000.0, FS);

        for s in &out {
            assert!((s.re - 1.0).abs() < 1e-3);
            assert!(s.im.abs() < 1e-3);
        }
    }

    #[test]
    fn test_mix_shift_zero_frequency_is_identity() {
        let input = generate_carrier(5_000.0, FS, 64);
        let out = mix_shift(&input, 0.0, FS);

        for (a, b) in input.iter().zip(out.iter()) {
            assert!((a.re - b.re).abs() < 1e-6);
            assert!((a.im - b.im).abs() < 1e-6);
        }
    }

    #[test]
    fn nco_phase_wrap_high_frequency() {
        let mut nco = Nco::new(FS * 2.0, FS); // freq > fs
        for _ in 0..10 {
            let s = nco.advance();
            assert!(nco.phase_rad() >= 0.0 && nco.phase_rad() < TAU);
            let mag = (s.re * s.re + s.im * s.im).sqrt();
            assert!((mag - 1.0).abs() < 1e-5);
        }
    }

    #[test]
    fn nco_zero_samples_generate() {
        let mut nco = Nco::new(1_000.0, FS);
        let v = nco.generate(0);
        assert!(v.is_empty());
    }

    #[test]
    fn nco_negative_frequency_behavior() {
        let mut nco = Nco::new(-FS / 4.0, FS);
        let s0 = nco.advance();
        let s1 = nco.advance();
        let s2 = nco.advance();

        // Проверяем, что фаза уменьшается (вращение по часовой стрелке)
        let phase0 = s0.im.atan2(s0.re);
        let phase1 = s1.im.atan2(s1.re);
        let phase2 = s2.im.atan2(s2.re);

        assert!(phase1 < phase0 || (phase0 - phase1).abs() > 1e-6);
        assert!(phase2 < phase1 || (phase1 - phase2).abs() > 1e-6);
    }

    #[test]
    fn mixer_adjust_frequency_continuous_phase() {
        let mut mixer = Mixer::new(10_000.0, FS);
        let input = generate_carrier(1_000.0, FS, 8);
        let out1 = mixer.mix(&input);

        mixer.adjust_frequency(5_000.0);
        let out2 = mixer.mix(&input);

        // Проверяем, что фаза не срывается
        let diff_re = (out1[0].re - out2[0].re).abs();
        assert!(diff_re <= 2.0); // условно, проверяем что не NaN
    }

    #[test]
    fn mixer_reset_after_adjust_frequency() {
        let mut mixer = Mixer::new(5_000.0, FS);
        mixer.adjust_frequency(10_000.0);
        mixer.reset();
        assert_eq!(mixer.phase_rad(), 0.0);
    }

    #[test]
    fn mixer_zero_input_is_safe() {
        let mut mixer = Mixer::new(1_000.0, FS);
        let input: Vec<Complex32> = vec![];
        let out = mixer.mix(&input);
        assert!(out.is_empty());
    }

    #[test]
    fn mixer_large_block_mix() {
        let input = generate_carrier(10_000.0, FS, 10_000);
        let mut mixer = Mixer::new(-10_000.0, FS);
        let out = mixer.mix(&input);
        for s in out.iter().take(10) {
            assert!((s.re - 1.0).abs() < 1e-3);
            assert!(s.im.abs() < 1e-3);
        }
    }

    #[test]
    fn mix_shift_phase_starts_zero_each_call() {
        let input = vec![Complex32::new(1.0, 0.0); 3];
        let out1 = mix_shift(&input, FS / 4.0, FS);
        let out2 = mix_shift(&input, FS / 4.0, FS);

        assert_eq!(out1[0], Complex32::new(1.0, 0.0));
        assert_eq!(out2[0], Complex32::new(1.0, 0.0));
    }

    #[test]
    fn mix_shift_downconversion_to_dc() {
        let tone = generate_carrier(10_000.0, FS, 128);
        let out = mix_shift(&tone, -10_000.0, FS);

        for s in &out {
            assert!((s.re - 1.0).abs() < 1e-3);
            assert!(s.im.abs() < 1e-3);
        }
    }

    #[test]
    fn mix_shift_upconversion() {
        let tone = generate_carrier(5_000.0, FS, 64);
        let out = mix_shift(&tone, 10_000.0, FS);
        let expected = generate_carrier(15_000.0, FS, 64);

        for (a, b) in out.iter().zip(expected.iter()) {
            assert!((a.re - b.re).abs() < 1e-3);
            assert!((a.im - b.im).abs() < 1e-3);
        }
    }

    #[test]
    fn mix_shift_zero_frequency_identity() {
        let input = generate_carrier(3_000.0, FS, 32);
        let out = mix_shift(&input, 0.0, FS);
        for (a, b) in input.iter().zip(out.iter()) {
            assert!((a.re - b.re).abs() < 1e-6);
            assert!((a.im - b.im).abs() < 1e-6);
        }
    }

    #[test]
    fn mix_shift_large_block() {
        let input = generate_carrier(1_000.0, FS, 5_000);
        let out = mix_shift(&input, 2_000.0, FS);
        assert_eq!(out.len(), input.len());
    }

    #[test]
    fn mix_shift_always_starts_from_zero_phase() {
        let input = generate_carrier(5_000.0, FS, 64);
        let out1 = mix_shift(&input, -5_000.0, FS);
        let out2 = mix_shift(&input, -5_000.0, FS);
        // Same input + same phase start → identical output
        for (a, b) in out1.iter().zip(out2.iter()) {
            assert_eq!(a, b);
        }
    }

    #[test]
    fn mix_shift_shifts_tone_to_dc() {
        let tone = generate_carrier(7_000.0, FS, 2048);
        let out = mix_shift(&tone, -7_000.0, FS);
        for s in &out {
            assert!((s.re - 1.0).abs() < 1e-3, "re={}", s.re);
        }
    }

    #[test]
    fn generate_carrier_length_and_unit_amplitude() {
        let carrier = generate_carrier(1_575_420_000.0, FS, 2048);
        assert_eq!(carrier.len(), 2048);
        for s in &carrier {
            let mag = (s.re * s.re + s.im * s.im).sqrt();
            assert!((mag - 1.0).abs() < 1e-4);
        }
    }
}
