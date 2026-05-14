//! FIR-фильтр и оконные функции для цифровой обработки сигналов (DSP).
//!
//! Модуль реализует:
//!
//! * оконные функции ([`Window`])
//! * FIR-фильтр с потоковой обработкой ([`FirFilter`])
//! * проектирование НЧ-фильтра методом окон (windowed-sinc)
//!
//! # Теория
//!
//! FIR-фильтр (Finite Impulse Response) реализует свёртку:
//!
//! ```text
//! y[n] = Σ h[k] · x[n - k]
//! ```
//!
//! где:
//!
//! * `x[n]` — входной сигнал
//! * `h[k]` — коэффициенты фильтра (taps)
//!
//! # Проектирование фильтра
//!
//! НЧ-фильтр строится как:
//!
//! ```text
//! h[n] = 2fc · sinc(2fc (n - M)) · w[n]
//! ```
//!
//! где:
//!
//! * `fc` — нормированная частота среза (`cutoff / fs`)
//! * `sinc(x) = sin(πx)/(πx)`
//! * `w[n]` — оконная функция
//!
//! Окно используется для:
//!
//! * подавления боковых лепестков
//! * контроля компромисса между шириной переходной зоны и затуханием
//!
//! # Поддерживаемые окна
//!
//! * Rectangular — минимальная обработка, сильные утечки
//! * Hamming — ~−43 dB
//! * Hann — ~−31 dB
//! * Blackman — ~−58 dB
//!
//! # Особенности реализации
//!
//! * потоковая обработка через внутреннюю линию задержки (`VecDeque`)
//! * сохранение состояния между вызовами `apply`
//! * нормализация коэффициентов по DC (усиление = 1)
//!
//! # Применение
//!
//! * антиалиасинговые фильтры (decimator)
//! * интерполяция (interpolator)
//! * выделение полосы (channel filtering)
//! * подготовка сигналов для корреляции GNSS

use std::{collections::VecDeque, f64::consts::PI};

use num_complex::Complex32;

/// Тип оконной функции, используемой при проектировании FIR-фильтра.
///
/// Окна применяются для уменьшения спектральных утечек при усечении
/// идеальной импульсной характеристики.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Window {
    /// Прямоугольное окно (без взвешивания).
    Rectangular,

    /// Окно Хэмминга.
    Hamming,

    /// Окно Ханна.
    Hann,

    /// Окно Блэкмана.
    Blackman,
}

/// FIR (Finite Impulse Response) фильтр.
///
/// Реализует дискретную свёртку входного сигнала с набором коэффициентов.
/// Поддерживает потоковую обработку с внутренней линией задержки.
pub struct FirFilter {
    /// Коэффициенты фильтра h[0..N].
    coeffs: Vec<f32>,

    /// Линия задержки (история входного сигнала).
    ///
    /// `state[0]` — последний поступивший сэмпл.
    state: VecDeque<Complex32>,
}

impl Window {
    /// Вычисляет значение оконной функции в точке `n` для длины окна `len`.
    ///
    /// # Аргументы
    /// - `n` — индекс отсчёта
    /// - `len` — длина окна
    ///
    /// # Возвращает
    /// Значение окна в диапазоне обычно `[0, 1]`.
    #[must_use]
    pub fn value(
        self,
        n: usize,
        len: usize,
    ) -> f64 {
        if len == 1 {
            return 1.0;
        }

        let m = (len - 1) as f64;
        let x = 2.0 * PI * n as f64 / m;

        match self {
            Window::Rectangular => 1.0,
            Window::Hamming => 0.54 - 0.46 * x.cos(),
            Window::Hann => 0.5 * (1.0 - x.cos()),
            Window::Blackman => 0.42 - 0.5 * x.cos() + 0.08 * (2.0 * x).cos(),
        }
    }
}

impl FirFilter {
    /// Создаёт FIR-фильтр с заданными коэффициентами.
    ///
    /// # Паника
    /// Паника возникает, если `coeffs` пуст.
    #[must_use]
    pub fn new(coeffs: Vec<f32>) -> Self {
        assert!(!coeffs.is_empty(), "coefficient vector must not be empty");

        let state_len = coeffs.len() - 1;

        Self {
            coeffs,
            state: VecDeque::from(vec![Complex32::default(); state_len]),
        }
    }

    /// Проектирует низкочастотный FIR-фильтр методом окон.
    ///
    /// # Аргументы
    /// - `cutoff_hz` — частота среза (Гц)
    /// - `sample_rate_hz` — частота дискретизации (Гц)
    /// - `num_taps` — количество коэффициентов фильтра
    /// - `window` — оконная функция
    ///
    /// # Паника
    /// - если `cutoff_hz` некорректна
    /// - если `num_taps < 1`
    #[must_use]
    pub fn low_pass(
        cutoff_hz: f64,
        sample_rate_hz: f64,
        num_taps: usize,
        window: Window,
    ) -> Self {
        assert!(cutoff_hz > 0.0 && cutoff_hz < sample_rate_hz / 2.0);

        let cutoff_norm = cutoff_hz / sample_rate_hz;
        let coeffs = design_low_pass_coeffs(cutoff_norm, num_taps, window);

        Self::new(coeffs)
    }

    /// Возвращает количество коэффициентов (taps) фильтра.
    #[must_use]
    pub const fn num_taps(&self) -> usize {
        self.coeffs.len()
    }

    /// Обрабатывает блок входных отсчётов.
    ///
    /// # Возвращает
    /// Вектор отфильтрованных значений той же длины.
    #[inline]
    pub fn apply_single(
        &mut self,
        x: Complex32,
    ) -> Complex32 {
        // Добавляем новый сэмпл в начало списка; линия задержки увеличивается до
        // coeffs.len()
        self.state.push_front(x);

        // Свертка
        let y: Complex32 = self
            .coeffs
            .iter()
            .zip(self.state.iter())
            .map(|(&h, &s)| s * h)
            .sum();

        // Удаляем самый старый образец, восстанавливая длину до coeffs.len() − 1
        self.state.pop_back();

        y
    }

    /// Фильтрует блок сэмплов, возвращая новый объект `Vec`.
    pub fn apply(
        &mut self,
        input: &[Complex32],
    ) -> Vec<Complex32> {
        input.iter().map(|&x| self.apply_single(x)).collect()
    }

    /// Сбрасывает внутреннее состояние фильтра (линию задержки).
    ///
    /// После вызова фильтр ведёт себя так, как будто обработка начинается с
    /// нуля.
    pub fn reset(&mut self) {
        for s in self.state.iter_mut() {
            *s = Complex32::default()
        }
    }

    /// Возвращает срез коэффициентов фильтра.
    #[must_use]
    pub fn coeffs(&self) -> &[f32] {
        &self.coeffs
    }
}

#[inline]
fn sinc(x: f64) -> f64 {
    if x.abs() < 1e-12 {
        1.0
    } else {
        let px = PI * x;

        px.sin() / px
    }
}

fn design_low_pass_coeffs(
    cutoff_norm: f64,
    num_taps: usize,
    window: Window,
) -> Vec<f32> {
    assert!(num_taps >= 1);
    assert!(cutoff_norm > 0.0 && cutoff_norm < 0.5);

    let m = (num_taps - 1) as f64 / 2.0;

    let mut coeffs: Vec<f32> = (0..num_taps)
        .map(|n| {
            let x = n as f64 - m;
            let h = 2.0 * cutoff_norm * sinc(2.0 * cutoff_norm * x); // поправил деление на умножение
            let w = window.value(n, num_taps);

            (h * w) as f32
        })
        .collect();

    // нормируем по DC
    let sum: f32 = coeffs.iter().sum();
    if sum.abs() > f32::EPSILON {
        for c in &mut coeffs {
            *c /= sum;
        }
    }

    coeffs
}

////////////////////////////////////////////////////////////////////////////////
// Тесты
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    // const FS: f64 = 2_048_000.0;

    #[test]
    fn test_rectangular_window_is_all_ones() {
        for n in 0..32 {
            assert_eq!(Window::Rectangular.value(n, 32), 1.0);
        }
    }

    #[test]
    fn test_haming_window_symmetry() {
        let len = 64;

        for n in 0..len / 2 {
            let w1 = Window::Hamming.value(n, len);
            let w2 = Window::Hamming.value(len - 1 - n, len);

            assert!((w1 - w2).abs() < 1e-10, "n={}: {} vs {}", n, w1, w2);
        }
    }

    #[test]
    fn test_hamming_window_endpoints() {
        let len = 64;

        // Hamming: w[0] = 0.54 - 0.46 = 0.08
        assert!((Window::Hamming.value(0, len) - 0.08).abs() < 1e-10);
        assert!((Window::Hamming.value(len - 1, len) - 0.08).abs() < 1e-10);
    }

    #[test]
    fn test_blackman_window_center_near_one() {
        let len = 65;
        let center = Window::Blackman.value(len / 2, len);

        assert!((center - 1.0).abs() < 1e-9, "center={}", center);
    }

    #[test]
    fn test_unit_filter_is_identity() {
        // h = [1.0] -> y[n] = x[n]
        let mut f = FirFilter::new(vec![1.0]);
        let input: Vec<Complex32> = (0..8).map(|n| Complex32::new(n as f32, 0.0)).collect();
        let out = f.apply(&input);

        for (x, y) in input.iter().zip(out.iter()) {
            assert!((x.re - y.re).abs() < 1e-6);
        }
    }

    #[test]
    fn test_two_tap_averager() {
        // h = [0.5, 0.5] -> y[n] = 0.5 * x[n] + 0.5 * x[n - 1]
        let mut f = FirFilter::new(vec![0.5, 0.5]);
        let input = vec![
            Complex32::new(1.0, 0.0),
            Complex32::new(1.0, 0.0),
            Complex32::new(0.0, 0.0),
        ];
        let out = f.apply(&input);

        assert!((out[0].re - 0.5).abs() < 1e-6); // 0.5 * 1 + 0.5 * 0 (sate)
        assert!((out[1].re - 1.0).abs() < 1e-6); // 0.5 * 1 + 0.5 * 1
        assert!((out[2].re - 0.5).abs() < 1e-6); // 0.5 * 0 + 0.5 * 1
    }

    #[test]
    fn test_state_preserved_across_blocks() {
        // Split input into two blocks and verify identical output to one long block
        let mut f1 = FirFilter::new(vec![0.5, 0.5]);
        let mut f2 = FirFilter::new(vec![0.5, 0.5]);
        let full_input: Vec<Complex32> = (0..8).map(|n| Complex32::new(n as f32, 0.0)).collect();
        let one_block = f1.apply(&full_input);
        let part1 = f2.apply(&full_input[..4]);
        let part2 = f2.apply(&full_input[4..]);
        let two_block: Vec<_> = part1.iter().chain(part2.iter()).cloned().collect();

        for (a, b) in one_block.iter().zip(two_block.iter()) {
            assert!((a.re - b.re).abs() < 1e-6, "a={} b={}", a.re, b.re);
        }
    }

    #[test]
    fn test_reset_clears_delay_line() {
        let mut f = FirFilter::new(vec![0.5, 0.5]);

        f.apply(&[Complex32::new(1.0, 0.0); 4]);
        f.reset();

        // После сброса состояние становится равным нулю → output[0] = 0.5 * input + 0.5
        // * 0
        let out = f.apply(&[Complex32::new(2.0, 0.0)]);

        assert!((out[0].re - 1.0).abs() < 1e-6);
    }
}
