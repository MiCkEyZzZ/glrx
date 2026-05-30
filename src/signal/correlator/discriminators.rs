//! Модуль корреляции GNSS: Early–Prompt–Late (EPL) и дискриминаторы.
//!
//! Содержит:
//!
//! * [`EplOutput`] — результат корреляции за один интервал интеграции
//! * DLL-дискриминаторы (кодовая петля):
//!   * Normalised Early-Late Power (NELP)
//!   * Early-Late Envelope (ELE)
//! * PLL-дискриминаторы (несущая):
//!   * atan2 discriminator
//!   * Decision-Directed atan
//!
//! # Контекст использования
//!
//! Модуль применяется в tracking-контуре GNSS-приёмника после этапа:
//!
//! ```text
//! RF → downconversion → correlator → EPL → DLL/PLL
//! ```
//!
//! Где:
//!
//! * входной сигнал уже приведён к базовой полосе (baseband)
//! * выполнена когерентная интеграция (обычно 1 ms для GPS L1 C/A)
//!
//! # Назначение EPL
//!
//! Early–Prompt–Late коррелятор используется для оценки:
//!
//! * ошибки задержки кода (DLL)
//! * ошибки фазы несущей (PLL)
//!
//! # Замечания по устойчивости
//!
//! * `dll_nelp` устойчив к изменению амплитуды сигнала (нормирован)
//! * `dll_ele` чувствителен к уровню сигнала
//! * `pll_atan2` требует известного навигационного бита
//! * `pll_dd_atan` устраняет 180° неоднозначность

use num_complex::Complex32;

/// Результат одного интервала корреляции Early–Prompt–Late.
///
/// Структура содержит три выхода коррелятора,
/// которые используются в tracking-контурах GNSS-приёмника:
///
/// * **Early (E)** — ранняя реплика кода
/// * **Prompt (P)** — основная реплика
/// * **Late (L)** — поздняя реплика
///
/// Эти значения вычисляются после когерентного накопления
/// (обычно 1 ms для GPS L1 C/A) и используются в:
///
/// * **DLL (Delay Lock Loop)** — слежение за фазой кода
/// * **PLL (Phase Lock Loop)** — слежение за фазой несущей
#[derive(Debug, Clone)]
pub struct EplOutput {
    /// Ранняя ветвь коррелятора (Early).
    ///
    /// Код опережает текущую оценку фазы примерно на **½ чипа**.
    pub early: Complex32,

    /// Основная ветвь коррелятора (Prompt).
    ///
    /// Код совпадает с текущей оценкой фазы.
    pub prompt: Complex32,

    /// Поздняя ветвь коррелятора (Late).
    ///
    /// Код запаздывает относительно prompt примерно на **½ чипа**.
    pub late: Complex32,
}

impl EplOutput {
    /// DLL дискриминатор **Normalised Early-Late Power (NELP)**.
    ///
    /// Формула:
    ///
    /// ```text
    /// (|E|² − |L|²) / (|E|² + |L|²)
    /// ```
    ///
    /// Свойства:
    ///
    /// * диапазон ≈ **[-1, +1]**
    /// * `0` → код синхронизирован
    /// * `>0` → код запаздывает
    /// * `<0` → код опережает
    ///
    /// Это наиболее распространённый DLL-дискриминатор
    /// в GNSS-приёмниках.
    #[must_use]
    pub fn dll_nelp(&self) -> f32 {
        let pe = self.early.norm_sqr();
        let pl = self.late.norm_sqr();
        let denom = pe + pl;

        if denom < f32::EPSILON {
            0.0
        } else {
            (pe - pl) / denom
        }
    }

    /// DLL дискриминатор **Early-Late Envelope (ELE)**.
    ///
    /// Формула:
    ///
    /// ```text
    /// |E| − |L|
    /// ```
    ///
    /// В отличие от `dll_nelp`, этот дискриминатор **не нормирован**
    /// по мощности и поэтому зависит от амплитуды сигнала.
    #[must_use]
    pub fn dll_ele(&self) -> f32 {
        self.early.norm() - self.late.norm()
    }

    /// PLL дискриминатор на основе функции `atan2`.
    ///
    /// Формула:
    ///
    /// ```text
    /// atan2(Q, I)
    /// ```
    ///
    /// Диапазон:
    ///
    /// ```text
    /// (-π, π]
    /// ```
    ///
    /// Ноль соответствует идеальной фазовой синхронизации.
    ///
    /// Требует известного навигационного бита
    /// (иначе возникает неоднозначность 180°).
    #[must_use]
    pub fn pll_atan2(&self) -> f32 {
        self.prompt.im.atan2(self.prompt.re)
    }

    /// Decision-Directed PLL дискриминатор.
    ///
    /// Формула:
    ///
    /// ```text
    /// atan(Q / |I|)
    /// ```
    ///
    /// Диапазон:
    ///
    /// ```text
    /// (-π/2, π/2]
    /// ```
    ///
    /// Устраняет неоднозначность 180° навигационного бита,
    /// поэтому часто используется для BPSK сигналов
    /// (например GPS L1 C/A).
    #[must_use]
    pub fn pll_dd_atan(&self) -> f32 {
        let i = self.prompt.re;
        let q = self.prompt.im;

        (q / i.abs().max(f32::EPSILON)).atan()
    }

    /// Мощность prompt-ветви коррелятора.
    ///
    /// ```text
    /// |P|²
    /// ```
    #[must_use]
    pub fn prompt_power(&self) -> f32 {
        self.prompt.norm_sqr()
    }

    /// In-phase компонент prompt-ветви (I).
    #[must_use]
    pub const fn prompt_i(&self) -> f32 {
        self.prompt.re
    }

    /// Quadrature компонент prompt-ветви (Q).
    #[must_use]
    pub const fn prompt_q(&self) -> f32 {
        self.prompt.im
    }
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use std::f32::consts::{FRAC_PI_2, PI};

    use super::*;

    fn epl(
        e: f32,
        p_re: f32,
        p_im: f32,
        l: f32,
    ) -> EplOutput {
        EplOutput {
            early: Complex32::new(e, 0.0),
            prompt: Complex32::new(p_re, p_im),
            late: Complex32::new(l, 0.0),
        }
    }

    #[test]
    fn test_dll_nelp_zero_when_equal() {
        let out = epl(1.0, 1.0, 0.0, 1.0);

        assert!(out.dll_nelp().abs() < 1e-6);
    }

    #[test]
    fn test_dll_nelp_positive_early_stronger() {
        let out = epl(2.0, 1.0, 0.0, 1.0);

        assert!(out.dll_nelp() > 0.0);
    }

    #[test]
    fn test_dll_nelp_negative_late_stronger() {
        let out = epl(1.0, 1.0, 0.0, 2.0);

        assert!(out.dll_nelp() < 0.0);
    }

    #[test]
    fn test_dll_nelp_range_bounded() {
        // |NELP| ≤ 1 по определению
        let out = epl(10.0, 1.0, 0.0, 0.0);
        let d = out.dll_nelp();

        assert!((1.0..=1.0).contains(&d), "NELP out of range: {d}");
    }

    #[test]
    fn test_dll_nelp_zero_signal_no_panic() {
        let out = epl(0.0, 0.0, 0.0, 0.0);

        assert!((out.dll_nelp() - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_dll_nelp_symmetric() {
        // NELP(E=a, L=b) = -NELP(E=b, L=a)
        let out1 = epl(3.0, 1.0, 0.0, 1.0);
        let out2 = epl(1.0, 1.0, 0.0, 3.0);

        assert!((out1.dll_nelp() + out2.dll_nelp()).abs() < 1e-6);
    }

    #[test]
    fn test_dll_ele_zero_when_equal() {
        let out = epl(2.0, 1.0, 0.0, 2.0);

        assert!(out.dll_ele().abs() < 1e-6);
    }

    #[test]
    fn test_dll_ele_positive_early_stronger() {
        let out = epl(3.0, 1.0, 0.0, 1.0);

        assert!(out.dll_ele() > 0.0);
    }

    #[test]
    fn test_pll_atan2_locked() {
        // Синхронизировано → prompt вещественный положительный → atan2 = 0
        let out = epl(0.0, 10.0, 0.0, 0.0);

        assert!(out.pll_atan2().abs() < 1e-5);
    }

    #[test]
    fn test_pll_atan2_quarter_phase_error() {
        // prompt = 0 + 1j → atan2(1, 0) = π/2
        let out = epl(0.0, 0.0, 1.0, 0.0);

        assert!((out.pll_atan2() - FRAC_PI_2).abs() < 1e-5);
    }

    #[test]
    fn test_pll_atan2_negative_bit() {
        // Навигационный бит = −1 → prompt = −|A| + 0j → atan2 ≈ ±π
        let out = epl(0.0, -10.0, 0.0, 0.0);

        assert!(out.pll_atan2().abs() > 3.0);
    }

    #[test]
    fn test_pll_atan2_range() {
        for i in 0..16 {
            let angle = i as f32 * PI / 8.0;
            let out = EplOutput {
                early: Complex32::default(),
                prompt: Complex32::new(angle.cos(), angle.sin()),
                late: Complex32::default(),
            };
            let d = out.pll_atan2();

            assert!(d > -PI - 1e-5 && d <= PI + 1e-5, "atan2={d}");
        }
    }

    #[test]
    fn test_pll_dd_atan_removes_bit_ambiguity() {
        // DD-atan должен давать ≈ 0 для обоих знаков бита при синхронизации
        let pos = epl(0.0, 10.0, 0.0, 0.0);
        let neg = epl(0.0, -10.0, 0.0, 0.0);

        assert!(pos.pll_dd_atan().abs() < 1e-4, "pos: {}", pos.pll_dd_atan());
        assert!(neg.pll_dd_atan().abs() < 1e-4, "neg: {}", neg.pll_dd_atan());
        // atan2 при негативном бите → ≈ π (ошибочно без DD)
        assert!(neg.pll_atan2().abs() > 3.0);
    }

    #[test]
    fn test_pll_dd_atan_range() {
        for i in 0..16 {
            let angle = (i as f32 - 8.0) * FRAC_PI_2 / 8.0;
            let out = EplOutput {
                early: Complex32::default(),
                prompt: Complex32::new(angle.cos(), angle.sin()),
                late: Complex32::default(),
            };
            let d = out.pll_dd_atan();

            assert!(
                d > -FRAC_PI_2 - 1e-4 && d <= FRAC_PI_2 + 1e-4,
                "dd_atan={d}"
            );
        }
    }

    #[test]
    fn test_prompt_power() {
        let out = epl(0.0, 3.0, 4.0, 0.0); // |P|² = 25

        assert!((out.prompt_power() - 25.0).abs() < 1e-5);
    }

    #[test]
    fn test_prompt_i_q() {
        let out = epl(0.0, 2.5, -1.5, 0.0);

        assert!((out.prompt_i() - 2.5).abs() < 1e-6);
        assert!((out.prompt_q() + 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_pll_atan2_zero_when_locked() {
        let epl = EplOutput {
            early: Complex32::default(),
            prompt: Complex32::new(10.0, 0.0),
            late: Complex32::default(),
        };

        assert!(epl.pll_atan2().abs() < 1e-6);
    }

    #[test]
    fn test_pll_atan2_quadrature() {
        let epl = EplOutput {
            early: Complex32::default(),
            prompt: Complex32::new(0.0, 1.0),
            late: Complex32::default(),
        };

        assert!((epl.pll_atan2() - FRAC_PI_2).abs() < 1e-6);
    }
}
