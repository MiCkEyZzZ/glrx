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
    pub fn prompt_power(&self) -> f32 {
        self.prompt.norm_sqr()
    }

    /// In-phase компонент prompt-ветви (I).
    pub fn prompt_i(&self) -> f32 {
        self.prompt.re
    }

    /// Quadrature компонент prompt-ветви (Q).
    pub fn prompt_q(&self) -> f32 {
        self.prompt.im
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use super::*;

    #[test]
    fn test_dll_nelp_zero_when_equal() {
        let epl = EplOutput {
            early: Complex32::new(1.0, 0.0),
            prompt: Complex32::new(1.0, 0.0),
            late: Complex32::new(1.0, 0.0),
        };

        assert!(epl.dll_nelp().abs() < 1e-6);
    }

    #[test]
    fn test_dll_nelp_positive_when_early_stronger() {
        let epl = EplOutput {
            early: Complex32::new(2.0, 0.0),
            prompt: Complex32::new(1.0, 0.0),
            late: Complex32::new(1.0, 0.0),
        };

        assert!(epl.dll_nelp() > 0.0);
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

    #[test]
    fn test_prompt_power() {
        let epl = EplOutput {
            early: Complex32::default(),
            prompt: Complex32::new(3.0, 4.0),
            late: Complex32::default(),
        };

        assert!((epl.prompt_power() - 25.0).abs() < 1e-6);
    }
}
