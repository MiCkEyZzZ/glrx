//! Модуль корреляторов для обработки сигналов GNSS/DS-SS.
//!
//! Содержит реализацию Early-Prompt-Late (EPL) коррелятора,
//! используемого для:
//! - оценки ошибки синхронизации кода (DLL)
//! - оценки фазы/частоты несущей (PLL/FLL)
//!
//! Основной алгоритм — накопление комплексной корреляции между
//! входным сигналом и локальными репликами псевдослучайного кода.
//!
//! # Термины
//!
//! - Early — опережающая копия кода (−Δτ)
//! - Prompt — синхронная копия (0)
//! - Late — запаздывающая копия (+Δτ)
//!
//! # Примечание
//!
//! Ожидается, что входной сигнал уже:
//! - переведён в базовую полосу (carrier wiped-off)
//! - синхронизирован по частоте
//! - разбит на интервалы интеграции (например, 1 ms для GPS)

use num_complex::Complex32;

use crate::EplOutput;

/// Выполняет EPL-корреляцию за один интервал интеграции.
///
/// # Алгоритм
/// Для каждого сэмпла вычисляется:
/// ```text
/// E += s[n] * code_early[n]
/// P += s[n] * code_prompt[n]
/// L += s[n] * code_late[n]
/// ```
///
/// # Требования
/// - Все входные массивы должны иметь одинаковую длину
/// - `signal` должен быть уже с удалённой несущей
///
/// # Возвращает
/// Комплексные значения корреляции для Early, Prompt и Late каналов
#[must_use]
pub fn correlator_epl(
    signal: &[Complex32],
    code_early: &[f32],
    code_prompt: &[f32],
    code_late: &[f32],
) -> EplOutput {
    let mut e = Complex32::default();
    let mut p = Complex32::default();
    let mut l = Complex32::default();

    for (((&s, &ce), &cp), &cl) in signal
        .iter()
        .zip(code_early)
        .zip(code_prompt)
        .zip(code_late)
    {
        e += s * ce;
        p += s * cp;
        l += s * cl;
    }

    EplOutput {
        early: e,
        prompt: p,
        late: l,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_signal(n: usize) -> Vec<Complex32> {
        vec![Complex32::new(1.0, 0.0); n]
    }

    fn bipolar_code(n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| if i % 2 == 0 { 1.0_f32 } else { -1.0 })
            .collect()
    }

    #[test]
    fn test_epl_with_ones_code_sums_signal() {
        let n = 8;
        let signal: Vec<Complex32> = (0..n).map(|i| Complex32::new(i as f32, 0.0)).collect();
        let code = vec![1.0_f32; n];
        let epl = correlator_epl(&signal, &code, &code, &code);
        // sum(0..8) = 28
        let expected = 28.0_f32;

        assert!((epl.prompt.re - expected).abs() < 1e-5);
        assert!(epl.prompt.im.abs() < 1e-5);
    }

    #[test]
    fn test_epl_bipolar_code_cancels_dc() {
        // Bipolar code с DC-сигналом → корреляция ≈ 0
        let n = 16;
        let signal = unit_signal(n);
        let code = bipolar_code(n);
        let epl = correlator_epl(&signal, &code, &code, &code);

        assert!(epl.prompt.re.abs() < 1e-5, "re={}", epl.prompt.re);
    }

    #[test]
    fn test_epl_self_correlation_is_positive() {
        // Код, скоррелированный сам с собой → положительный вещественный результат
        let n = 64;
        let code = bipolar_code(n);
        let signal: Vec<Complex32> = code.iter().map(|&c| Complex32::new(c, 0.0)).collect();
        let epl = correlator_epl(&signal, &code, &code, &code);

        assert!(epl.prompt.re > 0.0, "prompt.re={}", epl.prompt.re);
    }

    #[test]
    fn test_epl_three_arms_independent() {
        // Early/Prompt/Late должны быть независимы при разных кодах
        let n = 8;
        let signal = unit_signal(n);
        let ce = vec![1.0_f32; n];
        let cp = vec![2.0_f32; n];
        let cl = vec![3.0_f32; n];
        let epl = correlator_epl(&signal, &ce, &cp, &cl);

        assert!((epl.early.re - 8.0).abs() < 1e-5);
        assert!((epl.prompt.re - 16.0).abs() < 1e-5);
        assert!((epl.late.re - 24.0).abs() < 1e-5);
    }

    #[test]
    fn test_epl_complex_signal() {
        // Комплексный сигнал: s = 0+1j, code = 1.0 → result = 0+1j
        let signal = vec![Complex32::new(0.0, 1.0); 4];
        let code = vec![1.0_f32; 4];
        let epl = correlator_epl(&signal, &code, &code, &code);

        assert!(epl.prompt.re.abs() < 1e-5);
        assert!((epl.prompt.im - 4.0).abs() < 1e-5);
    }

    #[test]
    fn test_epl_empty_signal_is_zero() {
        let epl = correlator_epl(&[], &[], &[], &[]);

        assert_eq!(epl.early, Complex32::default());
        assert_eq!(epl.prompt, Complex32::default());
        assert_eq!(epl.late, Complex32::default());
    }

    #[test]
    fn test_epl_truncates_to_shortest() {
        // signal длиннее кода — используется только перекрытие
        let signal: Vec<Complex32> = vec![Complex32::new(1.0, 0.0); 8];
        let code = vec![1.0_f32; 4]; // только 4 элемента
        let epl = correlator_epl(&signal, &code, &code, &code);

        assert!((epl.prompt.re - 4.0).abs() < 1e-5);
    }

    #[test]
    fn test_epl_locked_dll_nelp_zero() {
        // При E = L → NELP = 0
        let n = 32;
        let signal = unit_signal(n);
        let code = vec![1.0_f32; n];
        let epl = correlator_epl(&signal, &code, &code, &code);

        assert!(epl.dll_nelp().abs() < 1e-5);
    }

    #[test]
    fn test_epl_locked_pll_atan2_zero() {
        // Синхронизированный сигнал: prompt должен быть вещественным
        let n = 64;
        let signal = unit_signal(n);
        let code = vec![1.0_f32; n];
        let epl = correlator_epl(&signal, &code, &code, &code);

        assert!(epl.pll_atan2().abs() < 1e-4);
    }

    #[test]
    fn test_epl_prompt_power_matches_sum() {
        let n = 16;
        let signal = unit_signal(n);
        let code = vec![1.0_f32; n];
        let epl = correlator_epl(&signal, &code, &code, &code);

        // prompt = (16+0j) → |P|² = 256
        assert!((epl.prompt_power() - 256.0).abs() < 1e-3);
    }
}
