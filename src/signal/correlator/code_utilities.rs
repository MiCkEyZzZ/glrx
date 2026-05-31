//! Утилиты для работы с кодовыми репликами (PRN-кодами).
//!
//! Содержит функции для:
//! - сдвига кодовой последовательности
//! - моделирования дробных задержек
//! - подготовки реплик для корреляции (EPL и др.)
//!
//! Основной сценарий использования — GNSS и DSSS системы,
//! где требуется точная подстройка фазы кода.
//!
//! # Особенности
//!
//! - Поддерживаются дробные смещения (sub-chip resolution)
//! - Используется линейная интерполяция
//! - Граничные значения обрабатываются через clamp (без wrap-around)
//!
//! # Важно
//!
//! Для периодических кодов (например GPS C/A):
//! - здесь **не используется циклический сдвиг**
//! - wrap-around должен реализовываться на уровне более высоких алгоритмов
//!   (например, коррелятора или трекинг-петли)

/// Сдвигает кодовую последовательность на произвольное (в том числе дробное)
/// количество сэмплов.
///
/// # Алгоритм
/// Для каждого выходного индекса `i`:
/// ```text
/// src = i - offset
/// y[i] = lerp(code[floor(src)], code[floor(src)+1])
/// ```
///
/// где `lerp(a, b, t) = a·(1−t) + b·t`.
///
/// # Аргументы
/// - `code` — входная кодовая последовательность
/// - `offset_samples` — смещение в сэмплах (может быть дробным)
///
/// # Поведение на границах
/// Используется **clamp**, а не wrap:
/// - выход за левую границу → `code[0]`
/// - выход за правую границу → `code[n-1]`
///
/// # Примечание
/// Это важно для корректного использования в:
/// - EPL корреляторах
/// - DLL (Delay Lock Loop)
///
/// Для циклических кодов wrap-around должен выполняться отдельно.
#[must_use]
pub fn shift_code(
    code: &[f32],
    offset_samples: f64,
) -> Vec<f32> {
    let n = code.len();
    (0..n)
        .map(|i| {
            let src_f = i as f64 - offset_samples;
            let src_floor_f = src_f.floor();
            let src_floor = src_floor_f as isize;
            let frac = (src_f - src_floor_f) as f32;

            let c0 = if src_floor >= 0 && (src_floor.cast_unsigned()) < n {
                code[src_floor.cast_unsigned()]
            } else if src_floor < 0 {
                code[0]
            } else {
                code[n - 1]
            };

            let c1_idx = src_floor + 1;

            let c1 = if c1_idx >= 0 && c1_idx.cast_unsigned() < n {
                code[c1_idx.cast_unsigned()]
            } else if c1_idx < 0 {
                code[0]
            } else {
                code[n - 1]
            };

            c0 * (1.0 - frac) + c1 * frac
        })
        .collect()
}

/// Подготавливает три кодовые реплики для EPL-коррелятора.
///
/// Обёртка над [`shift_code`], создающая Early, Prompt и Late реплики за один
/// вызов.
///
/// # Аргументы
///
/// * `prompt` - prompt-реплика (базовый код)
/// * `half_chip_samples` - половина чипа в сэмплах (обычно `fs / (2 *
///   chip_rate)`)
///
/// # Возвращает
///
/// `(early, prompt_clone, late)` - три вектора кода
#[must_use]
pub fn make_epl_replicas(
    prompt: &[f32],
    half_chip_samples: f64,
) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
    (
        shift_code(prompt, -half_chip_samples),
        prompt.to_vec(),
        shift_code(prompt, half_chip_samples),
    )
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shift_zero_offset_identity() {
        let code = vec![1.0_f32, -1.0, 1.0, -1.0];
        let shifted = shift_code(&code, 0.0);

        assert_eq!(shifted, code);
    }

    #[test]
    fn test_shift_zero_offset_longer() {
        let code: Vec<f32> = (0..32)
            .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
        let shifted = shift_code(&code, 0.0);

        for (a, b) in code.iter().zip(shifted.iter()) {
            assert!((a - b).abs() < 1e-6);
        }
    }

    #[test]
    fn test_shift_integer_delay_one() {
        // Задержка 1: y[i] = code[i-1], clamp у левой границы
        let code = vec![1.0_f32, 2.0, 3.0, 4.0];
        let shifted = shift_code(&code, 1.0);

        assert!((shifted[0] - 1.0).abs() < 1e-9); // clamp: code[0]
        assert!((shifted[1] - 1.0).abs() < 1e-9); // code[0]
        assert!((shifted[2] - 2.0).abs() < 1e-9); // code[1]
        assert!((shifted[3] - 3.0).abs() < 1e-9); // code[2]
    }

    #[test]
    fn test_shift_integer_advance_one() {
        // Опережение 1: y[i] = code[i+1]
        let code = vec![1.0_f32, 2.0, 3.0, 4.0];
        let shifted = shift_code(&code, -1.0);

        assert!((shifted[0] - 2.0).abs() < 1e-9); // code[1]
        assert!((shifted[1] - 3.0).abs() < 1e-9); // code[2]
        assert!((shifted[2] - 4.0).abs() < 1e-9); // code[3]
        assert!((shifted[3] - 4.0).abs() < 1e-9); // clamp: code[3]
    }

    #[test]
    fn test_shift_integer_delay_two() {
        let code = vec![10.0_f32, 20.0, 30.0, 40.0, 50.0];
        let shifted = shift_code(&code, 2.0);

        assert!((shifted[2] - 10.0).abs() < 1e-9);
        assert!((shifted[3] - 20.0).abs() < 1e-9);
        assert!((shifted[4] - 30.0).abs() < 1e-9);
    }

    #[test]
    fn test_shift_half_sample_interpolates() {
        // offset=0.5: y[i] = lerp(code[i], code[i+1], 0.5)
        let code = vec![0.0_f32, 10.0, 20.0, 30.0];
        let shifted = shift_code(&code, 0.5);

        // y[1] = lerp(code[0], code[1], 0.5) = lerp(0, 10, 0.5) = 5.0
        assert!((shifted[1] - 5.0).abs() < 1e-5, "shifted[1]={}", shifted[1]);
        // y[2] = lerp(code[1], code[2], 0.5) = 15.0
        assert!(
            (shifted[2] - 15.0).abs() < 1e-5,
            "shifted[2]={}",
            shifted[2]
        );
    }

    #[test]
    fn test_shift_quarter_sample_interpolates() {
        let code = vec![0.0_f32, 4.0];
        let shifted = shift_code(&code, 0.25);

        // y[1] = lerp(code[0], code[1], 0.75) = 3.0
        assert!((shifted[1] - 3.0).abs() < 1e-5, "shifted[1]={}", shifted[1]);
    }

    #[test]
    fn test_shift_fractional_advance() {
        let code = vec![0.0_f32, 0.0, 8.0, 0.0];
        // Опережение 0.5: импульс сдвигается на пол-сэмпла влево
        let shifted = shift_code(&code, -0.5);

        // y[1] = lerp(code[1], code[2], 0.5) = lerp(0, 8, 0.5) = 4.0
        assert!((shifted[1] - 4.0).abs() < 1e-5, "shifted[1]={}", shifted[1]);
    }

    #[test]
    fn test_shift_large_delay_clamps_right() {
        let code = vec![1.0_f32, 2.0, 3.0];
        let shifted = shift_code(&code, 100.0);

        // Все сэмплы должны быть code[0] = 1.0 (clamp слева)
        for s in &shifted {
            assert!((s - 1.0).abs() < 1e-6, "s={s}");
        }
    }

    #[test]
    fn test_shift_large_advance_clamps_left() {
        let code = vec![1.0_f32, 2.0, 3.0];
        let shifted = shift_code(&code, -100.0);

        // Все сэмплы должны быть code[2] = 3.0 (clamp справа)
        for s in &shifted {
            assert!((s - 3.0).abs() < 1e-6, "s={s}");
        }
    }

    #[test]
    fn test_shift_single_element_code() {
        let code = vec![42.0_f32];
        let shifted = shift_code(&code, 0.5);

        assert_eq!(shifted.len(), 1);
        assert!((shifted[0] - 42.0).abs() < 1e-6);
    }

    #[test]
    fn test_shift_output_length_matches_input() {
        let code: Vec<f32> = (0..100).map(|i| i as f32).collect();

        for offset in [-10.0, -0.5, 0.0, 0.5, 10.0] {
            let shifted = shift_code(&code, offset);
            assert_eq!(shifted.len(), code.len(), "offset={offset}");
        }
    }

    #[test]
    fn test_bpsk_code_delay_by_half_chip_reduces_correlation() {
        // При задержке на 0.5 чипа корреляция BPSK-кода с самим собой падает
        let code: Vec<f32> = (0..1023)
            .map(|i| if (i * 7 + 3) % 17 < 9 { 1.0 } else { -1.0 })
            .collect();
        let prompt_corr: f32 = code.iter().zip(code.iter()).map(|(a, b)| a * b).sum();
        let shifted = shift_code(&code, 0.5);
        let shifted_corr: f32 = code.iter().zip(shifted.iter()).map(|(a, b)| a * b).sum();

        assert!(
            prompt_corr > shifted_corr.abs(),
            "prompt_corr={prompt_corr} shifted_corr={shifted_corr}"
        );
    }

    #[test]
    fn test_make_epl_replicas_lengths() {
        let prompt: Vec<f32> = vec![1.0; 64];
        let (e, p, l) = make_epl_replicas(&prompt, 0.5);

        assert_eq!(e.len(), 64);
        assert_eq!(p.len(), 64);
        assert_eq!(l.len(), 64);
    }

    #[test]
    fn test_make_epl_replicas_prompt_clone() {
        let prompt = vec![1.0_f32, -1.0, 1.0, -1.0];
        let (_, p, _) = make_epl_replicas(&prompt, 1.0);

        assert_eq!(p, prompt);
    }

    #[test]
    fn test_make_epl_early_is_advanced_late_is_delayed() {
        let code = vec![0.0_f32, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        let (early, prompt, late) = make_epl_replicas(&code, 1.0);

        assert!((prompt[2] - 1.0).abs() < 1e-9);

        // Early = advance -> shift left
        assert!((early[1] - 1.0).abs() < 1e-9);

        // Late = delay -> shift right
        assert!((late[3] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_make_epl_zero_spacing_all_equal() {
        let prompt = vec![1.0_f32, -1.0, 1.0];
        let (e, p, l) = make_epl_replicas(&prompt, 0.0);

        for ((ei, pi), li) in e.iter().zip(p.iter()).zip(l.iter()) {
            assert!((ei - pi).abs() < 1e-9);
            assert!((pi - li).abs() < 1e-9);
        }
    }
}
