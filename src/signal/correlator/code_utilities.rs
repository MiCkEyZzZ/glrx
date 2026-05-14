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
            let c0 = if src_floor >= 0 && (src_floor as usize) < n {
                code[src_floor as usize]
            } else if src_floor < 0 {
                code[0]
            } else {
                code[n - 1]
            };
            let c1_idx = src_floor + 1;
            let c1 = if c1_idx >= 0 && (c1_idx as usize) < n {
                code[c1_idx as usize]
            } else if c1_idx < 0 {
                code[0]
            } else {
                code[n - 1]
            };

            c0 * (1.0 - frac) + c1 * frac
        })
        .collect()
}

////////////////////////////////////////////////////////////////////////////////
// Тесты
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shift_zero_offset() {
        let code = vec![1.0, 2.0, 3.0, 4.0];
        let shifted = shift_code(&code, 0.0);

        assert_eq!(shifted, code);
    }

    #[test]
    fn test_shift_integer_delay() {
        let code = vec![1.0, 2.0, 3.0, 4.0];
        let shifted = shift_code(&code, 1.0);

        assert_eq!(shifted[0], 1.0);
        assert_eq!(shifted[1], 1.0);
        assert_eq!(shifted[2], 2.0);
        assert_eq!(shifted[3], 3.0);
    }

    #[test]
    fn test_shift_fractional() {
        let code = vec![0.0, 10.0];
        let shifted = shift_code(&code, 0.5);

        // интерполяция между 0 и 10
        assert!((shifted[1] - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_shift_negative_offset() {
        let code = vec![1.0, 2.0, 3.0, 4.0];
        let shifted = shift_code(&code, -1.0);

        assert_eq!(shifted[0], 2.0);
        assert_eq!(shifted[1], 3.0);
    }

    #[test]
    fn test_shift_large_offset_clamps() {
        let code = vec![1.0, 2.0, 3.0];
        let shifted = shift_code(&code, 100.0);

        // всё должно прижаться к границе
        assert_eq!(shifted[0], 1.0);
    }
}
