use num_complex::Complex32;

use crate::EplOutput;

/// Коррелирует `signal` с одной репликой кода.
///
/// Возвращает когерентную сумму `Σ signal[n] · code[n]`.
///
/// Оба среза усекаются до `min(signal.len(), code.len())`.
#[inline]
pub fn correlator(
    signal: &[Complex32],
    code: &[f32],
) -> Complex32 {
    signal.iter().zip(code.iter()).map(|(&s, &c)| s * c).sum()
}

/// Коррелятор типа Early-Prompt-Late (EPL).
///
/// Вычисляет три корреляции за один проход по `signal`.
///
/// # Аргументы
///
/// * `signal` — IQ-сэмплы с удалённой несущей за один период интеграции.
/// * `code_early` / `code_prompt` / `code_late` — заранее сгенерированные
///   реплики кода для трёх смещений чипа. Генерируются с помощью
///   [`shift_code`].
pub fn correlator_epl(
    signal: &[Complex32],
    code_early: &[f32],
    code_prompt: &[f32],
    code_late: &[f32],
) -> EplOutput {
    EplOutput {
        early: correlator(signal, code_early),
        prompt: correlator(signal, code_prompt),
        late: correlator(signal, code_late),
    }
}
