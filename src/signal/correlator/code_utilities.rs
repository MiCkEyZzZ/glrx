/// Сдвигает реплику кода на `offset_samples` с использованием линейной
/// интерполяции.
///
/// * Положительное `offset_samples` → задержка (фаза увеличивается, реплика
///   приходит позже).
/// * Отрицательное `offset_samples` → опережение (фаза уменьшается, реплика
///   приходит раньше).
///
/// Дробные смещения обрабатываются с помощью линейной интерполяции между
/// соседними чипами. Позиции за пределами массива обрезаются до ближайшей
/// границы.
pub fn shift_code(
    code: &[f32],
    offset_samples: f64,
) -> Vec<f32> {
    let n = code.len();

    (0..n)
        .map(|i| {
            // Нужен code[i − offset], т.е. значение, которое было на позиции
            // (i − offset) в исходном массиве.
            let src_f = i as f64 - offset_samples;
            let src_floor = src_f.floor() as isize;
            let frac = (src_f - src_f.floor()) as f32;

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
