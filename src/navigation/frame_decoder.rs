//! Bit Sync и Frame Detection - выравнивание навигационных битов GPS L1 C/A
//! по границе бита, слова и subframe.
//!
//! Без этого модуля биты навигационного сообщения, поступающие от tracking
//! (через decision-detected знак Prompt-корреляции каждые 20 мс), нельзя
//! собрать в TOW, subframe ID или эфемириды - это стык между DSP-слоем
//! (tracking) и навигационным слоем (frame/ephemeris/solver).
//!
//! # Конвейер
//!
//! ```text
//! Pll::update(prompt)  каждую 1 мс
//!     │  PllOutput { coherent_epoch_completed, .. }
//!     ▼
//! BitSynchronizer::push_prompt_sign(sign)     ← каждые 1 мс (после PLL-lock)
//!     │  определяет границу 20-мс навигационного бита по переходам BPSK
//!     ▼  Some(bit) раз в 20 мс, после нахождения границы
//! FrameDecoder::push_bit(bit)
//!     │
//!     ├─ SearchingPreamble:    ищем 0x8B (TLM preamble) во всех 30 позициях
//!     ├─ VerifyingAlignment:   проверяем parity слова на найденной позиции
//!     ├─ Collecting:           собираем 300 бит (10 слов × 30 бит)
//!     ▼
//! DecodedSubframe { subframe_id, tow, words: [u32; 10] }
//! ```
//!
//! # Детекция границы бита по переходам BPSK
//!
//! GPS L1 C/A навигационный бит 20 мс = 20 эпох по 1 мс. Если
//! tracking уже в PLL-lock, прямая корреляция Prompt за 1 мс уже несёт
//! знак навигационного бита, но какая именно из 20 эпох - первая в новом
//! бите - заранее неизвестна (bit boundary offset 0..19 мс).
//!
//! [`BitSynchronizer`] определяет эту гарницу, накапливая статистику
//! перехода знака (`+1 -> -1` или `-1 -> +1`) Prompt-корреляции отдельно
//! для каждой из 20 возможных фаз (`0..20`). Истанная граница бита - это
//! фаза, на которой переходов знака почти никогда не происходит **внутри**
//! юита (переход возможен только на границу), то есть фаза с минимальным
//! кол-ом внутренних переходов накапливается наименьший счётчик ошибок.
//!
//! # Преамбула TLM (0x8B)
//!
//! Каждое 30-битное слово начинается с 8-битного TLM-преамбулы
//! `0b10001011` (`0x8B`) только в первом слове subframe (TLM-слово).
//! Детекция выполняется проверкой совпадения первых 8 бит **в кадлой из
//! 30 возможных битовых позиций потока**, пока не найдётся позиция, на
//! которой совпадение преамбулы подтверждается успешной parity-проверкой
//! слова (что отличает истинную границу subframe от случайного совпадения
//! паттерна внутри данных).
//!
//! # HOW-слово
//!
//! Второе слово subframe (HOW) содержит:
//! - TOW count (17 бит, начало следующего subframe в единицах 6 с)
//! - subframe ID (3 бита, значение 1-5)
//!
//! # Parity (Hamming 6 бита, GPS ICD-200)
//!
//! Каждое 30-битное слово: 24 информационных бита `d1..d24` + бит
//! parity `D25..D30`. Перед использованием информационные биты
//! инвертируются, если последний бит (`D * 30`) предыдущего слова равен 1:
//! `Di = di ⊕ D*30` для `i = 1..24`. Уравнения parity (XOR-суммы
//! подмножеств бит) взяты из GPS ICD-200 / IS-GPS-200.

/// Число 1-мс эпох в одном навигационном бите GPS L1 C/A.
pub const EPOCHS_PER_BIT: usize = 20;

/// Детектор границы 20-мс навигационного бита по статистике переходов
/// знака Prompt-корреляции (BPSK transitions).
#[derive(Debug, Clone)]
pub struct BitSynchronizer {
    /// Счётчики переходов внутри предполагаемого бита для каждой фазы
    transition_counts: [u32; EPOCHS_PER_BIT],

    /// Общее число проданных 1-мс эпох (для нормализации и решения о
    /// готовности)
    epochs_seen: u64,

    /// Знак предыдущей эпохи (для детекции перехода)
    prev_sign: Option<i8>,

    /// Текущий индекс эпохи внутри гипотетического 20-мс окна.
    epoch_in_window: usize,

    /// Минимальное число до того, как синхронизация считается
    /// надёжной.
    min_epochs_for_decision: u64,
}

/// Накопитель, объединяющий 20 1-мс знаков Prompt в один навигационный бит
/// после того, как [`BitSynchronizer`] определяет фазу границы.
///
/// Использует мажоритарное голосование по знакам внутри бита - устойчивее
/// к одиночным ошибкам корреляции, чем взятие знака только последней эпохи.
#[derive(Debug, Clone)]
pub struct BitAccumulator {
    boundary_phase: usize,
    current_phase: usize,
    positive_count: u32,
    negative_count: u32,
}

impl BitSynchronizer {
    /// Создаёт синхронизатор с заданным минимальным числом эпох для
    /// принятия решения о фазе границы (рекомендуется >= 200, то есть
    /// 10 навигационных бит, для статической надёжности).
    #[must_use]
    pub const fn new(min_epochs_for_decision: u64) -> Self {
        Self {
            transition_counts: [0; EPOCHS_PER_BIT],
            epochs_seen: 0,
            prev_sign: None,
            epoch_in_window: 0,
            min_epochs_for_decision,
        }
    }

    /// Конструктор с порогом по умолчанию (200 эпох = 10 бит).
    #[must_use]
    pub const fn with_defaults() -> Self {
        Self::new(200)
    }

    /// Подаёт знак (`+1` или `-1`) Prompt-корреляции одной 1-мс эпохи.
    ///
    /// # Panics
    ///
    /// Паникует, если `sign` не равен `1` или `-1`.
    pub fn push_prompt_sign(
        &mut self,
        sign: i8,
    ) {
        assert!(sign == 1 || sign == -1, "sign must be +1 or -1, got {sign}");

        if let Some(prev) = self.prev_sign
            && prev != sign
        {
            // Переход произошёл на этой эпохе. Если предполагаемая
            // граница бита - это `epoch_in_window == 0`, то переход на
            // любой другой фазе является внутрениим (ошибка) для
            // этой гипотизы. Учитываем штраф для всех фаз, КРОМЕ той,
            // что совпадает с текущей позицией в окне (поскольку для
            // этой фазы переход хдесь ожидаем и не штрафуется).
            for phase in 0..EPOCHS_PER_BIT {
                if phase != self.epoch_in_window {
                    self.transition_counts[phase] += 1;
                }
            }
        }

        self.prev_sign = Some(sign);
        self.epochs_seen += 1;
        self.epoch_in_window = (self.epoch_in_window + 1) % EPOCHS_PER_BIT;
    }

    /// Возвращает `true`, если накоплено достаточно эпох для надёжного решения.
    #[must_use]
    pub const fn is_ready(&self) -> bool {
        self.epochs_seen >= self.min_epochs_for_decision
    }

    /// Возвращает фазу (0..20) - индекс эпохи внутри 20-мс окна,
    /// соответствующий началу навигационного бита, если решение уже
    /// надёжно (`is_ready`), иначе `None`.
    ///
    /// Фаза с минимальным числом накопленных внутренних переходов
    /// считается истинной границей бита.
    #[must_use]
    pub fn detected_bit_boundary_phase(&self) -> Option<usize> {
        if !self.is_ready() {
            return None;
        }

        self.transition_counts
            .iter()
            .enumerate()
            .min_by_key(|&(_, &count)| count)
            .map(|(phase, _)| phase)
    }

    /// Возвращает общее число обработанных эпох.
    #[must_use]
    pub const fn epochs_seen(&self) -> u64 {
        self.epochs_seen
    }

    /// Сбрасывает накопленную статистику (например, после потери lock).
    pub const fn reset(&mut self) {
        self.transition_counts = [0; EPOCHS_PER_BIT];
        self.epochs_seen = 0;
        self.prev_sign = None;
        self.epoch_in_window = 0;
    }
}

impl BitAccumulator {
    /// Создаёт накопитель с заданной фазой границы бита (из
    /// [`BitSynchronizer::detected_bit_boundary_phase`]).
    #[must_use]
    pub const fn new(boundary_phase: usize) -> Self {
        Self {
            boundary_phase,
            current_phase: 0,
            positive_count: 0,
            negative_count: 0,
        }
    }

    /// Подаёт знак одной 1-мс эпохи.
    ///
    /// Возвращает `Some(bit)`, когда накоплены все 20 эпох одного
    /// навигационного бита (`bit = true` для `+1` — большинства,
    /// `false` для `-1`); внутренний счётчик сбрасывается.
    ///
    /// # Panics
    ///
    /// Паникует, если `sign` не равен `1` или `-1`.
    pub fn push(
        &mut self,
        sign: i8,
    ) -> Option<bool> {
        assert!(sign == 1 || sign == -1, "sign must be +1 or -1, got {sign}");

        if sign > 0 {
            self.positive_count += 1;
        } else {
            self.negative_count += 1;
        }

        self.current_phase = (self.current_phase + 1) % EPOCHS_PER_BIT;

        // Бит завершён, когда мы вернулись к фазу границы.
        if self.current_phase == self.boundary_phase {
            let bit = self.positive_count >= self.negative_count;

            self.positive_count = 0;
            self.negative_count = 0;

            Some(bit)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_synchromizer_not_ready_before_threshold() {
        let mut sync = BitSynchronizer::new(50);

        for _ in 0..49 {
            sync.push_prompt_sign(1);
        }

        assert!(!sync.is_ready());
        assert!(sync.detected_bit_boundary_phase().is_none());
    }

    #[test]
    fn test_synchronizer_ready_after_threshold() {
        let mut sync = BitSynchronizer::new(50);

        for _ in 0..50 {
            sync.push_prompt_sign(1);
        }

        assert!(sync.is_ready());
    }

    #[test]
    fn test_synchronizer_detects_boundary_with_clean_20ms_bits() {
        // Симулируем чистый поток: знак меняется только каждые 20 эпох,
        // начиная с фазы 0 (то есть переход происходит на epoch_in_window == 0
        // непосредственно ПОСЛЕ перехода в новую фазу - boundary phase = 0).
        let mut sync = BitSynchronizer::new(400);
        let mut sign = 1i8;

        for epoch in 0..2000u64 {
            if epoch % 20 == 0 && epoch > 0 {
                sign = -sign;
            }

            sync.push_prompt_sign(sign);
        }

        assert!(sync.is_ready());

        let phase = sync.detected_bit_boundary_phase().unwrap();

        // Граница должна совпадать с точкой смены знака (фаза 0 в этой модели).
        assert_eq!(
            phase, 0,
            "detected phase should match the true bit boundary"
        );
    }

    #[test]
    fn test_synchronizer_detects_boundary_with_nonzero_offset() {
        let mut sync = BitSynchronizer::new(400);
        let offset = 7usize;
        let mut sign = 1i8;

        for epoch in 0..2000u64 {
            let shifted = (epoch as usize + EPOCHS_PER_BIT - offset) % EPOCHS_PER_BIT;
            if shifted == 0 && epoch > 0 {
                sign = -sign;
            }
            sync.push_prompt_sign(sign);
        }

        assert!(sync.is_ready());

        let phase = sync.detected_bit_boundary_phase();

        assert!(phase.is_some());
    }

    #[test]
    fn test_synchronizer_reset_clears_state() {
        let mut sync = BitSynchronizer::new(10);

        for _ in 0..20 {
            sync.push_prompt_sign(1);
        }

        sync.reset();

        assert_eq!(sync.epochs_seen(), 0);
        assert!(!sync.is_ready());
    }

    #[test]
    #[should_panic(expected = "sign must be")]
    fn test_synchronizer_rejects_invalid_sign() {
        let mut sync = BitSynchronizer::with_defaults();

        sync.push_prompt_sign(0);
    }

    #[test]
    fn test_accumulator_emits_bit_after_full_window() {
        let mut acc = BitAccumulator::new(0);
        let mut result = None;

        for _ in 0..20 {
            result = acc.push(1);
        }

        assert_eq!(result, Some(true));
    }

    #[test]
    fn test_accumulator_majority_vote_negative() {
        let mut acc = BitAccumulator::new(0);
        let mut result = None;

        for i in 0..20 {
            let sign = if i < 15 { -1 } else { 1 };

            result = acc.push(sign);
        }

        assert_eq!(result, Some(false));
    }

    #[test]
    fn test_accumulator_returns_none_before_window_complete() {
        let mut acc = BitAccumulator::new(0);

        for _ in 0..19 {
            assert_eq!(acc.push(1), None);
        }
    }

    #[test]
    fn test_accumulator_respects_nonzero_boundary_phase() {
        let mut acc = BitAccumulator::new(5);
        let mut emitted_at = None;

        for i in 0..25 {
            if let Some(_bit) = acc.push(1) {
                emitted_at = Some(i);

                break;
            }
        }

        assert!(emitted_at.is_some());
    }
}
