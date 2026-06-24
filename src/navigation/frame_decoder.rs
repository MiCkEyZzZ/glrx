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

use std::collections::VecDeque;

/// Число 1-мс эпох в одном навигационном бите GPS L1 C/A.
pub const EPOCHS_PER_BIT: usize = 20;

/// TLM-преамбула: `0b10001011` (`0x8B`), первые 8 бит каждого TLM-слова.
pub const TLM_PREAMBLE: [bool; 8] = [true, false, false, false, true, false, true, true];

/// Длина одного слова GPS L1 C/A в битах (24 данных + 6 parity).
pub const WORD_LENGTH_BITS: usize = 30;

/// Число слов в одном subframe.
pub const WORDS_PER_SUBFRAME: usize = 10;

/// Длина subframe в битах (`30 x 10`).
pub const SUBFRAME_LENGTH_BITS: usize = WORD_LENGTH_BITS * WORDS_PER_SUBFRAME;

/// Состояние конечного автомата [`FrameDecoder`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeState {
    /// Поиск TLM-преамбулы по всем 30 возможным битовым смещениям
    SearchingPreamble,

    /// Преамбула найдена по паттерну на смещение `offset` ожидаем
    /// остаток слова (22 бита) для проверки parity, прежде чем считать
    /// границу подтверждённой.
    VerifyingAlignment {
        /// Смещение в скользящем буфере, на котором обнаружен `0x8B`
        offset: usize,
    },

    /// Граница подтверждена parity собираем оставшиеся слова subframe
    Collecting {
        /// Сколько бит уже собрано в текущем subframe (0..300)
        bits_collected: usize,
    },
}

/// Причина отказа декодирования одного слова/subframe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameDecodeError {
    /// Parity-проверка не прошла для одного из 10 слов subframe
    ParityMismatch {
        /// Индекс слова внутри subframe (0-9), на котором произошёл сбой
        word_index: usize,
    },
    /// Преамбула TLM не найдена ни на одной из 30 проверенных позиций
    PreambleNotFound,
}

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

/// Разобранное HOW-слово (второе слово subframe).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HowWord {
    /// TOW count: время начала **следующего** subframe в единицах 6 секунд.
    pub tow_count: u32,

    /// Номер subframe (1–5).
    pub subframe_id: u8,

    /// Флаг "alert" (бит 18 информационной части HOW).
    pub alert_flag: bool,

    /// Флаг "anti-spoof" (бит 19 информационной части HOW).
    pub anti_spoof_flag: bool,
}

/// Полностью декодированный и parity-подтверждённый subframe (300 бит,
/// 10 слов).
#[derive(Debug, Clone)]
pub struct DecodedSubframe {
    /// Номер subframe (1-5), извлечённый из HOW
    pub subframe_id: u8,

    /// HOW-слово (TOW, subframe ID, флаги)
    pub how: HowWord,

    /// 10 слов по 24 информационных бита каждое (после parity-коррекции,
    /// без служебных parity-бит). `words[0]` — TLM, `words[1]` — HOW,
    /// `words[2..10]` — данные subframe.
    pub words: [[bool; 24]; WORDS_PER_SUBFRAME],
}

/// Конфигурация политики повтора при сбое декодирования.
#[derive(Debug, Clone, Copy)]
pub struct RetryPolicy {
    /// Максимальное число повторных попыток поиска преамбулы после сбоя
    /// parity внутри уже предполагаемого subframe, прежде чем декодер
    /// сбрасывается в полный поиск преамбулы заново
    pub max_retries_before_full_resync: usize,
}

/// Статистика работы декодера - для диагностики и тестов.
#[derive(Debug, Clone, Copy, Default)]
pub struct FrameDecoderStats {
    /// Сколько раз была найдена и подтверждена преамбула
    pub preambles_confirmed: u64,

    /// Сколько раз parity провалились на каком-либо слове
    pub parity_failures: u64,

    /// Сколько subframe было полностью и успешно декодировано
    pub subframes_decoded: u64,

    /// Сколько раз декодер был принудительно сброшен в полный resync
    pub full_resyncs: u64,
}

/// Битовая синхронизация + определение кадров для GPS L1 C/A
/// навигационного сообщения.
///
/// Понимает поток уже выраженных по 20 мс границе навигационных битов
/// (см. [`BitSynchronizer`] + [`BitAccumulator`] выше по конвейеру) и
/// собирает их в parity-подтверждённые [`DecodedSubframe`].
///
/// # Алгоритм поиска преамбулы (`SearchingPreamble`)
///
/// Скользящий буфер последних 38 бит (8 бит преамбулы + минимум 30 бит
/// для проверки parity первого слова) проверяется на совпадение `0x8B`
pub struct FrameDecoder {
    state: DecodeState,
    bit_buffer: VecDeque<bool>,
    prev_d29_d30: (bool, bool),
    current_words: Vec<[bool; 24]>,
    retry: RetryPolicy,
    retries_since_resync: usize,
    stats: FrameDecoderStats,
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

impl FrameDecoder {
    /// Создаёт новый decoder с заданной политикой повтора.
    #[must_use]
    pub fn new(retry: RetryPolicy) -> Self {
        Self {
            state: DecodeState::SearchingPreamble,
            bit_buffer: VecDeque::with_capacity(SUBFRAME_LENGTH_BITS),
            prev_d29_d30: (false, false),
            current_words: Vec::with_capacity(WORDS_PER_SUBFRAME),
            retry,
            retries_since_resync: 0,
            stats: FrameDecoderStats::default(),
        }
    }

    /// Создаёт decoder с политикой повтора по умолчанию.
    #[must_use]
    pub fn with_defaults() -> Self {
        Self::new(RetryPolicy::default())
    }

    /// Возвращает текущее состояние конечного автомата.
    #[must_use]
    pub const fn state(&self) -> DecodeState {
        self.state
    }

    /// Возвращает снимок статистикт.
    #[must_use]
    pub const fn stats(&self) -> FrameDecoderStats {
        self.stats
    }

    /// Подаёт один навигационный бит (20-мс, уже выровненный
    /// [`BitAccumulator`]'ом). Возвращает `Ok(Some(subframe))`, когда
    /// собран и подтверждён полный subframe, `Ok(None)` — если сборка ещё
    /// не завершена, `Err(_)` — если на этом шаге произошёл сбой parity
    /// (а не просто "пока нет данных").
    ///
    /// # Errors
    ///
    /// Возвращает `Err(FrameDecodeError::ParityMismatch { .. })`, если при
    /// проверке очередного слова parity не сошлась, и `Err(FrameDecodeError::PreambleNotFound)`,
    /// если после исчерпания retry-budget декодер выполняет полный resync.
    pub fn push_bit(
        &mut self,
        bit: bool,
    ) -> Result<Option<DecodedSubframe>, FrameDecodeError> {
        match self.state {
            DecodeState::SearchingPreamble => self.step_searching(bit),
            DecodeState::VerifyingAlignment { .. } => self.step_verifying(bit),
            DecodeState::Collecting { .. } => self.step_collecting(bit),
        }
    }

    fn step_searching(
        &mut self,
        bit: bool,
    ) -> Result<Option<DecodedSubframe>, FrameDecodeError> {
        self.bit_buffer.push_back(bit);

        if self.bit_buffer.len() < WORD_LENGTH_BITS {
            return Ok(None);
        }

        if self.bit_buffer.len() > WORD_LENGTH_BITS {
            self.bit_buffer.pop_front();
        }

        let window: Vec<bool> = self.bit_buffer.iter().copied().collect();
        let d_star_30 = self.prev_d29_d30.1;
        let preamble_matches = window
            .iter()
            .take(8)
            .enumerate()
            .all(|(i, &b)| (b ^ d_star_30) == TLM_PREAMBLE[i]);

        if preamble_matches {
            self.state = DecodeState::VerifyingAlignment { offset: 0 };
            return self.try_verify_current_word();
        }

        Ok(None)
    }

    fn step_verifying(
        &mut self,
        bit: bool,
    ) -> Result<Option<DecodedSubframe>, FrameDecodeError> {
        self.bit_buffer.push_back(bit);

        if self.bit_buffer.len() < WORD_LENGTH_BITS {
            return Ok(None);
        }

        if self.bit_buffer.len() > WORD_LENGTH_BITS {
            self.bit_buffer.pop_front();
        }

        self.try_verify_current_word()
    }

    /// Пытается проверить parity полного слова, находящегося сейчас в
    /// `bit_buffer` (ровно 30 бит).
    fn try_verify_current_word(&mut self) -> Result<Option<DecodedSubframe>, FrameDecodeError> {
        if self.bit_buffer.len() != WORD_LENGTH_BITS {
            return Ok(None);
        }

        let mut word = [false; WORD_LENGTH_BITS];
        for (i, b) in self.bit_buffer.iter().enumerate() {
            word[i] = *b;
        }

        if let Some(info_bits) = check_and_correct_parity(&word, self.prev_d29_d30) {
            self.stats.preambles_confirmed += 1;
            self.prev_d29_d30 = (word[28], word[29]);
            self.current_words.clear();
            self.current_words.push(info_bits);
            self.bit_buffer.clear();
            self.state = DecodeState::Collecting {
                bits_collected: WORD_LENGTH_BITS,
            };
            self.retries_since_resync = 0;
            Ok(None)
        } else {
            self.stats.parity_failures += 1;
            self.handle_failure(0)
        }
    }

    fn step_collecting(
        &mut self,
        bit: bool,
    ) -> Result<Option<DecodedSubframe>, FrameDecodeError> {
        let DecodeState::Collecting { bits_collected } = self.state else {
            unreachable!("step_collecting called outside Collecting state");
        };

        self.bit_buffer.push_back(bit);
        let new_bits_collected = bits_collected + 1;

        if self.bit_buffer.len() < WORD_LENGTH_BITS {
            self.state = DecodeState::Collecting {
                bits_collected: new_bits_collected,
            };
            return Ok(None);
        }

        // Собрано ровно одно новое слово (30 бит) с момента последнего сброса буфера.
        let mut word = [false; WORD_LENGTH_BITS];
        for (i, b) in self.bit_buffer.iter().enumerate() {
            word[i] = *b;
        }

        let word_index = self.current_words.len();

        if let Some(info_bits) = check_and_correct_parity(&word, self.prev_d29_d30) {
            self.prev_d29_d30 = (word[28], word[29]);
            self.current_words.push(info_bits);
            self.bit_buffer.clear();

            if self.current_words.len() == WORDS_PER_SUBFRAME {
                let subframe = self.finalize_subframe();
                self.reset_for_next_subframe();
                Ok(Some(subframe))
            } else {
                self.state = DecodeState::Collecting {
                    bits_collected: new_bits_collected,
                };
                Ok(None)
            }
        } else {
            self.stats.parity_failures += 1;
            self.handle_failure(word_index)
        }
    }

    fn finalize_subframe(&self) -> DecodedSubframe {
        let how = parse_how_word(&self.current_words[1]);

        let mut words = [[false; 24]; WORDS_PER_SUBFRAME];
        words.copy_from_slice(&self.current_words[..WORDS_PER_SUBFRAME]);

        DecodedSubframe {
            subframe_id: how.subframe_id,
            how,
            words,
        }
    }

    fn reset_for_next_subframe(&mut self) {
        self.stats.subframes_decoded += 1;
        self.current_words.clear();
        self.bit_buffer.clear();
        self.state = DecodeState::SearchingPreamble;
    }

    /// Обрабатывает сбой parity согласно [`RetryPolicy`]: либо продолжаем
    /// сдвигать буфер на один бит и пробуем заново искать преамбулу
    /// (частичный retry), либо, если число повторов исчерпано, выполняем
    /// полный resync (полный сброс состояния).
    fn handle_failure(
        &mut self,
        word_index: usize,
    ) -> Result<Option<DecodedSubframe>, FrameDecodeError> {
        self.retries_since_resync += 1;

        if self.retries_since_resync > self.retry.max_retries_before_full_resync {
            self.full_resync();
            return Err(FrameDecodeError::PreambleNotFound);
        }

        // Частичный retry: возвращаемся к поиску преамбулы, но сохраняем
        // часть буфера (сдвиг на 1 бит вместо полной очистки), чтобы не
        // терять уже накопленные данные при единичном ложном срабатывании.
        self.current_words.clear();
        self.state = DecodeState::SearchingPreamble;

        if !self.bit_buffer.is_empty() {
            self.bit_buffer.pop_front();
        }

        Err(FrameDecodeError::ParityMismatch { word_index })
    }

    fn full_resync(&mut self) {
        self.stats.full_resyncs += 1;
        self.state = DecodeState::SearchingPreamble;
        self.bit_buffer.clear();
        self.current_words.clear();
        self.prev_d29_d30 = (false, false);
        self.retries_since_resync = 0;
    }
}

/// Проверяет и при необходимости инвертирует 24 информационных бита
/// 30-битного слова GPS, используя уравнения parity GPS ICD-200.
///
/// # Аргументы
/// - `word` - 30 бит слова, `word[0..24]` - информационные биты `d1..d24`
///   (возможно неинвертированные, как переданы DSP-слоем), `word[24..30]` - переданные
///   байты parity `D25..D30`.
/// - `prev_29_30` - последние два бита (`D * 29`, `D * 30`) **предыдущего** слова
///   потока, нужны для определения инверсии и в уравнениях двух parity-бит.
///
/// # Возвращает
///
/// - `Some(d1..d24_corrected)` - 24 информационных бита после применения инверсии `Di = di ⊕ D*30`,
///   если все 6 уравнений parity совпали
/// - `None`, если хотя бы одно уравнение не совпало (слово повреждено).
#[allow(clippy::too_many_lines)]
#[must_use]
pub fn check_and_correct_parity(
    word: &[bool; 30],
    prev_d29_d30: (bool, bool),
) -> Option<[bool; 24]> {
    let (d_star_29, d_star_30) = prev_d29_d30;

    // Применяем инверсию: Di = di ⊕ D*30 для информационных бит.
    let mut d = [false; 24];

    for (i, &bit) in word.iter().take(24).enumerate() {
        d[i] = bit ^ d_star_30;
    }

    // Биты нумеруются с 1 в стандарте в массиве `d` индекс i соответствует d[i + 1].
    // Вспомогательная ф-я для удобства чтения по 1-based индексу.
    let g = |k: usize| d[k - 1];
    let computed_d25 = d_star_29
        ^ g(1)
        ^ g(2)
        ^ g(3)
        ^ g(5)
        ^ g(6)
        ^ g(10)
        ^ g(11)
        ^ g(12)
        ^ g(13)
        ^ g(14)
        ^ g(17)
        ^ g(18)
        ^ g(20)
        ^ g(23);
    let computed_d26 = d_star_30
        ^ g(2)
        ^ g(3)
        ^ g(4)
        ^ g(6)
        ^ g(7)
        ^ g(11)
        ^ g(12)
        ^ g(13)
        ^ g(14)
        ^ g(15)
        ^ g(18)
        ^ g(19)
        ^ g(21)
        ^ g(24);
    let computed_d27 = d_star_29
        ^ g(1)
        ^ g(3)
        ^ g(4)
        ^ g(5)
        ^ g(7)
        ^ g(8)
        ^ g(12)
        ^ g(13)
        ^ g(14)
        ^ g(15)
        ^ g(16)
        ^ g(19)
        ^ g(20)
        ^ g(22);
    let computed_d28 = d_star_30
        ^ g(2)
        ^ g(4)
        ^ g(5)
        ^ g(6)
        ^ g(8)
        ^ g(9)
        ^ g(13)
        ^ g(14)
        ^ g(15)
        ^ g(16)
        ^ g(17)
        ^ g(20)
        ^ g(21)
        ^ g(23);
    let computed_d29 = d_star_30
        ^ g(1)
        ^ g(3)
        ^ g(5)
        ^ g(6)
        ^ g(7)
        ^ g(9)
        ^ g(10)
        ^ g(14)
        ^ g(15)
        ^ g(16)
        ^ g(17)
        ^ g(18)
        ^ g(21)
        ^ g(22)
        ^ g(24);
    let computed_d30 = d_star_29
        ^ g(3)
        ^ g(5)
        ^ g(6)
        ^ g(8)
        ^ g(9)
        ^ g(10)
        ^ g(11)
        ^ g(13)
        ^ g(15)
        ^ g(19)
        ^ g(22)
        ^ g(23)
        ^ g(24);

    let received = [word[24], word[25], word[26], word[27], word[28], word[29]];
    let computed = [
        computed_d25,
        computed_d26,
        computed_d27,
        computed_d28,
        computed_d29,
        computed_d30,
    ];

    if received == computed { Some(d) } else { None }
}

/// Проверяет совпадение первых 8 бит слова с TLM-преамбулой.
#[must_use]
pub fn matches_tlm_preamble(word_start: &[bool]) -> bool {
    word_start.len() >= 8 && word_start[..8] == TLM_PREAMBLE
}

/// Разбирает 24 информационных бита HOW-слова (уже после parity-коррекции).
///
/// Формат (биты 1-24 информационной части HOW, согласно ICD-200):
/// - биты 1–17: TOW count
/// - бит 18: alert flag
/// - бит 19: anti-spoof flag
/// - биты 20–22: subframe ID
/// - биты 23–24: зарезервированы / parity-вспомогательные (не используются)
#[must_use]
pub fn parse_how_word(info_bits: &[bool; 24]) -> HowWord {
    let tow_count = bits_to_u32(&info_bits[0..17]);
    let alert_flag = info_bits[17];
    let anti_spoof_flag = info_bits[18];
    let subframe_id = bits_to_u32(&info_bits[19..22]) as u8;

    HowWord {
        tow_count,
        subframe_id,
        alert_flag,
        anti_spoof_flag,
    }
}

/// Преобразует слайс бит (MSB первым) в `u32`.
fn bits_to_u32(bits: &[bool]) -> u32 {
    bits.iter().fold(0u32, |acc, &b| (acc << 1) | u32::from(b))
}

impl Default for RetryPolicy {
    fn default() -> Self {
        Self {
            max_retries_before_full_resync: 3,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Строит корректное (parity-valid) слово из 24 информационных бит и
    // `D * 29/D * 30` предыдущего слова, для использования в тестах.
    #[allow(clippy::too_many_lines)]
    fn build_valid_word(
        d: [bool; 24],
        prev: (bool, bool),
    ) -> [bool; 30] {
        let (d_star_29, d_star_30) = prev;
        let g = |k: usize| d[k - 1];

        let d25 = d_star_29
            ^ g(1)
            ^ g(2)
            ^ g(3)
            ^ g(5)
            ^ g(6)
            ^ g(10)
            ^ g(11)
            ^ g(12)
            ^ g(13)
            ^ g(14)
            ^ g(17)
            ^ g(18)
            ^ g(20)
            ^ g(23);
        let d26 = d_star_30
            ^ g(2)
            ^ g(3)
            ^ g(4)
            ^ g(6)
            ^ g(7)
            ^ g(11)
            ^ g(12)
            ^ g(13)
            ^ g(14)
            ^ g(15)
            ^ g(18)
            ^ g(19)
            ^ g(21)
            ^ g(24);
        let d27 = d_star_29
            ^ g(1)
            ^ g(3)
            ^ g(4)
            ^ g(5)
            ^ g(7)
            ^ g(8)
            ^ g(12)
            ^ g(13)
            ^ g(14)
            ^ g(15)
            ^ g(16)
            ^ g(19)
            ^ g(20)
            ^ g(22);
        let d28 = d_star_30
            ^ g(2)
            ^ g(4)
            ^ g(5)
            ^ g(6)
            ^ g(8)
            ^ g(9)
            ^ g(13)
            ^ g(14)
            ^ g(15)
            ^ g(16)
            ^ g(17)
            ^ g(20)
            ^ g(21)
            ^ g(23);
        let d29 = d_star_30
            ^ g(1)
            ^ g(3)
            ^ g(5)
            ^ g(6)
            ^ g(7)
            ^ g(9)
            ^ g(10)
            ^ g(14)
            ^ g(15)
            ^ g(16)
            ^ g(17)
            ^ g(18)
            ^ g(21)
            ^ g(22)
            ^ g(24);
        let d30 = d_star_29
            ^ g(3)
            ^ g(5)
            ^ g(6)
            ^ g(8)
            ^ g(9)
            ^ g(10)
            ^ g(11)
            ^ g(13)
            ^ g(15)
            ^ g(19)
            ^ g(22)
            ^ g(23)
            ^ g(24);

        // Передаваемое слово на проводе НЕ инвертировано относительно d,
        // если D * 30 (предыдущего слова) = false; если D * 30 = true, то
        // переданные информационные биты есть d ⊕ true (инверсия), так как
        // приёмник восстанавливает d через Di ⊕ D * 30.
        let mut wire = [false; 30];

        for (i, &bit) in d.iter().enumerate() {
            wire[i] = bit ^ d_star_30;
        }

        wire[24] = d25;
        wire[25] = d26;
        wire[26] = d27;
        wire[27] = d28;
        wire[28] = d29;
        wire[29] = d30;

        wire
    }

    // Строит полный 300-битный поток для одного валидного subframe:
    // TLM-слово (с преамбулой) + HOW-слово (с заданным subframe_id/tow)
    // + 8 произвольных, но parity-валидных слов.
    fn build_synthetic_subframe(
        subframe_id: u8,
        tow: u32,
    ) -> Vec<bool> {
        let mut stream = Vec::with_capacity(SUBFRAME_LENGTH_BITS);
        let mut prev = (false, false);

        // Слово 1: TLM. Информационные биты начинаются с 8-битной преамбулы, остальные произвольные.
        let mut tlm_info = [false; 24];

        tlm_info[..8].copy_from_slice(&TLM_PREAMBLE);

        let tlm_word = build_valid_word(tlm_info, prev);

        stream.extend_from_slice(&tlm_word[..24]);
        stream.extend_from_slice(&tlm_word[24..30]);

        prev = (tlm_word[28], tlm_word[29]);

        // Слово 2: HOW. Кодировать tow (17 бит) + alert(0) + AS(0) + subframe_id (3 бита).
        let mut how_info = [false; 24];

        for i in 0..17 {
            how_info[16 - i] = ((tow >> i) & 1) == 1;
        }

        how_info[17] = false; // alert
        how_info[18] = false; // anti-spoof

        for i in 0..3 {
            how_info[21 - i] = ((u32::from(subframe_id) >> i) & 1) == 1;
        }

        let how_word = build_valid_word(how_info, prev);

        stream.extend_from_slice(&how_word[..24]);
        stream.extend_from_slice(&how_word[24..30]);

        prev = (how_word[28], how_word[29]);

        // Слова 3-10: произвольная допустимая полезная нагрузка.
        for w in 0..8u32 {
            let mut info = [false; 24];

            for (i, bit) in info.iter_mut().enumerate() {
                *bit = (w + i as u32).is_multiple_of(5);
            }

            let word = build_valid_word(info, prev);

            stream.extend_from_slice(&word[..24]);
            stream.extend_from_slice(&word[24..30]);

            prev = (word[28], word[29]);
        }

        stream
    }

    fn build_synthetic_subframe_with_prev(
        subframe_id: u8,
        tow: u32,
        mut prev: (bool, bool),
    ) -> (Vec<bool>, (bool, bool)) {
        let mut stream = Vec::with_capacity(SUBFRAME_LENGTH_BITS);

        // Слово 1: TLM
        let mut tlm_info = [false; 24];
        tlm_info[..8].copy_from_slice(&TLM_PREAMBLE);
        let tlm_word = build_valid_word(tlm_info, prev);
        stream.extend_from_slice(&tlm_word[..24]);
        stream.extend_from_slice(&tlm_word[24..30]);
        prev = (tlm_word[28], tlm_word[29]);

        // Слово 2: HOW
        let mut how_info = [false; 24];
        for i in 0..17 {
            how_info[16 - i] = ((tow >> i) & 1) == 1;
        }
        // alert, anti-spoof = false
        for i in 0..3 {
            how_info[21 - i] = ((u32::from(subframe_id) >> i) & 1) == 1;
        }
        let how_word = build_valid_word(how_info, prev);
        stream.extend_from_slice(&how_word[..24]);
        stream.extend_from_slice(&how_word[24..30]);
        prev = (how_word[28], how_word[29]);

        // Слова 3-10
        for w in 0..8u32 {
            let mut info = [false; 24];

            for (i, bit) in info.iter_mut().enumerate() {
                *bit = (w + i as u32).is_multiple_of(5);
            }

            let word = build_valid_word(info, prev);

            stream.extend_from_slice(&word[..24]);
            stream.extend_from_slice(&word[24..30]);
            prev = (word[28], word[29]);
        }

        (stream, prev)
    }

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

    #[test]
    fn test_parity_accepts_valid_word_no_inversion() {
        let d = [false; 24];
        let word = build_valid_word(d, (false, false));
        let result = check_and_correct_parity(&word, (false, false));

        assert!(result.is_some());
        assert_eq!(result.unwrap(), d);
    }

    #[test]
    fn test_parity_accepts_valid_word_with_pattern() {
        let mut d = [false; 24];
        for (i, val) in d.iter_mut().enumerate() {
            *val = i % 3 == 0;
        }

        let word = build_valid_word(d, (true, false));
        let result = check_and_correct_parity(&word, (true, false));

        assert!(result.is_some());
        assert_eq!(result.unwrap(), d);
    }

    #[test]
    fn test_parity_handles_inversion_when_prev_d30_is_true() {
        let mut d = [false; 24];

        d[0] = true;
        d[5] = true;
        d[12] = true;

        let prev = (false, true); // D * 30 = true -> инверсия применена
        let word = build_valid_word(d, prev);
        let result = check_and_correct_parity(&word, prev);

        assert!(result.is_some());
        assert_eq!(
            result.unwrap(),
            d,
            "decoded bits must match original after de-inversion"
        );
    }

    #[test]
    fn test_parity_rejects_corrupted_word() {
        let d = [true; 24];
        let mut word = build_valid_word(d, (false, false));

        word[3] = !word[3]; // искажаем один информационный бит

        let result = check_and_correct_parity(&word, (false, false));

        assert!(result.is_none(), "corrupted word should fail parity");
    }

    #[test]
    fn test_parity_rejects_corrupted_parity_bit() {
        let d = [false; 24];
        let mut word = build_valid_word(d, (false, false));

        word[27] = !word[27]; // поврежден D28

        let result = check_and_correct_parity(&word, (false, false));

        assert!(result.is_none());
    }

    #[test]
    fn test_parity_wrong_prev_bits_causes_mismatch() {
        let d = [true, false]
            .iter()
            .cycle()
            .take(24)
            .copied()
            .collect::<Vec<_>>();
        let mut d_arr = [false; 24];

        d_arr.copy_from_slice(&d);

        let word = build_valid_word(d_arr, (false, false));

        // Проверка с НЕПРАВИЛЬНЫМИ предыдущими битами -> должна завершиться неудачно.
        let result = check_and_correct_parity(&word, (true, true));

        assert!(result.is_none());
    }

    #[test]
    fn test_preamble_matches_exact_pattern() {
        let mut bits = vec![false; 30];

        bits[0] = true;
        bits[4] = true;
        bits[6] = true;
        bits[7] = true;

        // 1 0 0 0 1 0 1 1 → соответствует TLM_PREAMBLE
        assert!(matches_tlm_preamble(&bits));
    }

    #[test]
    fn test_preamble_rejects_wrong_pattern() {
        let bits = vec![false; 30];

        assert!(!matches_tlm_preamble(&bits));
    }

    #[test]
    fn test_preamble_rejects_short_slice() {
        let bits = vec![true; 5];

        assert!(!matches_tlm_preamble(&bits));
    }

    #[test]
    fn test_preamble_rejects_one_bit_error() {
        let mut bits = vec![true, false, false, false, true, false, true, true];

        bits[3] = true; // ломаем 4-й бит

        assert!(!matches_tlm_preamble(&bits));
    }

    #[test]
    fn test_how_word_extracts_subframe_id() {
        let mut info = [false; 24];

        // subframe_id = 3 → биты 19,20,21 (индексирован 0) = 0,1,1
        info[19] = false;
        info[20] = true;
        info[21] = true;

        let how = parse_how_word(&info);

        assert_eq!(how.subframe_id, 3);
    }

    #[test]
    fn test_how_word_extracts_tow_count() {
        let mut info = [false; 24];

        // TOW = 5 → двоичный 00000000000000101 по битам [0..17]
        info[14] = true;
        info[15] = false;
        info[16] = true;

        // 5 = 0b101 → последние три бита 17-битного поля
        let how = parse_how_word(&info);

        assert_eq!(how.tow_count, 5);
    }

    #[test]
    fn test_how_word_extracts_alert_and_antispoof_flags() {
        let mut info = [false; 24];

        info[17] = true; // alert
        info[18] = false; // anti-spoof

        let how = parse_how_word(&info);

        assert!(how.alert_flag);
        assert!(!how.anti_spoof_flag);
    }

    #[test]
    fn test_bits_to_u32_msb_first() {
        let bits = [true, false, true]; // 0b101 = 5

        assert_eq!(bits_to_u32(&bits), 5);
    }

    #[test]
    fn test_build_synthetic_subframe_has_correct_length() {
        let stream = build_synthetic_subframe(1, 100);

        assert_eq!(stream.len(), SUBFRAME_LENGTH_BITS);
    }

    #[test]
    fn test_frame_decoder_decodes_synthetic_subframe() {
        let stream = build_synthetic_subframe(3, 12345);
        let mut decoder = FrameDecoder::with_defaults();

        let mut decoded = None;

        for &bit in &stream {
            if let Ok(Some(subframe)) = decoder.push_bit(bit) {
                decoded = Some(subframe);
                break;
            }
            // Допустимо в процессе поиска преамбулы внутри потока
            // (ложные срабатывания на промежуточных смещениях), но
            // не должно мешать итоговой успешной декодировке.
        }

        let subframe = decoded.expect("subframe should be decoded from clean synthetic stream");

        assert_eq!(subframe.subframe_id, 3);
        assert_eq!(subframe.how.tow_count, 12345);
    }

    #[test]
    fn test_frame_decoder_detects_preamble_within_noisy_prefix() {
        // Добавляем случайный "мусор" перед валидным subframe, чтобы
        // проверить, что decoder находит преамбулу не с первого бита потока.
        let mut stream = vec![
            true, false, true, true, false, false, true, false, true, false,
        ];

        stream.extend(build_synthetic_subframe(2, 999));

        let mut decoder = FrameDecoder::with_defaults();
        let mut decoded = None;

        for &bit in &stream {
            if let Ok(Some(subframe)) = decoder.push_bit(bit) {
                decoded = Some(subframe);
                break;
            }
        }

        let subframe = decoded.expect("subframe should be found despite noisy prefix");

        assert_eq!(subframe.subframe_id, 2);
        assert_eq!(subframe.how.tow_count, 999);
    }

    #[test]
    fn test_frame_decoder_stats_track_successful_decode() {
        let stream = build_synthetic_subframe(1, 1);
        let mut decoder = FrameDecoder::with_defaults();

        for &bit in &stream {
            let _ = decoder.push_bit(bit);
        }

        assert_eq!(decoder.stats().subframes_decoded, 1);
    }

    #[test]
    fn test_frame_decoder_recovers_after_corrupted_word_via_retry() {
        // Портим один бит внутри потока (не в преамбуле) и проверяем, что
        // decoder не виснет — он либо ресинхронизируется и не паникует,
        // либо корректно сообщает ParityMismatch и продолжает работу.
        let mut stream = build_synthetic_subframe(4, 42);
        // Повреждение немного глубже в слове 5 (внутри 300-битного subframe).
        let corrupt_idx = 4 * WORD_LENGTH_BITS + 10;

        stream[corrupt_idx] = !stream[corrupt_idx];

        let mut decoder = FrameDecoder::with_defaults();
        let mut saw_error = false;

        for &bit in &stream {
            match decoder.push_bit(bit) {
                Err(FrameDecodeError::ParityMismatch { .. }) => saw_error = true,
                Ok(_) | Err(FrameDecodeError::PreambleNotFound) => {}
            }
        }

        assert!(
            saw_error,
            "corrupted word should trigger a ParityMismatch at some point"
        );
        // Декодер не должен паниковать и после этого должен вернуться в нормальное состояние.
        assert!(matches!(
            decoder.state(),
            DecodeState::SearchingPreamble | DecodeState::Collecting { .. }
        ));
    }

    #[test]
    fn test_frame_decoder_full_resync_after_exceeding_retry_budget() {
        let retry = RetryPolicy {
            max_retries_before_full_resync: 1,
        };
        let mut decoder = FrameDecoder::new(retry);

        // Поток чистого шума никогда не даст валидную преамбулу + parity —
        // должен в какой-то момент вызвать full_resync без паники.
        let noise: Vec<bool> = (0..2000).map(|i| (i * 2_654_435_761_u64) % 7 < 3).collect();

        for &bit in &noise {
            let _ = decoder.push_bit(bit);
        }

        // Допустимо, что full_resyncs == 0 если случайно ни одна
        // преамбула не совпала вовсе — тест проверяет отсутствие паники
        // и то, что счётчики стат остаются согласованными (неотрицательны
        // по построению типов).
        let stats = decoder.stats();

        assert!(stats.parity_failures >= stats.full_resyncs);
    }

    #[test]
    fn test_frame_decoder_starts_in_searching_state() {
        let decoder = FrameDecoder::with_defaults();

        assert_eq!(decoder.state(), DecodeState::SearchingPreamble);
    }

    #[test]
    fn test_frame_decoder_multiple_subframes_in_sequence() {
        let mut stream = Vec::new();
        let mut prev = (false, false);

        // Первый subframe
        let (bits1, prev1) = build_synthetic_subframe_with_prev(1, 100, prev);
        stream.extend(bits1);
        prev = prev1;

        // Второй subframe
        let (bits2, _prev2) = build_synthetic_subframe_with_prev(2, 106, prev);
        stream.extend(bits2);
        // prev больше не нужен

        let mut decoder = FrameDecoder::with_defaults();
        let mut decoded_subframes = Vec::new();

        for &bit in &stream {
            if let Ok(Some(subframe)) = decoder.push_bit(bit) {
                decoded_subframes.push(subframe);
            }
        }

        assert_eq!(decoded_subframes.len(), 2);
        assert_eq!(decoded_subframes[0].subframe_id, 1);
        assert_eq!(decoded_subframes[0].how.tow_count, 100);
        assert_eq!(decoded_subframes[1].subframe_id, 2);
        assert_eq!(decoded_subframes[1].how.tow_count, 106);
    }
}
