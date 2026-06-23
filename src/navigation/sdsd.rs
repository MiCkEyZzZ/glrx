//! Управление временем и синхронизация — receiver-специфичный слой над
//! [`gnss_time`](https://crates.io/crates/gnss-time).
//!
//! GPS-время, TOW и clock bias критичны для точности псевдодальностей: ошибка
//! в 1 мс времени соответствует ~300 км ошибки дальности через скорость
//! света. Этот модуль не реализует шкалы времени и leap-second таблицы
//! заново — для этого используется типобезопасный крейт `gnss_time`
//! (`Time<Gps>`, `Time<Utc>`, `LeapSeconds`). Здесь реализована receiver-
//! специфичная надстройка, которую общая time-арифметика не покрывает:
//!
//! - **sub-millisecond TOW interpolation** — `gnss_time` хранит TOW с
//!   точностью до наносекунды, но не знает, как получить долю миллисекунды
//!   *из кодовой фазы DLL* конкретного канала;
//! - **week rollover detection** — навигационное сообщение GPS передаёт
//!   week number как 10-битное поле (mod 1024); `gnss_time::Time<Gps>::week()`
//!   возвращает полный (uncapped) номер недели и не знает о приёмном
//!   контексте, в котором нужно разворачивать обрезанное поле;
//! - **синхронизация между каналами** — выбор/усреднение единого
//!   референсного момента (common epoch) среди нескольких
//!   `TrackingChannel`, у каждого из которых TOW мог быть декодирован в
//!   разные физические моменты.
//!
//! # Конвейер
//!
//! ```text
//! DecodedSubframe::how.tow_count   (из navigation::frame_decoder, GLRX-10)
//!     │  TOW в единицах 6 с, week mod 1024 (из подкадров 4/5, либо извне)
//!     ▼
//! WeekRollover::resolve_week()     ← разворачивает 10-битное поле
//!     │
//!     ▼
//! Time::<Gps>::from_week_tow(full_week, DurationParts { .. })  (gnss_time)
//!     │
//!     ▼
//! SubMsTowInterpolator::interpolate()   ← добавляет суб-мс точность из DLL
//!     │  Time<Gps> с наносекундной точностью
//!     ▼
//! ChannelEpochSync::common_epoch()      ← согласование нескольких каналов
//!     │
//!     ▼
//! ReceiverTimestamp { gpst, utc, unix_ns }   ← выход для observables/solver
//! ```

use gnss_time::{DurationParts, GnssTimeError, Gps, LeapSeconds, LeapSecondsProvider, Time, Utc};

// ─────────────────────────────────────────────────────────────────────────────
// GPS ↔ UTC: leap-second correction
// ─────────────────────────────────────────────────────────────────────────────

/// Конвертирует GPS-время в UTC, используя встроенную (built-in) таблицу
/// leap-секунд `gnss_time`.
///
/// Тонкая обёртка для единообразного использования внутри receiver-кода:
/// сама конверсия и таблица leap-секунд полностью делегированы крейту.
///
/// # Errors
///
/// Возвращает [`GnssTimeError`], если внутреннее преобразование переполняет
/// диапазон `u64` наносекунд (практически невозможно для реальных GPS-времён).
pub fn gps_to_utc(gps: Time<Gps>) -> Result<Time<Utc>, GnssTimeError> {
    gps.to_utc()
}

/// Конвертирует UTC обратно в GPS-время, используя встроенную таблицу
/// leap-секунд.
///
/// # Errors
///
/// Возвращает [`GnssTimeError`], если преобразование переполняет диапазон.
pub fn utc_to_gps(utc: Time<Utc>) -> Result<Time<Gps>, GnssTimeError> {
    utc.to_gps()
}

/// Конвертирует GPS-время в UTC с явным (например, обновлённым из almanac)
/// провайдером leap-секунд.
///
/// Используйте эту версию, если приёмник получил более новую таблицу
/// leap-секунд (например, через `RuntimeLeapSeconds`), чем зашита в крейт
/// как встроенная.
///
/// # Errors
///
/// Возвращает [`GnssTimeError`], если преобразование переполняет диапазон.
pub fn gps_to_utc_with<P: LeapSecondsProvider>(
    gps: Time<Gps>,
    ls: &P,
) -> Result<Time<Utc>, GnssTimeError> {
    gps.to_utc_with(ls)
}

/// Конвертирует UTC в GPS-время с явным провайдером leap-секунд.
///
/// # Errors
///
/// Возвращает [`GnssTimeError`], если преобразование переполняет диапазон.
pub fn utc_to_gps_with<P: LeapSecondsProvider>(
    utc: Time<Utc>,
    ls: &P,
) -> Result<Time<Gps>, GnssTimeError> {
    utc.to_gps_with(ls)
}

// ─────────────────────────────────────────────────────────────────────────────
// Week rollover detection
// ─────────────────────────────────────────────────────────────────────────────

/// Ширина поля week number в навигационном сообщении GPS L1 C/A (10 бит,
/// значения 0..1023, IS-GPS-200).
pub const GPS_WEEK_FIELD_BITS: u32 = 10;

/// Модуль 10-битного week number, передаваемого в навигационном сообщении.
pub const GPS_WEEK_FIELD_MODULUS: u32 = 1 << GPS_WEEK_FIELD_BITS; // 1024

/// Разворачивает обрезанное (10-битное, mod 1024) week number из
/// навигационного сообщения в полный (uncapped) номер недели, пригодный для
/// [`Time::<Gps>::from_week_tow`].
///
/// GPS передаёт week number только как 10 бит (`0..1023`); каждые ~19.6 лет
/// счётчик переполняется и начинается заново с 0 (последний rollover —
/// 2019-04-06, следующий — 2038-11-21). Чтобы получить корректную полную
/// неделю, приёмник должен знать **приблизительное** текущее время из
/// внешнего источника (системные часы, предыдущий fix, либо встроенная
/// "не раньше чем" дата прошивки) и развернуть переданное поле в ближайшую
/// полную неделю к этому приблизительному времени.
///
/// # Аргументы
///
/// * `transmitted_week_mod_1024` — week number, как передан в навигационном
///   сообщении (`0..=1023`)
/// * `approx_full_week` — приблизительный (заведомо в пределах ±512 недель
///   от истинного значения) полный номер недели, полученный из внешнего
///   источника времени приёмника
///
/// # Возвращает
///
/// Полный номер недели, ближайший к `approx_full_week`, чьё значение по
/// модулю 1024 совпадает с `transmitted_week_mod_1024`.
#[derive(Debug, Clone, Copy)]
pub struct WeekRollover;

impl WeekRollover {
    /// Разворачивает 10-битное week number в полное значение.
    ///
    /// # Panics
    ///
    /// Паникует, если `transmitted_week_mod_1024 >= GPS_WEEK_FIELD_MODULUS`
    /// (то есть передано значение вне диапазона 10-битного поля).
    #[must_use]
    pub fn resolve_week(
        transmitted_week_mod_1024: u32,
        approx_full_week: u32,
    ) -> u32 {
        assert!(
            transmitted_week_mod_1024 < GPS_WEEK_FIELD_MODULUS,
            "transmitted week must fit in 10 bits (0..1023), got {transmitted_week_mod_1024}"
        );

        let approx_mod = approx_full_week % GPS_WEEK_FIELD_MODULUS;
        let approx_cycle_base = approx_full_week - approx_mod;

        // Кандидаты: тот же цикл, на цикл раньше, на цикл позже —
        // выбираем ближайший к approx_full_week.
        let candidates = [
            approx_cycle_base.wrapping_sub(GPS_WEEK_FIELD_MODULUS) + transmitted_week_mod_1024,
            approx_cycle_base + transmitted_week_mod_1024,
            approx_cycle_base + GPS_WEEK_FIELD_MODULUS + transmitted_week_mod_1024,
        ];

        candidates
            .into_iter()
            .min_by_key(|&c| approx_full_week.abs_diff(c))
            .expect("candidates is non-empty")
    }

    /// `true`, если переход от `prev_week_mod_1024` к `curr_week_mod_1024`
    /// (оба — переданные 10-битные значения, наблюдаемые в
    /// последовательных navigation-сообщениях) соответствует rollover
    /// (переполнению с 1023 на 0), а не обычному увеличению на 1.
    ///
    /// Используется для обнаружения rollover "на лету" без необходимости
    /// внешнего приблизительного времени — пригодно, когда приёмник
    /// непрерывно отслеживает последовательные week-значения и хочет
    /// зафиксировать момент перехода.
    #[must_use]
    pub const fn detected_rollover(
        prev_week_mod_1024: u32,
        curr_week_mod_1024: u32,
    ) -> bool {
        // Обычное увеличение: curr == prev + 1 (без переполнения).
        // Rollover: prev == 1023 и curr == 0.
        prev_week_mod_1024 == GPS_WEEK_FIELD_MODULUS - 1 && curr_week_mod_1024 == 0
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Sub-millisecond TOW interpolation
// ─────────────────────────────────────────────────────────────────────────────

/// Интерполирует TOW с точностью лучше 1 мс, используя накопленную фазу
/// кода DLL текущего канала.
///
/// Декодированный из навигационного сообщения TOW count привязан к границе
/// subframe (6-секундный шаг) и сам по себе не несёт суб-миллисекундной
/// точности. Точная суб-миллисекундная позиция внутри текущей 1-мс эпохи
/// определяется накопленной фазой кода DLL (`code_phase_offset_chips`),
/// нормализованной на длину периода кода.
///
/// # Формула
///
/// ```text
/// sub_ms_fraction = code_phase_chips / GPS_CODE_LENGTH_CHIPS
/// tow_correction_ns = sub_ms_fraction × 1_000_000 ns (1 мс = 10⁶ нс)
/// ```
///
/// где `GPS_CODE_LENGTH_CHIPS = 1023` (длина периода GPS L1 C/A в чипах).
#[derive(Debug, Clone, Copy)]
pub struct SubMsTowInterpolator {
    /// Длина периода PRN-кода в чипах (1023 для GPS L1 C/A).
    code_length_chips: f64,
}

impl SubMsTowInterpolator {
    /// Длина периода GPS L1 C/A в чипах.
    pub const GPS_CODE_LENGTH_CHIPS: f64 = 1023.0;

    /// Создаёт интерполятор для заданной длины периода кода (в чипах).
    #[must_use]
    pub const fn new(code_length_chips: f64) -> Self {
        Self { code_length_chips }
    }

    /// Создаёт интерполятор для GPS L1 C/A (1023 чипа).
    #[must_use]
    pub const fn for_gps_l1ca() -> Self {
        Self::new(Self::GPS_CODE_LENGTH_CHIPS)
    }

    /// Вычисляет суб-миллисекундную поправку к TOW (наносекунды, `0..1_000_000`)
    /// из текущей накопленной фазы кода DLL.
    ///
    /// # Аргументы
    ///
    /// * `code_phase_chips` — текущая фаза кода (чипы), нормализована в
    ///   `[0, code_length_chips)` — например,
    ///   `Dll::code_phase_offset_chips()` из `tracking::dll`.
    ///
    /// # Возвращает
    ///
    /// Поправку в наносекундах в диапазоне `[0, 999_999]`, добавляемую к
    /// TOW, выровненному по 1-мс эпохе.
    #[must_use]
    pub fn sub_ms_correction_ns(
        &self,
        code_phase_chips: f64,
    ) -> u32 {
        let fraction = (code_phase_chips / self.code_length_chips).rem_euclid(1.0);
        let ns = fraction * 1_000_000.0;

        // Clamp защищает от пограничных случаев округления (fraction ≈ 1.0
        // из-за погрешности float должно давать 999_999, не 1_000_000).
        (ns as u32).min(999_999)
    }

    /// Строит точное GPS-время с суб-миллисекундной точностью из:
    /// - `whole_ms_tow` — TOW, выровненный по границе 1-мс эпохи (как
    ///   обычно отслеживается tracking-слоем — целое число миллисекунд от
    ///   начала недели);
    /// - `code_phase_chips` — текущая фаза кода для суб-мс интерполяции.
    ///
    /// # Errors
    ///
    /// Возвращает [`GnssTimeError`] при некорректных входных параметрах
    /// (см. [`Time::<Gps>::from_week_tow`]) — например, если `whole_ms_tow`
    /// выходит за пределы недели.
    pub fn interpolate(
        &self,
        week: u16,
        whole_ms_tow: u64,
        code_phase_chips: f64,
    ) -> Result<Time<Gps>, GnssTimeError> {
        let sub_ms_ns = self.sub_ms_correction_ns(code_phase_chips);
        let seconds = whole_ms_tow / 1000;
        let ms_remainder = whole_ms_tow % 1000;
        let nanos = u32::try_from(ms_remainder * 1_000_000).map_err(|_| GnssTimeError::Overflow)?
            + sub_ms_ns;

        let tow = DurationParts::new(seconds, nanos)?;

        Time::<Gps>::from_week_tow(week, tow)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Channel synchronization (common epoch)
// ─────────────────────────────────────────────────────────────────────────────

/// Снимок времени одного канала на момент измерения — вход для
/// синхронизации между каналами.
#[derive(Debug, Clone, Copy)]
pub struct ChannelTimeSample {
    /// PRN канала.
    pub prn: u8,
    /// GPS-время, вычисленное этим каналом (включая суб-мс интерполяцию).
    pub gps_time: Time<Gps>,
    /// Оценка C/N₀ канала (дБ-Гц) — используется как вес при усреднении;
    /// более качественный сигнал должен сильнее влиять на общий epoch.
    pub cn0_db_hz: f32,
}

/// Результат синхронизации нескольких каналов к единому опорному моменту
/// (common epoch).
#[derive(Debug, Clone, Copy)]
pub struct CommonEpoch {
    /// Согласованное GPS-время, общее для всех входных каналов.
    pub gps_time: Time<Gps>,
    /// Максимальное по модулю отклонение времени отдельного канала от
    /// `gps_time` среди входных образцов (наносекунды) — индикатор качества
    /// синхронизации.
    pub max_deviation_ns: u64,
    /// Число каналов, использованных для вычисления.
    pub channel_count: usize,
}

/// Синхронизирует измерения времени от нескольких независимых
/// tracking-каналов к единому common epoch.
///
/// Каждый `TrackingChannel` независимо отслеживает свой спутник и
/// формирует собственную оценку GPS-времени (через декодированный TOW +
/// суб-мс интерполяцию своей DLL). Поскольку каналы физически
/// синхронизированы с одним и тем же приёмным трактом (общие часы
/// дискретизации), их оценки времени должны совпадать в пределах шума
/// измерения; данный механизм агрегирует их в единый момент, взвешивая по
/// качеству сигнала (C/N₀), и сообщает максимальное расхождение как
/// индикатор целостности (большое расхождение — признак сбоя одного из
/// каналов).
#[derive(Debug, Clone, Copy)]
pub struct ChannelEpochSync;

impl ChannelEpochSync {
    /// Вычисляет common epoch из набора измерений каналов.
    ///
    /// Используется взвешенное (по линейному C/N₀, переведённому из дБ)
    /// усреднение наносекундных смещений относительно первого образца —
    /// это устойчиво к большим абсолютным значениям `Time<Gps>` (избегаем
    /// потери точности при усреднении огромных `u64`).
    ///
    /// # Errors
    ///
    /// Возвращает [`GnssTimeError::InvalidInput`], если `samples` пуст.
    pub fn common_epoch(samples: &[ChannelTimeSample]) -> Result<CommonEpoch, GnssTimeError> {
        let Some(reference) = samples.first() else {
            return Err(GnssTimeError::InvalidInput("samples must not be empty"));
        };

        let ref_nanos = reference.gps_time.as_nanos();

        let mut weighted_sum_offset: f64 = 0.0;
        let mut weight_sum: f64 = 0.0;
        let mut max_deviation_ns: u64 = 0;

        for sample in samples {
            let offset_ns = i128::from(sample.gps_time.as_nanos()) - i128::from(ref_nanos);
            let weight = f64::from(10.0_f32.powf(sample.cn0_db_hz / 10.0));

            weighted_sum_offset += offset_ns as f64 * weight;
            weight_sum += weight;

            let abs_offset = offset_ns.unsigned_abs();
            let abs_offset_u64 = u64::try_from(abs_offset).unwrap_or(u64::MAX);
            max_deviation_ns = max_deviation_ns.max(abs_offset_u64);
        }

        let mean_offset_ns = if weight_sum > 0.0 {
            (weighted_sum_offset / weight_sum).round() as i64
        } else {
            0
        };

        let combined_nanos = if mean_offset_ns >= 0 {
            ref_nanos.checked_add(mean_offset_ns.unsigned_abs())
        } else {
            ref_nanos.checked_sub(mean_offset_ns.unsigned_abs())
        }
        .ok_or(GnssTimeError::Overflow)?;

        Ok(CommonEpoch {
            gps_time: Time::<Gps>::from_nanos(combined_nanos),
            max_deviation_ns,
            channel_count: samples.len(),
        })
    }

    /// Доля каналов (от общего числа), чьё измерение расходится с
    /// итоговым `common_epoch.gps_time` более чем на `threshold_ns`.
    ///
    /// Полезно для решения о деаллокации "выпадающих" каналов: если
    /// конкретный канал систематически расходится с консенсусом остальных,
    /// это сигнал о потере lock или ошибке в его собственном tracking.
    #[must_use]
    pub fn outlier_count(
        samples: &[ChannelTimeSample],
        common: &CommonEpoch,
        threshold_ns: u64,
    ) -> usize {
        let ref_nanos = common.gps_time.as_nanos();

        samples
            .iter()
            .filter(|s| {
                let diff = s.gps_time.as_nanos().abs_diff(ref_nanos);
                diff > threshold_ns
            })
            .count()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Output timestamp aggregation
// ─────────────────────────────────────────────────────────────────────────────

/// Итоговый выходной timestamp решения, агрегирующий все три представления,
/// требуемые downstream-потребителями (observables, solver, NMEA/UBX вывод).
#[derive(Debug, Clone, Copy)]
pub struct ReceiverTimestamp {
    /// GPS-время (week/TOW представление доступно через методы `Time<Gps>`).
    pub gpst: Time<Gps>,
    /// UTC-время (leap-second corrected).
    pub utc: Time<Utc>,
    /// Unix-время в наносекундах (для логирования, телеметрии, JSON-вывода).
    pub unix_nanos: i64,
}

impl ReceiverTimestamp {
    /// Строит выходной timestamp из GPS-времени, используя встроенную
    /// таблицу leap-секунд для конверсии в UTC/Unix.
    ///
    /// # Errors
    ///
    /// Возвращает [`GnssTimeError`], если GPS→UTC конверсия не удаётся
    /// (переполнение диапазона).
    pub fn from_gps(gpst: Time<Gps>) -> Result<Self, GnssTimeError> {
        let utc = gpst.to_utc()?;
        let unix_nanos = utc.as_unix_nanos();

        Ok(Self {
            gpst,
            utc,
            unix_nanos,
        })
    }

    /// Строит выходной timestamp из GPS-времени с явным провайдером
    /// leap-секунд (например, обновлённым из almanac во время работы
    /// приёмника).
    ///
    /// # Errors
    ///
    /// Возвращает [`GnssTimeError`], если GPS→UTC конверсия не удаётся.
    pub fn from_gps_with<P: LeapSecondsProvider>(
        gpst: Time<Gps>,
        ls: &P,
    ) -> Result<Self, GnssTimeError> {
        let utc = gpst.to_utc_with(ls)?;
        let unix_nanos = utc.as_unix_nanos();

        Ok(Self {
            gpst,
            utc,
            unix_nanos,
        })
    }

    /// GPS week number (полный, uncapped — см. [`WeekRollover`] для
    /// работы с переданным 10-битным полем).
    #[must_use]
    pub const fn gps_week(&self) -> u32 {
        self.gpst.week()
    }

    /// Время начала недели (TOW) в целых секундах.
    #[must_use]
    pub const fn gps_tow_seconds(&self) -> u32 {
        self.gpst.tow_seconds()
    }

    /// Суб-секундный остаток TOW в наносекундах.
    #[must_use]
    pub const fn gps_tow_sub_second_nanos(&self) -> u32 {
        self.gpst.sub_second_nanos()
    }

    /// ISO 8601 / RFC 3339 представление UTC-момента
    /// (`YYYY-MM-DDTHH:MM:SS.nnnnnnnnnZ`).
    #[must_use]
    pub fn utc_iso8601(&self) -> alloc_free_string::CivilString {
        alloc_free_string::CivilString::from(self.utc.to_civil())
    }

    /// Unix-время в целых секундах (усечение в сторону нуля).
    #[must_use]
    pub const fn unix_seconds(&self) -> i64 {
        self.unix_nanos / 1_000_000_000
    }
}

/// Минимальная no-std-совместимая обёртка для форматирования
/// [`gnss_time::CivilDateTime`] без зависимости от `alloc`/`std::String`,
/// чтобы `timing.rs` оставался пригодным для embedded-сборок receiver-кода.
///
/// `Display` уже реализован у `CivilDateTime` в `gnss_time` — этот тип
/// просто переэкспортирует его в удобном для вызывающего кода виде через
/// `core::fmt::Display`, не выделяя `String` напрямую внутри `timing.rs`.
pub mod alloc_free_string {
    use core::fmt;

    use gnss_time::CivilDateTime;

    /// Обёртка над `CivilDateTime`, реализующая `Display` идентично
    /// исходному типу. Вызывающий код в `std`-окружениях может просто
    /// вызвать `.to_string()` через стандартный `ToString`, доступный по
    /// `Display`.
    #[derive(Debug, Clone, Copy)]
    pub struct CivilString(CivilDateTime);

    impl From<CivilDateTime> for CivilString {
        fn from(dt: CivilDateTime) -> Self {
            Self(dt)
        }
    }

    impl fmt::Display for CivilString {
        fn fmt(
            &self,
            f: &mut fmt::Formatter<'_>,
        ) -> fmt::Result {
            fmt::Display::fmt(&self.0, f)
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── GPS ↔ UTC ──────────────────────────────────────────────────────────────

    #[test]
    fn gps_to_utc_at_gps_epoch_matches_known_offset() {
        let gps = Time::<Gps>::EPOCH;
        let utc = gps_to_utc(gps).unwrap();
        assert_eq!(utc.as_nanos(), 252_892_800_000_000_000);
    }

    #[test]
    fn gps_utc_gps_roundtrip() {
        let gps = Time::<Gps>::from_week_tow(
            2300,
            DurationParts {
                seconds: 100,
                nanos: 0,
            },
        )
        .unwrap();
        let utc = gps_to_utc(gps).unwrap();
        let back = utc_to_gps(utc).unwrap();
        assert_eq!(gps, back);
    }

    #[test]
    fn gps_to_utc_with_explicit_provider_matches_default() {
        let gps = Time::<Gps>::from_seconds(1_000_000_000);
        let ls = LeapSeconds::builtin();
        let via_default = gps_to_utc(gps).unwrap();
        let via_explicit = gps_to_utc_with(gps, ls).unwrap();
        assert_eq!(via_default, via_explicit);
    }

    // ── WeekRollover ──────────────────────────────────────────────────────────

    #[test]
    fn resolve_week_same_cycle_exact_match() {
        // approx_full_week is in the same 1024-cycle as transmitted value.
        let resolved = WeekRollover::resolve_week(300, 2300);
        // 2300 mod 1024 = 252, base = 2048; closest candidate with mod=300
        // should be near 2300.
        assert_eq!(resolved % GPS_WEEK_FIELD_MODULUS, 300);
        assert!(resolved.abs_diff(2300) < GPS_WEEK_FIELD_MODULUS / 2);
    }

    #[test]
    fn resolve_week_handles_rollover_boundary() {
        // Transmitted week 5 should resolve near approx 2300 if the
        // closest full week with mod 1024 == 5 is within half a cycle.
        let resolved = WeekRollover::resolve_week(5, 2300);
        assert_eq!(resolved % GPS_WEEK_FIELD_MODULUS, 5);
        assert!(resolved.abs_diff(2300) <= GPS_WEEK_FIELD_MODULUS / 2 + 1);
    }

    #[test]
    fn resolve_week_zero_approx_zero_transmitted() {
        let resolved = WeekRollover::resolve_week(0, 0);
        assert_eq!(resolved, 0);
    }

    #[test]
    #[should_panic(expected = "must fit in 10 bits")]
    fn resolve_week_rejects_out_of_range_transmitted() {
        let _ = WeekRollover::resolve_week(1024, 2300);
    }

    #[test]
    fn detected_rollover_true_at_boundary() {
        assert!(WeekRollover::detected_rollover(1023, 0));
    }

    #[test]
    fn detected_rollover_false_for_normal_increment() {
        assert!(!WeekRollover::detected_rollover(500, 501));
    }

    #[test]
    fn detected_rollover_false_for_non_adjacent_values() {
        assert!(!WeekRollover::detected_rollover(100, 200));
    }

    // ── SubMsTowInterpolator ──────────────────────────────────────────────────

    #[test]
    fn sub_ms_correction_zero_phase_is_zero() {
        let interp = SubMsTowInterpolator::for_gps_l1ca();
        assert_eq!(interp.sub_ms_correction_ns(0.0), 0);
    }

    #[test]
    fn sub_ms_correction_half_chip_period_is_half_ms() {
        let interp = SubMsTowInterpolator::for_gps_l1ca();
        let half_chips = SubMsTowInterpolator::GPS_CODE_LENGTH_CHIPS / 2.0;
        let ns = interp.sub_ms_correction_ns(half_chips);
        assert!(
            (ns as i64 - 500_000).abs() < 100,
            "expected ~500000 ns, got {ns}"
        );
    }

    #[test]
    fn sub_ms_correction_full_period_wraps_to_zero() {
        let interp = SubMsTowInterpolator::for_gps_l1ca();
        let ns = interp.sub_ms_correction_ns(SubMsTowInterpolator::GPS_CODE_LENGTH_CHIPS);
        assert!(ns < 1000, "full period should wrap near zero, got {ns}");
    }

    #[test]
    fn sub_ms_correction_never_reaches_one_million() {
        let interp = SubMsTowInterpolator::for_gps_l1ca();
        for chips in [0.0, 100.0, 500.0, 1000.0, 1022.999] {
            let ns = interp.sub_ms_correction_ns(chips);
            assert!(ns <= 999_999, "ns={ns} exceeded bound for chips={chips}");
        }
    }

    #[test]
    fn interpolate_produces_valid_gps_time() {
        let interp = SubMsTowInterpolator::for_gps_l1ca();
        let result = interp.interpolate(2300, 5_000, 0.0); // 5 whole ms
        assert!(result.is_ok());
        let t = result.unwrap();
        assert_eq!(t.week(), 2300);
    }

    #[test]
    fn interpolate_adds_sub_ms_correction_to_nanos() {
        let interp = SubMsTowInterpolator::for_gps_l1ca();
        let half_chips = SubMsTowInterpolator::GPS_CODE_LENGTH_CHIPS / 2.0;
        let t = interp.interpolate(100, 1_000, half_chips).unwrap();
        // 1000 ms = 1 s exactly, plus ~500_000 ns sub-ms correction
        assert_eq!(t.tow_seconds(), 1);
        assert!(t.sub_second_nanos() > 400_000 && t.sub_second_nanos() < 600_000);
    }

    #[test]
    fn interpolate_zero_phase_gives_whole_ms_exactly() {
        let interp = SubMsTowInterpolator::for_gps_l1ca();
        let t = interp.interpolate(500, 2_500, 0.0).unwrap();
        assert_eq!(t.tow_seconds(), 2);
        assert_eq!(t.sub_second_nanos(), 500_000_000);
    }

    // ── ChannelEpochSync ──────────────────────────────────────────────────────

    fn sample(
        prn: u8,
        gps_nanos: u64,
        cn0: f32,
    ) -> ChannelTimeSample {
        ChannelTimeSample {
            prn,
            gps_time: Time::<Gps>::from_nanos(gps_nanos),
            cn0_db_hz: cn0,
        }
    }

    #[test]
    fn common_epoch_rejects_empty_input() {
        let result = ChannelEpochSync::common_epoch(&[]);
        assert!(result.is_err());
    }

    #[test]
    fn common_epoch_single_sample_returns_same_time() {
        let samples = [sample(1, 1_000_000_000, 45.0)];
        let result = ChannelEpochSync::common_epoch(&samples).unwrap();
        assert_eq!(result.gps_time.as_nanos(), 1_000_000_000);
        assert_eq!(result.max_deviation_ns, 0);
        assert_eq!(result.channel_count, 1);
    }

    #[test]
    fn common_epoch_identical_samples_zero_deviation() {
        let samples = [
            sample(1, 5_000_000_000, 45.0),
            sample(2, 5_000_000_000, 40.0),
            sample(3, 5_000_000_000, 50.0),
        ];
        let result = ChannelEpochSync::common_epoch(&samples).unwrap();
        assert_eq!(result.gps_time.as_nanos(), 5_000_000_000);
        assert_eq!(result.max_deviation_ns, 0);
    }

    #[test]
    fn common_epoch_averages_small_offsets() {
        let samples = [
            sample(1, 1_000_000_000, 45.0),
            sample(2, 1_000_000_100, 45.0), // +100 ns, same weight
        ];
        let result = ChannelEpochSync::common_epoch(&samples).unwrap();
        // Equal weights → mean offset ≈ 50ns from first sample.
        let expected = 1_000_000_050u64;
        assert!(
            result.gps_time.as_nanos().abs_diff(expected) <= 1,
            "got {}",
            result.gps_time.as_nanos()
        );
    }

    #[test]
    fn common_epoch_max_deviation_reflects_largest_outlier() {
        let samples = [
            sample(1, 1_000_000_000, 45.0),
            sample(2, 1_000_000_000, 45.0),
            sample(3, 1_000_050_000, 45.0), // 50_000 ns away
        ];
        let result = ChannelEpochSync::common_epoch(&samples).unwrap();
        assert!(result.max_deviation_ns > 0);
    }

    #[test]
    fn common_epoch_weights_stronger_signal_more() {
        // Strong-signal channel close to 1_000_000_000, weak-signal channel
        // far away — weighted mean should be pulled toward the strong one.
        let samples = [
            sample(1, 1_000_000_000, 50.0), // strong, weight dominant
            sample(2, 1_001_000_000, 10.0), // weak, 1ms away
        ];
        let result = ChannelEpochSync::common_epoch(&samples).unwrap();
        let dist_to_strong = result.gps_time.as_nanos().abs_diff(1_000_000_000);
        let dist_to_weak = result.gps_time.as_nanos().abs_diff(1_001_000_000);
        assert!(
            dist_to_strong < dist_to_weak,
            "result should be closer to the stronger-signal channel"
        );
    }

    #[test]
    fn outlier_count_zero_when_all_close() {
        let samples = [
            sample(1, 1_000_000_000, 45.0),
            sample(2, 1_000_000_010, 45.0),
        ];
        let common = ChannelEpochSync::common_epoch(&samples).unwrap();
        let outliers = ChannelEpochSync::outlier_count(&samples, &common, 1_000);
        assert_eq!(outliers, 0);
    }

    #[test]
    fn outlier_count_detects_diverging_channel() {
        let samples = [
            sample(1, 1_000_000_000, 45.0),
            sample(2, 1_000_000_000, 45.0),
            sample(3, 1_010_000_000, 45.0), // 10ms away — clear outlier
        ];
        let common = ChannelEpochSync::common_epoch(&samples).unwrap();
        let outliers = ChannelEpochSync::outlier_count(&samples, &common, 1_000_000);
        assert_eq!(outliers, 1);
    }

    // ── ReceiverTimestamp ─────────────────────────────────────────────────────

    #[test]
    fn receiver_timestamp_from_gps_populates_all_fields() {
        let gps = Time::<Gps>::from_week_tow(
            2300,
            DurationParts {
                seconds: 0,
                nanos: 0,
            },
        )
        .unwrap();
        let ts = ReceiverTimestamp::from_gps(gps).unwrap();
        assert_eq!(ts.gpst, gps);
        assert!(ts.unix_nanos > 0);
    }

    #[test]
    fn receiver_timestamp_gps_week_and_tow_accessors() {
        let gps = Time::<Gps>::from_week_tow(
            2345,
            DurationParts {
                seconds: 432_000,
                nanos: 123_000_000,
            },
        )
        .unwrap();
        let ts = ReceiverTimestamp::from_gps(gps).unwrap();
        assert_eq!(ts.gps_week(), 2345);
        assert_eq!(ts.gps_tow_seconds(), 432_000);
        assert_eq!(ts.gps_tow_sub_second_nanos(), 123_000_000);
    }

    #[test]
    fn receiver_timestamp_unix_seconds_matches_utc() {
        let gps = Time::<Gps>::EPOCH;
        let ts = ReceiverTimestamp::from_gps(gps).unwrap();
        assert_eq!(ts.unix_seconds(), ts.utc.as_unix_seconds());
    }

    #[test]
    fn receiver_timestamp_utc_iso8601_contains_t_and_z() {
        let gps = Time::<Gps>::EPOCH;
        let ts = ReceiverTimestamp::from_gps(gps).unwrap();
        let s = ts.utc_iso8601();
        let formatted = format!("{s}");
        assert!(formatted.contains('T'));
        assert!(formatted.ends_with('Z'));
    }

    #[test]
    fn receiver_timestamp_from_gps_with_explicit_provider() {
        let gps = Time::<Gps>::from_seconds(1_000_000_000);
        let ls = LeapSeconds::builtin();
        let ts = ReceiverTimestamp::from_gps_with(gps, ls).unwrap();
        assert_eq!(ts.gpst, gps);
    }

    // ── Integration: full pipeline sanity ────────────────────────────────────

    #[test]
    fn full_pipeline_decoded_tow_to_output_timestamp() {
        // Simulates: decoded week (10-bit) + approx full week (receiver
        // clock) -> resolve -> interpolate sub-ms -> aggregate -> output.
        let transmitted_week_mod = 2300u32 % GPS_WEEK_FIELD_MODULUS;
        let approx_full_week = 2300u32;
        let full_week = WeekRollover::resolve_week(transmitted_week_mod, approx_full_week);
        assert_eq!(full_week, 2300);

        let interp = SubMsTowInterpolator::for_gps_l1ca();
        let gps_time = interp
            .interpolate(u16::try_from(full_week).unwrap(), 86_400_000, 0.0)
            .unwrap();

        let ts = ReceiverTimestamp::from_gps(gps_time).unwrap();
        assert_eq!(ts.gps_week(), 2300);
        assert_eq!(ts.gps_tow_seconds(), 86_400);
    }
}
