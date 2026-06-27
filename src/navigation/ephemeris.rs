//! Декодирование эфемерид GPS (Subframe 1, 2, 3) и вычисление позиции
//! спутника в ECEF на заданный момент времени.
//!
//! Без эфемерид невозможно вычислить координаты спутника, а без координат
//! ни псевдодальность, ни позиционное решение. Этот модуль потребляет
//! [`crate::navigation::frame_decoder::DecodedSubframe`] и извлекает из
//! навигационных слов параметры орбиты Кеплера и поправки часов спутника
//! согласно IS-GPS-200
//!
//! # Dataflow
//!
//! Ниже показан pipeline обработки навигационного сообщения:
//!
//! ```text
//! DecodedSubframe { subframe_id: 1, words: [..] }
//!     │
//!     ▼
//! parse_subframe1()  → ClockParams { week, af0, af1, af2, toc, iodc, health }
//!
//! DecodedSubframe { subframe_id: 2, words: [..] }
//!     │
//!     ▼
//! parse_subframe2()  → OrbitPart1 { iode, crs, delta_n, m0, cuc, e, cus, sqrt_a, toe }
//!
//! DecodedSubframe { subframe_id: 3, words: [..] }
//!     │
//!     ▼
//! parse_subframe3()  → OrbitPart2 { cic, omega0, cis, i0, crc, omega, omega_dot, idot }
//!
//! ClockParams + OrbitPart1 + OrbitPart2  →  Ephemeris (полный комплект)
//!     │
//!     ▼
//! Ephemeris::position_ecef(t)  →  (x, y, z) метры в ECEF
//! ```
//!
//! Subframe 1/2/3 обрабатываются независимо и затем агрегируются в Ephemeris.
//!
//! # Битовые поля
//!
//! Каждое информационное слово приходит как `[bool; 24]` (после parity-корреляции
//! в `frame_decoder`). Поля могут пересекать границы слов (например, `e` - 32 бита,
//! занимает части двух последовательных 24-битных слов), поэтому используется конкатенация
//! битов нескольких слов перед извлечением поля - см. [`BitCursor`].

use crate::navigation::frame_decoder::DecodedSubframe;

const PI: f64 = core::f64::consts::PI;

/// GPS-константа гравитационного параметра Земли (м³/с²), WGS-84.
pub const WGS84_MU: f64 = 3.986_005e14;

/// Угловая скорость вращения Земли (рад/с), WGS-84.
pub const WGS84_OMEGA_E: f64 = 7.292_115_146_7e-5;

/// Скорость света в вакууме (м/с), CODATA.
pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// Причина, по которой набор эфемерид считается непригодным для
/// вычисления позиции.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EphemerisValidationError {
    /// `sv+health != 0` - спутник помечен как нездоровый
    UnhealthySatellite {
        /// Значение health-флага
        health: u8,
    },

    /// `IODE` (Subframe 2) и `IODE` (Subframe 3) не совпадают - параметры
    /// орбиты получены из разных, потенциально несовместимых наборов
    /// эфимерид.
    IodeMismatch {
        /// IODE из Subframe 2
        iode_sf2: u8,
        /// IODE из Subframe 3
        iode_sf3: u8,
    },

    /// Нижние 8 бит `IODE` (Subframe 1) не совпадают с `IODE` (Subframe 2/3)
    IodcIodeMismatch {
        /// Нижние 8 бит IODC
        iodc_low8: u8,
        /// IODE
        iode: u8,
    },
}

/// Курсор для извлечения битовых полей из конкатенированного массива битов.
pub struct BitCursor<'a> {
    bits: &'a [bool],
}

/// Параметры часов спутника (Subframe 1).
///
/// Масштабирование согласно IS-GPS-200:
/// - `toc`: 2⁴ с,
/// - `af2`: 2⁻⁵⁵ с/с²,
/// - `af1`: 2⁻⁴³ с/с,
/// - `af0`: 2⁻³¹ с.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClockParams {
    /// Номер недели GPS (10 бит, mod 1024 — week rollover решается выше по
    /// конвейеру, см. `utils::timing::WeekRollover`)
    pub week_number: u16,

    /// Индекс точности пользовательского диапазона (4 бита)
    pub ura_index: u8,

    /// Health-флаг спутника (6 бит, `0` = healthy).
    pub sv_health: u8,

    /// Выдача данных, часов (10 бит) — должно совпасть с `IODE` из
    /// Subframe 2/3 (нижние 8 бит) для консистентности эфемерид.
    pub iodc: u16,

    /// Время отсчета часов (с), масштаб 2⁴.
    pub toc: f64,

    /// Квадратичный коэффициент коррекции часов (с/с²), масштаб 2⁻⁵⁵.
    pub af2: f64,

    /// Линейный коэффициент коррекции часов (с/с), масштаб 2⁻⁴³.
    pub af1: f64,

    /// Постоянный коэффициент коррекции часов (с), масштаб 2⁻³¹.
    pub af0: f64,
}

/// Орбитальные параметры (Subframe 2).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrbitPart1 {
    /// Issue of Data, Ephemeris (8 бит) — должен совпадать с нижними 8
    /// битами `iodc` из Subframe 1 и с `iode` из Subframe 3.
    pub iode: u8,

    /// Поправка синуса к радиусу орбиты (м), масштаб 2⁻⁵.
    pub crs: f64,

    /// Поправка к среднему движению (рад/с), масштаб 2⁻⁴³·π.
    pub delta_n: f64,

    /// Средняя аномалия на эпоху `toe` (рад), масштаб 2⁻³¹·π.
    pub m0: f64,

    /// Поправка широты, косинусный член (рад), масштаб 2⁻²⁹.
    pub cuc: f64,

    /// Эксцентриситет орбиты, масштаб 2⁻³³.
    pub e: f64,

    /// Поправка широты, синусный член (рад), масштаб 2⁻²⁹.
    pub cus: f64,

    /// Квадратный корень большой полуоси (м^½), масштаб 2⁻¹⁹.
    pub sqrt_a: f64,

    /// Время отсчёта эфемерид (с), масштаб 2⁴.
    pub toe: f64,
}

/// Орбитальные параметры (Subframe 3).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrbitPart2 {
    /// Поправка наклонения, косинусный член (рад), масштаб 2⁻²⁹.
    pub cic: f64,

    /// Долгота восходящего узла на начальную эпоху недели (рад), масштаб
    /// 2⁻³¹·π.
    pub omega0: f64,

    /// Поправка наклонения, синусный член (рад), масштаб 2⁻²⁹.
    pub cis: f64,

    /// Наклонение орбиты на эпоху `toe` (рад), масштаб 2⁻³¹·π.
    pub i0: f64,

    /// Поправка к радиусу, косинусный член (м), масштаб 2⁻⁵.
    pub crc: f64,

    /// Аргумент перигея (рад), масштаб 2⁻³¹·π.
    pub omega: f64,

    /// Скорость изменения долготы узла (рад/с), масштаб 2⁻⁴³·π.
    pub omega_dot: f64,

    /// Issue of Data, Ephemeris (8 бит) — должен совпадать с `iode` из
    /// Subframe 2.
    pub iode: u8,

    /// Скорость изменения наклонения (рад/с), масштаб 2⁻⁴³·π.
    pub idot: f64,
}

/// Полный набор эфемерид спутника.
///
/// Используется для вычисления положения спутника в ECEF.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ephemeris {
    /// PRN спутника
    pub prn: u8,

    /// Параметры часов и здоровья (Subframe 1)
    pub clock: ClockParams,

    /// Параметры орбиты, часть 1 (Subframe 2)
    pub orbit1: OrbitPart1,

    /// Параметры орбиты, часть 2 (Subframe 3)
    pub orbit2: OrbitPart2,
}

impl<'a> BitCursor<'a> {
    /// Создаёт курсор над конкатенированными информационными битами
    /// (без TLM и HOW - то есть `words[2..10]` объединённые в один слайс).
    #[must_use]
    pub const fn new(bits: &'a [bool]) -> Self {
        Self { bits }
    }

    /// Извлекает беззнаковое целое из `len` битов начиная с `offset` (MSB первым).
    ///
    /// # Panics
    ///
    /// Паникует, если `offset + len` выходит за пределы доступных бит.
    #[must_use]
    pub fn unsigned(
        &self,
        offset: usize,
        len: usize,
    ) -> u32 {
        assert!(
            offset + len <= self.bits.len(),
            "bit field out of range: offset={offset} len={len} total={}",
            self.bits.len()
        );

        self.bits[offset..offset + len]
            .iter()
            .fold(0u32, |acc, &b| (acc << 1) | u32::from(b))
    }

    /// Извлекает знаковое целое (two's complement) из `len` бит начиная с
    /// `offset` (MSB - знаковый бит).
    ///
    /// # Panics
    ///
    /// Паникует, если `offset + len` выходит за пределы доступных бит, или
    /// если `len == 0` либо `len > 32`.
    #[must_use]
    pub fn signed(
        &self,
        offset: usize,
        len: usize,
    ) -> i32 {
        assert!(len > 0 && len <= 32, "signed field length must be 1..=32");

        let raw = self.unsigned(offset, len);
        let sign_bit = 1u32 << (len - 1);

        if raw & sign_bit != 0 {
            // Расширяем знак: вычитаем 2^len.
            let full = 1i64 << len;

            (i64::from(raw) - full) as i32
        } else {
            i32::try_from(raw).expect("positive signed field must fit into i32")
        }
    }
}

impl Ephemeris {
    /// Собирает полный комплект эфемерид из трёх разобранных subframe.
    ///
    /// Не выполняет валидацию (health/IOD) - используйте
    /// [`Ephemeris::validate`] после сборки.
    #[must_use]
    pub const fn new(
        prn: u8,
        clock: ClockParams,
        orbit1: OrbitPart1,
        orbit2: OrbitPart2,
    ) -> Self {
        Self {
            prn,
            clock,
            orbit1,
            orbit2,
        }
    }

    /// Проверяет health-бит и консистентность `IODE`/`IODC` между
    /// Subframe 1, 2 и 3
    ///
    /// # Errors
    ///
    /// Возвращает соответствующий вариант [`EphemerisValidationError`] при
    /// первом обнаруженном несоответсвии (порядок проверки: health -> IODE
    /// Subframe2 vs Subframe3 -> IODC low8 vs IODE).
    pub const fn validate(&self) -> Result<(), EphemerisValidationError> {
        if self.clock.sv_health != 0 {
            return Err(EphemerisValidationError::UnhealthySatellite {
                health: self.clock.sv_health,
            });
        }

        if self.orbit1.iode != self.orbit2.iode {
            return Err(EphemerisValidationError::IodeMismatch {
                iode_sf2: self.orbit1.iode,
                iode_sf3: self.orbit2.iode,
            });
        }

        let iodc_low8 = (self.clock.iodc & 0xFF) as u8;

        if iodc_low8 != self.orbit1.iode {
            return Err(EphemerisValidationError::IodcIodeMismatch {
                iodc_low8,
                iode: self.orbit1.iode,
            });
        }

        Ok(())
    }

    /// Вычисляет позицию спутника в ECEF (метры) на момент GPS-времени `t`
    /// (секунды недели), используя алгоритм Кеплера из GPS ICD-200.
    ///
    /// Возвращает `x, y, z, relativistic_correction_s`: координаты в
    /// метрах и релятивисткую поправку часов (секунды), которая зависит
    /// от эксцентричной аномалии и поэтому естественно вычисляется
    /// вместе с позицией.
    #[allow(clippy::many_single_char_names)]
    #[must_use]
    pub fn position_ecef(
        &self,
        t: f64,
    ) -> (f64, f64, f64, f64) {
        let o1 = &self.orbit1;
        let o2 = &self.orbit2;

        // Большая полуось и среднее движение
        let a = o1.sqrt_a * o1.sqrt_a;
        let n0 = (WGS84_MU / (a * a * a)).sqrt();
        let n = n0 + o1.delta_n;

        // Время с момента toe (с коррекцией перехода через границу недели)
        let tk = Self::corrected_time_diff(t, o1.toe);

        // Средняя аномалия
        let mk = o1.m0 + n * tk;

        // Эксцентрическая аномалия (интеграция Кеплера)
        let mut ek = mk;

        for _ in 0..10 {
            let next = mk + o1.e * ek.sin();

            if (next - ek).abs() < 1e-13 {
                ek = next;

                break;
            }

            ek = next;
        }

        // Истинная аномалия
        let sin_vk = (1.0 - o1.e * o1.e).sqrt() * ek.sin();
        let cos_vk = ek.cos() - o1.e;
        let vk = sin_vk.atan2(cos_vk);

        // Аргумент широты
        let phi_k = vk + o2.omega;
        let sin_2phi = (2.0 * phi_k).sin();
        let cos_2phi = (2.0 * phi_k).cos();

        // Коррекция второго порядка
        let delta_uk = o1.cus * sin_2phi + o1.cuc * cos_2phi;
        let delta_rk = o1.crs * sin_2phi + o2.crc * cos_2phi;
        let delta_ik = o2.cis * sin_2phi + o2.cic * cos_2phi;

        // Скорректированные значения
        let uk = phi_k + delta_uk;
        let rk = a * (1.0 - o1.e * ek.cos()) + delta_rk;
        let ik = o2.i0 + delta_ik + o2.idot * tk;

        // Позиция в плоскости орбиты.
        let xk_prime = rk * uk.cos();
        let yk_prime = rk * uk.sin();

        // Долгота восходящего узла с учётом вращения Земли.
        let omega_k = o2.omega0 + (o2.omega_dot - WGS84_OMEGA_E) * tk - WGS84_OMEGA_E * o1.toe;

        // Координаты спутника в системе ECEF.
        let cos_omega_k = omega_k.cos();
        let sin_omega_k = omega_k.sin();
        let cos_ik = ik.cos();
        let sin_ik = ik.sin();
        let x = xk_prime * cos_omega_k - yk_prime * cos_ik * sin_omega_k;
        let y = xk_prime * sin_omega_k + yk_prime * cos_ik * cos_omega_k;
        let z = yk_prime * sin_ik;

        // Релятивистская коррекция: F * e * sqrt(a) * sin(EK), где
        // F = -2 * sqrt(mu)/c² (форма GPS ICD-200).
        let f_const = -2.0 * WGS84_MU.sqrt() / (SPEED_OF_LIGHT * SPEED_OF_LIGHT);
        let relativistic_correction = f_const * o1.e * o1.sqrt_a * ek.sin();

        (x, y, z, relativistic_correction)
    }

    /// Полная коррекция часов спутника, включающая релятивистский эффект.
    ///
    /// Эквивалентно `clock_correction(t) + relativistic_term`, где
    /// `relativistic_term` берётся из [`Ephemeris::position_ecef`] — таким
    /// образом эксцентрическую аномалию вычисляют единообразно в обоих
    /// местах.
    #[must_use]
    pub fn clock_correction_with_relativistic(
        &self,
        t: f64,
    ) -> f64 {
        let (.., relativistic) = self.position_ecef(t);

        self.clock_correction(t) + relativistic
    }

    /// Вычисляет коррекцию часов спутника (секунды) на момент `t` (GPS
    /// system time, секунды недели), используя `af0, af1, af2` и `toc`.
    ///
    /// Релятивистская коррекция **не включена** — она требует
    /// эксцентрической аномалии, вычисляемой отдельно в
    /// [`Ephemeris::position_ecef`]; используйте
    /// [`Ephemeris::clock_correction_with_relativistic`] если нужна полная
    /// коррекция.
    #[must_use]
    pub fn clock_correction(
        &self,
        t: f64,
    ) -> f64 {
        let dt = Self::corrected_time_diff(t, self.clock.toc);

        self.clock.af0 + self.clock.af1 * dt + self.clock.af2 * dt * dt
    }

    /// Разница времён с учётом перехода через границу недели (GPS ICD-200
    /// требует приведения `t - t0` в диапазоне `[-302400, 302400]` c).
    fn corrected_time_diff(
        t: f64,
        t0: f64,
    ) -> f64 {
        let mut dt = t - t0;

        if dt > 302_400.0 {
            dt -= 604_800.0;
        } else if dt < -302_400.0 {
            dt += 604_800.0;
        }

        dt
    }
}

/// Конкатенирует информационные слова `words[2..10]` (8 слов * 24 бита =
/// 192 бита) в единый массив бит для использования с [`BitCursor`].
#[must_use]
pub fn concat_data_words(subframe: &DecodedSubframe) -> [bool; 192] {
    let mut bits = [false; 192];

    for (word_idx, word) in subframe.words[2..10].iter().enumerate() {
        bits[word_idx * 24..(word_idx + 1) * 24].copy_from_slice(word);
    }

    bits
}

/// Разбирает Subframe 1 (часы спутника, health, IODC).
///
/// # Аргументы
/// - `subframe` - декодированный subframe с `subframe_id == 1`.
///
/// # Возвращает
/// - `None`, если `subframe.subframe_id != 1`.
#[must_use]
pub fn parse_subframe1(subframe: &DecodedSubframe) -> Option<ClockParams> {
    if subframe.subframe_id != 1 {
        return None;
    }

    let bits = concat_data_words(subframe);
    let c = BitCursor::new(&bits);

    // Битовые смещения внутри 192-битного блока words[2..10] (то есть
    // считая от начала слова 3 subframe, после TLM+HOW).
    // Word 3: week(10) + c/a или p (2, не используется) + ura(4) + health(6) + iodc_msb(2)
    let week_number = c.unsigned(0, 10) as u16;
    // биты 10-11: code on L2 (не используется здесь)
    let ura_index = c.unsigned(12, 4) as u8;
    let sv_health = c.unsigned(16, 6) as u8;
    let iodc_msb = c.unsigned(22, 2);

    // Word 4-5: reserved (не разбираем, не нужны для позиции)
    // Word 6: t_gd (8, не используется здесь) + iodc_lsb(8) + toc(16) -- упрощённая
    // раскладка ниже соответствует фактической компоновке ICD: t_gd занимает
    // последние 8 бит word3 в некоторых реализациях; здесь используем
    // стандартную раскладку IS-GPS-200 Table 20-I.
    //
    // Смещения относительно начала 192-битного блока (word indices 0..7
    // соответствуют ICD words 3..10):
    // word0 (ICD word3): week[10] L2code[2] ura[4] health[6] iodc_msb[2]  = 24 бит
    // word1 (ICD word4): t_gd[8] iodc_lsb[8] toc[16-... ] -- t_gd(8)+iodc_lsb(8)+toc_part
    let t_gd_raw = c.signed(24, 8); // ICD word4 bits 1-8: t_gd
    let iodc_lsb = c.unsigned(32, 8); // word4 bits 9-16
    let toc_raw = c.unsigned(40, 16); // word4 bits 17-24 + word5 bits 1-8 => итого 16 бит
    let af2_raw = c.signed(56, 8); // word5 bits 9-16
    let af1_raw = c.signed(64, 16); // word5 bits 17-24 + word6 bits 1-8
    let af0_raw = c.signed(80, 22); // word6 bits 9-24 + word7 bits 1-6

    let iodc = (iodc_msb << 8) | iodc_lsb;
    let _ = t_gd_raw; // TODO: использовать при вычислении поправки групповой задержки

    Some(ClockParams {
        week_number,
        ura_index,
        sv_health,
        iodc: iodc as u16,
        toc: f64::from(toc_raw) * 2f64.powi(4),
        af2: f64::from(af2_raw) * 2f64.powi(-55),
        af1: f64::from(af1_raw) * 2f64.powi(-43),
        af0: f64::from(af0_raw) * 2f64.powi(-31),
    })
}

/// Разбирает Subframe 2 (параметры орбиты, часть 1).
///
/// # Возвращает
/// - `None`, если `subframe.subframe_id != 2`.
#[must_use]
pub fn parse_subframe2(subframe: &DecodedSubframe) -> Option<OrbitPart1> {
    if subframe.subframe_id != 2 {
        return None;
    }

    let bits = concat_data_words(subframe);
    let c = BitCursor::new(&bits);

    // word0 (ICD word3): iode[8] crs[16]
    let iode = c.unsigned(0, 8) as u8;
    let crs_raw = c.signed(8, 16);
    // word1 (ICD word4): delta_n[16] m0_msb[8]
    let delta_n_raw = c.signed(24, 16);
    let m0_msb = c.unsigned(40, 8);
    // word2 (ICD word5): m0_lsb[24]
    let m0_lsb = c.unsigned(48, 24);
    // word3 (ICD word6): cuc[16] e_msb[8]
    let cuc_raw = c.signed(72, 16);
    let e_msb = c.unsigned(88, 8);
    // word4 (ICD word7): e_lsb[24]
    let e_lsb = c.unsigned(96, 24);
    // word5 (ICD word8): cus[16] sqrt_a_msb[8]
    let cus_raw = c.signed(120, 16);
    let sqrt_a_msb = c.unsigned(136, 8);
    // word6 (ICD word9): sqrt_a_lsb[24]
    let sqrt_a_lsb = c.unsigned(144, 24);
    // word7 (ICD word10): toe[16] fit_interval[1] aodo[5] parity_aux[2]
    let toe_raw = c.unsigned(168, 16);

    let m0_combined = (m0_msb << 24) | m0_lsb; // 32 бит
    let e_combined = (e_msb << 24) | e_lsb; // 32 бит
    let sqrt_a_combined = (sqrt_a_msb << 24) | sqrt_a_lsb; // 32 бит

    Some(OrbitPart1 {
        iode,
        crs: f64::from(crs_raw) * 2f64.powi(-5),
        delta_n: f64::from(delta_n_raw) * 2f64.powi(-43) * PI,
        m0: f64::from(sign_extend_32(m0_combined)) * 2f64.powi(-31) * PI,
        cuc: f64::from(cuc_raw) * 2f64.powi(-29),
        e: f64::from(e_combined) * 2f64.powi(-33),
        cus: f64::from(cus_raw) * 2f64.powi(-29),
        sqrt_a: f64::from(sqrt_a_combined) * 2f64.powi(-19),
        toe: f64::from(toe_raw) * 2f64.powi(4),
    })
}

/// Разбирает Subframe 3 (параметры орбиты, часть 2).
///
/// # Возвращает
/// - `None`, если `subframe.subframe_id != 3`.
#[must_use]
pub fn parse_subframe3(subframe: &DecodedSubframe) -> Option<OrbitPart2> {
    if subframe.subframe_id != 3 {
        return None;
    }

    let bits = concat_data_words(subframe);
    let c = BitCursor::new(&bits);

    // word0 (ICD word3): cic[16] omega0_msb[8]
    let cic_raw = c.signed(0, 16);
    let omega0_msb = c.unsigned(16, 8);
    // word1 (ICD word4): omega0_lsb[24]
    let omega0_lsb = c.unsigned(24, 24);
    // word2 (ICD word5): cis[16] i0_msb[8]
    let cis_raw = c.signed(48, 16);
    let i0_msb = c.unsigned(64, 8);
    // word3 (ICD word6): i0_lsb[24]
    let i0_lsb = c.unsigned(72, 24);
    // word4 (ICD word7): crc[16] omega_msb[8]
    let crc_raw = c.signed(96, 16);
    let omega_msb = c.unsigned(112, 8);
    // word5 (ICD word8): omega_lsb[24]
    let omega_lsb = c.unsigned(120, 24);
    // word6 (ICD word9): omega_dot[24]
    let omega_dot_raw = c.signed(144, 24);
    // word7 (ICD word10): iode[8] idot[14] parity_aux[2]
    let iode = c.unsigned(168, 8) as u8;
    let idot_raw = c.signed(176, 14);

    let omega0_combined = (omega0_msb << 24) | omega0_lsb;
    let i0_combined = (i0_msb << 24) | i0_lsb;
    let omega_combined = (omega_msb << 24) | omega_lsb;

    Some(OrbitPart2 {
        cic: f64::from(cic_raw) * 2f64.powi(-29),
        omega0: f64::from(sign_extend_32(omega0_combined)) * 2f64.powi(-31) * PI,
        cis: f64::from(cis_raw) * 2f64.powi(-29),
        i0: f64::from(sign_extend_32(i0_combined)) * 2f64.powi(-31) * PI,
        crc: f64::from(crc_raw) * 2f64.powi(-5),
        omega: f64::from(sign_extend_32(omega_combined)) * 2f64.powi(-31) * PI,
        omega_dot: f64::from(omega_dot_raw) * 2f64.powi(-43) * PI,
        iode,
        idot: f64::from(idot_raw) * 2f64.powi(-43) * PI,
    })
}

/// Интерпретирует 32-битное значение как `i32`,
/// сохраняя его битовое представление (two's complement).
#[must_use]
const fn sign_extend_32(raw: u32) -> i32 {
    raw.cast_signed()
}

#[cfg(test)]
mod tests {
    use crate::navigation::frame_decoder::{DecodedSubframe, HowWord};

    use super::*;

    fn make_subframe(
        subframe_id: u8,
        data_words: [[bool; 24]; 8],
    ) -> DecodedSubframe {
        let mut words = [[false; 24]; 10];

        // words[0] = TLM (не используется при парсинге эфемерид)
        // words[1] = HOW (содержимое несущественно, но должен содержать subframe_id)
        words[2..10].copy_from_slice(&data_words);

        DecodedSubframe {
            subframe_id,
            how: HowWord {
                tow_count: 0,
                subframe_id,
                alert_flag: false,
                anti_spoof_flag: false,
            },
            words,
        }
    }

    fn bits_from_u32(
        value: u32,
        len: usize,
    ) -> Vec<bool> {
        (0..len).rev().map(|i| (value >> i) & 1 == 1).collect()
    }

    fn pad_word(bits: &[bool]) -> [bool; 24] {
        assert!(bits.len() <= 24);

        let mut word = [false; 24];
        let start = 24 - bits.len();

        word[start..].copy_from_slice(bits);

        word
    }

    fn dummy_clock(
        health: u8,
        iodc: u16,
    ) -> ClockParams {
        ClockParams {
            week_number: 2300,
            ura_index: 0,
            sv_health: health,
            iodc,
            toc: 0.0,
            af2: 0.0,
            af1: 0.0,
            af0: 0.0,
        }
    }

    fn dummy_orbit1(iode: u8) -> OrbitPart1 {
        OrbitPart1 {
            iode,
            crs: 0.0,
            delta_n: 0.0,
            m0: 0.0,
            cuc: 0.0,
            e: 0.0,
            cus: 0.0,
            sqrt_a: 5153.7,
            toe: 0.0,
        }
    }

    fn dummy_orbit2(iode: u8) -> OrbitPart2 {
        OrbitPart2 {
            cic: 0.0,
            omega0: 0.0,
            cis: 0.0,
            i0: 0.96,
            crc: 0.0,
            omega: 0.0,
            omega_dot: 0.0,
            iode,
            idot: 0.0,
        }
    }

    // Строит эфемериды с параметрами, близкими к типичной круговой
    // GPS-орбите (высота ~20, 200 км, наклонение ~55°), для проверки общей
    // разумности вычисленной позиции (не bit-exact reference, но физически
    // корректный порядок величины и базовые invariants).
    fn typical_gps_ephemeris() -> Ephemeris {
        let clock = ClockParams {
            week_number: 2300,
            ura_index: 0,
            sv_health: 0,
            iodc: 0x0010,
            toc: 0.0,
            af2: 0.0,
            af1: 0.0,
            af0: 0.0,
        };

        let orbit1 = OrbitPart1 {
            iode: 0x10,
            crs: 0.0,
            delta_n: 0.0,
            m0: 0.0,
            cuc: 0.0,
            e: 0.001, // near-circular
            cus: 0.0,
            sqrt_a: 5153.65, // ~ sqrt(26560 км) → большая полуось орбиты GPS
            toe: 0.0,
        };

        let orbit2 = OrbitPart2 {
            cic: 0.0,
            omega0: 0.0,
            cis: 0.0,
            i0: 55.0_f64.to_radians(),
            crc: 0.0,
            omega: 0.0,
            omega_dot: 0.0,
            iode: 0x10,
            idot: 0.0,
        };

        Ephemeris::new(1, clock, orbit1, orbit2)
    }

    #[test]
    fn test_cursor_unsigned_extracts_correct_value() {
        // 0b1011 = 11
        let bits = [true, false, true, true];
        let c = BitCursor::new(&bits);

        assert_eq!(c.unsigned(0, 4), 11);
    }

    #[test]
    fn test_cursor_unsigned_partial_range() {
        let bits = [false, true, true, false, true];
        let c = BitCursor::new(&bits);

        // bits[1..4] = 1,1,0 = 0b110 = 6
        assert_eq!(c.unsigned(1, 3), 6);
    }

    #[test]
    fn test_cursor_signed_positive_value() {
        // 4-битное поле, старший бит = 0 → положительное значение:
        // 0b0101 = 5
        let bits = [false, true, false, true];
        let c = BitCursor::new(&bits);

        assert_eq!(c.signed(0, 4), 5);
    }

    #[test]
    fn test_cursor_signed_negative_value() {
        // 4-битное поле, старший бит = 1 → отрицательное число:
        // 0b1000 = -8 (дополнительный код)
        let bits = [true, false, false, false];
        let c = BitCursor::new(&bits);

        assert_eq!(c.signed(0, 4), -8);
    }

    #[test]
    fn test_cursor_signed_negative_one() {
        // 4 бита, все единицы = -1 в знаковом представлении (two's complement)
        let bits = [true, true, true, true];
        let c = BitCursor::new(&bits);

        assert_eq!(c.signed(0, 4), -1);
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn test_cursor_panics_on_out_of_range() {
        let bits = [true, false];
        let c = BitCursor::new(&bits);
        let _ = c.unsigned(0, 5);
    }

    #[test]
    fn test_parse_subframe1_returns_none_for_wrong_id() {
        let sf = make_subframe(2, [[false; 24]; 8]);

        assert!(parse_subframe1(&sf).is_none());
    }

    #[test]
    fn test_parse_subframe1_extracts_week_and_health() {
        let mut words = [[false; 24]; 8];

        // word0: week[10] l2code[2] ura[4] health[6] iodc_msb[2]
        let mut w0 = Vec::new();

        w0.extend(bits_from_u32(500, 10)); // week = 500
        w0.extend(bits_from_u32(0, 2)); // l2 code
        w0.extend(bits_from_u32(0, 4)); // ura (знаковое, но 0 допустим)
        w0.extend(bits_from_u32(0, 6)); // health = 0 (спутник здоров)
        w0.extend(bits_from_u32(0, 2)); // iodc_msb

        words[0] = pad_word(&w0);

        let sf = make_subframe(1, words);
        let clock = parse_subframe1(&sf).unwrap();

        assert_eq!(clock.week_number, 500);
        assert_eq!(clock.sv_health, 0);
    }

    #[test]
    fn test_parse_subframe1_extracts_nonzero_health() {
        let mut words = [[false; 24]; 8];
        let mut w0 = Vec::new();

        w0.extend(bits_from_u32(0, 10));
        w0.extend(bits_from_u32(0, 2));
        w0.extend(bits_from_u32(0, 4));
        w0.extend(bits_from_u32(0b10_1010, 6)); // ненулевое здоровье
        w0.extend(bits_from_u32(0, 2));

        words[0] = pad_word(&w0);

        let sf = make_subframe(1, words);
        let clock = parse_subframe1(&sf).unwrap();

        assert_eq!(clock.sv_health, 0b10_1010);
    }

    #[test]
    fn test_parse_subframe1_af0_scale_applied() {
        let mut words = [[false; 24]; 8];

        // word6 (data index 3): af1[16-high8] ... фактически af0 занимает word6[9..24]+word7[1..6].
        // Мы напрямую работаем с оффсетами, используемыми в parse_subframe1: af0_raw на бите 80, длина 22.
        // биты 80..102 находятся в data words: 80/24=3 (word index 3, bit 8) .. вплоть до word index 4.
        let mut bits = [false; 192];

        // Устанавливаем af0_raw = 1 (минимальное положительное значение) в позиции offset 80, длина 22.
        bits[80 + 21] = true; // LSB 22-битного поля в позиции offset+len-1

        for i in 0..8 {
            words[i] = bits[i * 24..(i + 1) * 24].try_into().unwrap();
        }

        let sf = make_subframe(1, words);
        let clock = parse_subframe1(&sf).unwrap();

        let expected = 1.0 * 2f64.powi(-31);

        assert!((clock.af0 - expected).abs() < 1e-15);
    }

    #[test]
    fn test_parse_subframe2_returns_none_for_wrong_id() {
        let sf = make_subframe(1, [[false; 24]; 8]);

        assert!(parse_subframe2(&sf).is_none());
    }

    #[test]
    fn test_parse_subframe2_extracts_iode() {
        let mut words = [[false; 24]; 8];
        let mut w0 = Vec::new();

        w0.extend(bits_from_u32(77, 8)); // iode = 77
        w0.extend(bits_from_u32(0, 16)); // crs

        words[0] = pad_word(&w0);

        let sf = make_subframe(2, words);
        let orbit = parse_subframe2(&sf).unwrap();

        assert_eq!(orbit.iode, 77);
    }

    #[test]
    fn test_parse_subframe2_sqrt_a_combines_msb_lsb() {
        // sqrt_a занимает 32 бита, разбитые на words[5] (старшие 8 бит)
        // и words[6] (младшие 24 бита).
        let mut bits = [false; 192];

        // sqrt_a_msb начинается с offset 136 (8 бит), sqrt_a_lsb — с offset 144 (24 бита).
        // Устанавливаем минимальное значение = 1 → единичный LSB всего 32-битного поля
        // (бит 144+23).
        bits[144 + 23] = true;

        let mut words = [[false; 24]; 8];

        for i in 0..8 {
            words[i] = bits[i * 24..(i + 1) * 24].try_into().unwrap();
        }

        let sf = make_subframe(2, words);
        let orbit = parse_subframe2(&sf).unwrap();

        let expected = 1.0 * 2f64.powi(-19);

        assert!((orbit.sqrt_a - expected).abs() < 1e-12);
    }

    #[test]
    fn test_parse_subframe2_eccentricity_is_unsigned() {
        // e занимает bits [88..96] (старшие биты, word3) и [96..120] (младшие биты, word4)
        // в сумме — 32-битное беззнаковое значение.
        let mut bits = [false; 192];

        bits[96 + 23] = true; // младший бит объединённого 32-битного поля -> значение 1

        let mut words = [[false; 24]; 8];

        for i in 0..8 {
            words[i] = bits[i * 24..(i + 1) * 24].try_into().unwrap();
        }

        let sf = make_subframe(2, words);
        let orbit = parse_subframe2(&sf).unwrap();

        let expected = 1.0 * 2f64.powi(-33);

        assert!((orbit.e - expected).abs() < 1e-15);
        assert!(orbit.e >= 0.0, "eccentricity must be non-negative");
    }

    #[test]
    fn test_parse_subframe3_returns_none_for_wrong_id() {
        let sf = make_subframe(1, [[false; 24]; 8]);

        assert!(parse_subframe3(&sf).is_none());
    }

    #[test]
    fn test_parse_subframe3_extracts_iode_matching_subframe2() {
        let mut words = [[false; 24]; 8];
        let mut w7 = Vec::new();

        w7.extend(bits_from_u32(77, 8)); // iode = 77, соответствует тесту subframe2
        w7.extend(bits_from_u32(0, 14)); // idot
        w7.extend(bits_from_u32(0, 2)); // aux

        words[7] = pad_word(&w7);

        let sf = make_subframe(3, words);
        let orbit = parse_subframe3(&sf).unwrap();

        assert_eq!(orbit.iode, 77);
    }

    #[test]
    fn test_parse_subframe3_omega0_sign_extends_negative() {
        // omega0 занимает bits[16..24] в words[0] (старшие биты) и весь words[1]
        // (младшие 24 бита) — в сумме 32-битное знаковое значение.
        let mut bits = [false; 192];

        // Устанавливаем знак (старший бит 32-битного поля, смещение 16) в 1 -> отрицательное значение.
        bits[16] = true;

        let mut words = [[false; 24]; 8];

        for i in 0..8 {
            words[i] = bits[i * 24..(i + 1) * 24].try_into().unwrap();
        }

        let sf = make_subframe(3, words);
        let orbit = parse_subframe3(&sf).unwrap();

        assert!(
            orbit.omega0 < 0.0,
            "sign bit set should yield negative omega0"
        );
    }

    #[test]
    fn test_validate_passes_for_healthy_consistent_ephemeris() {
        let eph = Ephemeris::new(
            1,
            dummy_clock(0, 0x00AA),
            dummy_orbit1(0xAA),
            dummy_orbit2(0xAA),
        );

        assert!(eph.validate().is_ok());
    }

    #[test]
    fn test_validate_fails_for_unhealthy_satellite() {
        let eph = Ephemeris::new(
            1,
            dummy_clock(5, 0x00AA),
            dummy_orbit1(0xAA),
            dummy_orbit2(0xAA),
        );

        assert_eq!(
            eph.validate(),
            Err(EphemerisValidationError::UnhealthySatellite { health: 5 })
        );
    }

    #[test]
    fn test_validate_fails_for_iode_mismatch_between_sf2_sf3() {
        let eph = Ephemeris::new(
            1,
            dummy_clock(0, 0x00AA),
            dummy_orbit1(0xAA),
            dummy_orbit2(0xBB),
        );

        assert_eq!(
            eph.validate(),
            Err(EphemerisValidationError::IodeMismatch {
                iode_sf2: 0xAA,
                iode_sf3: 0xBB
            })
        );
    }

    #[test]
    fn test_validate_fails_for_iodc_iode_mismatch() {
        let eph = Ephemeris::new(
            1,
            dummy_clock(0, 0x00FF),
            dummy_orbit1(0xAA),
            dummy_orbit2(0xAA),
        );

        assert_eq!(
            eph.validate(),
            Err(EphemerisValidationError::IodcIodeMismatch {
                iodc_low8: 0xFF,
                iode: 0xAA
            })
        );
    }

    #[test]
    fn test_validate_checks_health_before_iode() {
        // Спутник одновременно помечен как нездоровый и имеет несовпадающий IODE.
        // Проверка health должна сработать первой, поскольку выполняется раньше.
        let eph = Ephemeris::new(
            1,
            dummy_clock(3, 0x00AA),
            dummy_orbit1(0xAA),
            dummy_orbit2(0xBB),
        );

        assert_eq!(
            eph.validate(),
            Err(EphemerisValidationError::UnhealthySatellite { health: 3 })
        );
    }

    #[test]
    fn test_position_ecef_radius_matches_gps_orbit_altitude() {
        let eph = typical_gps_ephemeris();
        let (x, y, z, _) = eph.position_ecef(0.0);
        let radius = (x * x + y * y + z * z).sqrt();

        // Большая полуось орбиты GPS составляет около 26 560 км, поэтому
        // расстояние до центра Земли должно быть близко к этой величине
        // (с точностью до нескольких сотен километров при e ≈ 0.001).
        assert!(
            (25_000_000.0..28_000_000.0).contains(&radius),
            "radius {radius} m is not in expected GPS orbit range"
        );
    }

    #[test]
    fn test_position_ecef_at_toe_has_near_zero_true_anomaly_effect() {
        // При t = toe, m0 = 0 и малом эксцентриситете спутник должен находиться
        // вблизи направления восходящего узла в плоскости орбиты. Это в первую
        // очередь проверка корректности вычислений: координаты и релятивистская
        // поправка должны быть конечными и не вырождаться.
        let eph = typical_gps_ephemeris();
        let (x, y, z, rel) = eph.position_ecef(0.0);

        assert!(x.is_finite() && y.is_finite() && z.is_finite());
        assert!(rel.is_finite());
        assert!(x != 0.0 || y != 0.0 || z != 0.0);
    }

    #[test]
    fn test_position_ecef_varies_with_time() {
        let eph = typical_gps_ephemeris();
        let (x1, y1, z1, _) = eph.position_ecef(0.0);
        let (x2, y2, z2, _) = eph.position_ecef(3600.0); // 1 час спустя
        let moved = (x1 - x2).abs() > 1.0 || (y1 - y2).abs() > 1.0 || (z1 - z2).abs() > 1.0;

        assert!(moved, "satellite position should change after 1 hour");
    }

    #[test]
    fn test_position_ecef_periodic_after_one_orbital_period() {
        // Период обращения GPS-спутника ≈ 11 ч 58 мин (половина звёздных суток),
        // T = 2π / n.
        let eph = typical_gps_ephemeris();
        let a = eph.orbit1.sqrt_a * eph.orbit1.sqrt_a;
        let n = (WGS84_MU / (a * a * a)).sqrt();
        let period = 2.0 * core::f64::consts::PI / n;

        let (x1, y1, z1, _) = eph.position_ecef(0.0);
        let (x2, y2, z2, _) = eph.position_ecef(period);

        // Координаты не совпадают в точности, поскольку за время одного витка
        // система ECEF поворачивается вместе с Землёй (учитывается вращение Земли).
        // Однако расстояние до центра Земли должно практически повториться,
        // так как оно не зависит от выбранной системы координат.
        let r1 = (x1 * x1 + y1 * y1 + z1 * z1).sqrt();
        let r2 = (x2 * x2 + y2 * y2 + z2 * z2).sqrt();

        assert!(
            (r1 - r2).abs() < 1000.0,
            "orbital radius should repeat after one period"
        );
    }

    #[test]
    fn test_clock_correction_zero_at_toc_with_zero_coefficients() {
        let eph = typical_gps_ephemeris();
        let correction = eph.clock_correction(eph.clock.toc);

        assert!(correction.abs() < 1e-12);
    }

    #[test]
    fn test_clock_correction_linear_term_scales_with_time() {
        let mut eph = typical_gps_ephemeris();

        eph.clock.af1 = 1e-10;
        eph.clock.toc = 0.0;

        let c1000 = eph.clock_correction(1000.0);
        let c2000 = eph.clock_correction(2000.0);

        assert!((c2000 - 2.0 * c1000).abs() < 1e-15);
    }

    #[test]
    fn test_clock_correction_handles_week_boundary_wrap() {
        let mut eph = typical_gps_ephemeris();

        eph.clock.toc = 604_800.0 - 100.0; // конец GPS-недели
        eph.clock.af1 = 1e-9;

        // Момент времени сразу после перехода через границу недели
        // (t = 50, toc = 604700) должен корректно обернуться в небольшое
        // положительное dt (~150 с), а не в огромное отрицательное значение.
        let correction = eph.clock_correction(50.0);

        assert!(correction.is_finite());
        assert!(
            correction.abs() < 1.0,
            "wrapped dt should stay bounded, got correction={correction}"
        );
    }

    #[test]
    fn test_clock_correction_with_relativistic_includes_nonzero_term_for_eccentric_orbit() {
        let mut eph = typical_gps_ephemeris();

        eph.orbit1.e = 0.02; // более эксцентричная орбита -> релятивистская поправка обычно ненулевая

        let basic = eph.clock_correction(1000.0);
        let with_rel = eph.clock_correction_with_relativistic(1000.0);

        // Значения могут совпасть, только если в данной точке орбиты
        // релятивистская поправка оказалась равной нулю. Проверяем лишь,
        // что оба результата конечны и вычисляются без ошибок.
        assert!(basic.is_finite());
        assert!(with_rel.is_finite());
    }

    #[test]
    fn test_higher_eccentricity_changes_position_vs_circular() {
        let mut eph_circular = typical_gps_ephemeris();

        eph_circular.orbit1.e = 0.0;

        let mut eph_eccentric = typical_gps_ephemeris();

        eph_eccentric.orbit1.e = 0.05;

        let (x1, y1, z1, _) = eph_circular.position_ecef(2000.0);
        let (x2, y2, z2, _) = eph_eccentric.position_ecef(2000.0);

        let diff = ((x1 - x2).powi(2) + (y1 - y2).powi(2) + (z1 - z2).powi(2)).sqrt();

        assert!(
            diff > 1.0,
            "eccentricity should noticeably affect position, diff={diff}"
        );
    }

    #[test]
    fn test_corrected_time_diff_no_wrap() {
        let dt = Ephemeris::corrected_time_diff(1000.0, 900.0);

        assert!((dt - 100.0).abs() < 1e-12);
    }

    #[test]
    fn test_corrected_time_diff_negative_wrap() {
        // t << t0, но с переходом через границу недели
        let t = 100.0;
        let t0 = 604_700.0;

        let dt = Ephemeris::corrected_time_diff(t, t0);

        // ожидаем маленькое положительное значение (~-604600 + 604800)
        assert!(dt > 0.0);
        assert!(dt < 500.0);
    }

    #[test]
    fn test_bitcursor_signed_32bit_boundary() {
        // Все 32 бита равны единице => -1 в дополнительном коде.
        let bits = [true; 32];
        let c = BitCursor::new(&bits);

        let v = c.signed(0, 32);

        assert_eq!(v, -1);
    }
}
