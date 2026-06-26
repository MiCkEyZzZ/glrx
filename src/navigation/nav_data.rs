//! Хранилище навигационных данных приёмника: эфемериды по PRN,
//! ионосферная модель (Klobuchar), almanac (заглушка под GLRX-12).
//!
//! `NavData` - центральная точка накопления данных, декодированных из
//! навигационного сообщения GPS (через [`crate::navigation::ephemeris`]),
//! используемая потребителями выше по конвеёеру (observables, solver) для
//! получения текущих эфемерид конкретного спутника.

use crate::navigation::{
    ephemeris::{BitCursor, concat_data_words},
    frame_decoder::DecodedSubframe,
};

/// Параметры ионосферной модели Клобухара, декодируемые из Subframe 4
/// (страница 18).
///
/// GPS ICD-200 передаёт 4 амплитудных коэффициента `α₀..α₃` и 4
/// коэффициента периода `β₀..β₃`, используемых для оценки ионосферной
/// задержки сигнала L1 в зависимости от позиции пользователя и времени
/// суток (см. `docs/NAVIGATION.md`, раздел "Ionospheric Model").
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IonosphericModel {
    /// Амплитудные коэффициенты (секунды), масштабы `2⁻³⁰, 2⁻²⁷, 2⁻²⁴, 2⁻²⁴`
    pub alpha: [f64; 4],

    /// Коэффициенты периода (секунды), масштабы `2¹², 2¹⁴, 2¹⁶, 2¹⁶`
    pub beta: [f64; 4],
}

impl IonosphericModel {
    /// Разбирает параметры ионосферной модели из Subframe 4, страница 18.
    ///
    /// # Примечание о раскладке
    ///
    /// Subframe 4 используется механизм "страниц" (page 1-25, циклически
    /// переключаемых через биты данных), и точная битовая раскладка
    /// page 18 не определяется одним только `subfrane_id == 4` - нужна
    /// дополнительная проверка page ID, не входящая в [`DecodedSubframe`]
    /// текущего вида. Этот парсер предполагает, что вызывающий код уже
    /// отфильтрован нужный subframe по внешнему признаку (data ID / DV ID - 56,
    /// согласно ICD) и передаёт сюда корректные информационные слова.
    ///
    /// Раскладка (биты, начиная с начала информационных слов 3..10):
    /// `α₀[8] α₁[8] α₂[8] α₃[8] β₀[8] β₁[8] β₂[8] β₃[8] ...`
    #[must_use]
    pub fn parse_page18(subframe: &DecodedSubframe) -> Option<Self> {
        if subframe.subframe_id != 4 {
            return None;
        }

        let bits = concat_data_words(subframe);
        let c = BitCursor::new(&bits);

        let alpha0 = c.signed(0, 8);
        let alpha1 = c.signed(8, 8);
        let alpha2 = c.signed(16, 8);
        let alpha3 = c.signed(24, 8);
        let beta0 = c.signed(32, 8);
        let beta1 = c.signed(40, 8);
        let beta2 = c.signed(48, 8);
        let beta3 = c.signed(56, 8);

        Some(Self {
            alpha: [
                f64::from(alpha0) * 2f64.powi(-30),
                f64::from(alpha1) * 2f64.powi(-27),
                f64::from(alpha2) * 2f64.powi(-24),
                f64::from(alpha3) * 2f64.powi(-24),
            ],
            beta: [
                f64::from(beta0) * 2f64.powi(12),
                f64::from(beta1) * 2f64.powi(14),
                f64::from(beta2) * 2f64.powi(16),
                f64::from(beta3) * 2f64.powi(16),
            ],
        })
    }
}

#[cfg(test)]
mod tests {
    use crate::navigation::frame_decoder::HowWord;

    use super::*;

    fn make_subframe(
        subframe_id: u8,
        words: [[bool; 24]; 10],
    ) -> DecodedSubframe {
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

    #[test]
    fn test_iono_parse_page18_returns_none_for_wrong_subframe_id() {
        let sf = make_subframe(1, [[false; 24]; 10]);

        assert!(IonosphericModel::parse_page18(&sf).is_none());
    }
}
