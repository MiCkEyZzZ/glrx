//! Двойная верификация acquisition-результатов.
//!
//! После первичного обнаружения пика PCPS-алгоритма качество сигнала
//! нестабильно: возможны ложные срабатывания от шумов пиков, боковых
//! лепестков автокорреляции и RFI. Этот модуль реализует двухэтапную
//! схему верификации:
//!
//! ```text
//! Первый проход (грубый, широкий Doppler)
//!     │
//!     ▼
//! Кандидат найден? → нет → retry / fail
//!     │ да
//!     ▼
//! Второй проход (точный, узкий Doppler ±step вокруг кандидата)
//!     │
//!     ▼
//! Оба прохода подтверждены? → VerificationVerdict::Confirmed
//! Только первый?           → VerificationVerdict::Marginal
//! Ни один?                 → VerificationVerdict::Rejected
//! ```
//!
//! # Политика повтора
//!
//! При неудаче верификации применяется экспоненциальный back-off:
//!
//! - попытка 0: 0 мс задержки
//! - попытка 1: `base_delay_ms`
//! - попытка k: `base_delay_ms * 2^(k - 1)`
//! - максимум: `max_delay_ms`
//!
//! # Статистика
//!
//! [`VerifierStats`] накапливает счётчики по всем PRN, позволяя
//! оценить надёжность обнаружения и уровень ложных срабатываний.

use std::time::Duration;

use crate::acquisition::fft_search::SearchResult;

/// Результат двухэтапной верификации acquisition.
#[derive(Debug, Clone)]
pub enum VerificationVerdict {
    /// Оба прохода обнаружили спутник с уровнем выше порога
    ///
    /// Поля содержат окончательный уточнённый результат
    Confirmed {
        /// Результат второго (уточняющего) прохода
        result: SearchResult,
        /// Оценка C/N₀ в дБ-Гц
        cn0_db_hz: f32,
        /// Время, затраченное на оба прохода
        elapsed: Duration,
    },

    /// Первый проход дал пик, но второй не подтвердил с тем же порогом
    ///
    /// Возможна нестабильность сигнала, рекомендуется повтор
    Marginal {
        /// Результат первого прохода (ненадёжный)
        result: SearchResult,
        /// Время, затраченное на оба прохода
        elapsed: Duration,
    },

    /// Оба прохода не обнаружили спутники
    Rejected {
        /// PRN, который искали
        prn: u8,
        /// Peak-to-noise из первого прохода (если было хоть что-то)
        peak_to_noise: Option<f32>,
        /// Время, затраченное на попытку
        elapsed: Duration,
    },
}

impl VerificationVerdict {
    /// Возвращает `true` если верификация прошла успешно.
    #[must_use]
    pub fn is_confirmed(&self) -> bool {
        matches!(self, VerificationVerdict::Confirmed { .. })
    }

    /// Возвращает `SearchResult` если подтверждён или маргинален.
    #[must_use]
    pub fn search_result(&self) -> Option<&SearchResult> {
        match self {
            VerificationVerdict::Confirmed { result, .. } => Some(result),
            VerificationVerdict::Marginal { result, .. } => Some(result),
            VerificationVerdict::Rejected { .. } => None,
        }
    }
}
