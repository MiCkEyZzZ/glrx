//! Подсистема RF для обработки IQ-данных.
//!
//! Содержит конфигурацию, ошибки, форматы сэмплов, источники IQ,
//! метрики, нормализацию, SDR-абстракции и потоковый буфер.

pub mod config;
pub mod error;
pub mod file;
pub mod format;
pub mod iq_source;
pub mod metrics;
pub mod normalise;
pub mod sdr;
pub mod stream;

pub use config::*;
pub use error::*;
pub use file::*;
pub use format::*;
pub use iq_source::*;
pub use metrics::*;
