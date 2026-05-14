//! Ошибки RF-подсистемы и единый результат операций.

use std::time::Duration;

use thiserror::Error;

/// Унифицированный результат операций RF-подсистемы.
pub type RfResult<T> = Result<T, RfError>;

/// Ошибки RF-подсистемы.
///
/// Используется для ошибок чтения IQ-данных, SDR-устройств, потоков,
/// буферов и некорректной конфигурации.
#[derive(Debug, Error)]
pub enum RfError {
    /// Ошибка ввода-вывода.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Достигнут конец IQ-файла или потока данных.
    #[error("end of IQ file reached")]
    EndOfFile,

    /// Указан неподдерживаемый формат образцов.
    #[error("unsupported sample format: {0}")]
    UnsupportedFormat(String),

    /// Переполнение кольцевого буфера.
    #[error("ring buffer overflow — {dropped} samples dropped")]
    BufferOverflow {
        /// Количество сэмплов, потерянных при переполнении.
        dropped: usize,
    },

    /// Обнаружен разрыв в потоке IQ-данных.
    #[error("stream interrupted after {gap:?} gap")]
    StreamInterrupted {
        /// Длительность разрыва потока.
        gap: Duration,
    },

    /// Ошибка, возвращённая SDR-устройством или его драйвером.
    #[error("SDR error: {0}")]
    Sdr(String),

    /// Ошибка конфигурации RF-подсистемы.
    #[error("invalid configuration: {0}")]
    Config(String),
}
