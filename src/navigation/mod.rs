//! Navigation layer - декодирование навигационных сообщений
//! GPS L1 C/A.
//!
//! # Структура
//!
//! ```text
//! navigation
//! └── frame_decoder  — Bit Sync + Frame Detection: выравнивание битов,
//!                       детекция TLM-преамбулы, parity (Hamming 6 бит),
//!                       сборка 300-битного subframe.
//! ```

pub mod ephemeris;
pub mod frame_decoder;
pub mod nav_data;
