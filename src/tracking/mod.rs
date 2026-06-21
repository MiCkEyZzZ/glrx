//! Tracking layer — DLL/FLL/PLL и многоканальное сопровождение спутников.
//!
//! # Структура
//!
//! ```text
//! tracking
//! ├── dll      — Delay Lock Loop: фаза/частота PRN-кода (GLRX-6)
//! ├── fll      — Frequency Lock Loop: грубый частотный захват (GLRX-8)
//! ├── pll      — Phase Lock Loop: фазовая синхронизация несущей (GLRX-7)
//! └── channel  — TrackingChannel / ChannelBank: многоканальное
//!                сопровождение, аллокация/деаллокация, метрики (GLRX-9)
//! ```

pub mod dll;
pub mod fll;
pub mod pll;
