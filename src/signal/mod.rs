//! DSP-примитивы GNSS-приёмника.
//!
//! # Структура модуля
//!
//! | Подмодуль | Назначение |
//! |-----------|-----------|
//! | [`mixer`] | NCO, частотный сдвиг, carrier wipe-off |
//! | [`filter`] | FIR-фильтр, windowed-sinc design |
//! | [`resampler`] | Децимация и интерполяция |
//! | [`fft`] | FFT/IFFT, кросс-корреляция, спектральный анализ |
//! | [`correlator`] | EPL-коррелятор, дискриминаторы, утилиты кода, C/N₀ |
//! | [`block`] | [`SignalBlock`] — блок данных после signal-обработки |
//!
//! # Принципы проектирования
//!
//! Каждый примитив работает с `&[Complex32]` и либо:
//! * возвращает новый `Vec<Complex32>` (простой API), или
//! * изменяет `&mut [Complex32]` in-place (zero-allocation для горячих путей)
//!
//! Stateful примитивы ([`mixer::Mixer`], [`filter::FirFilter`], …)
//! сохраняют внутреннее состояние между вызовами, обеспечивая
//! корректную потоковую обработку.

pub mod block;
pub mod correlator;
pub mod fft;
pub mod filter;
pub mod mixer;
pub mod prn_code;
pub mod resampler;
