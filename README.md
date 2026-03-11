# GLRX — Rust GNSS Receiver

![Rust](https://img.shields.io/badge/language-Rust-orange)
![Status](https://img.shields.io/badge/status-pre--alpha-yellow)

**GLRX** - это модульный GNSS ресивер на Rust, ориентированный на исследовательские
и инженерные эксперименты с сигналами GPS, ГЛОНАС и другими спутниковыми системами.

## Возможности

- Чтение IQ данных из файлов или SDR-устройств.
- Базовая обработка сигнала: mixing, фильтрация, ресемплирование.
- Acquisition спутников через FFT.
- Tracking loops: DLL, PLL, FLL.
- Декодирование навигационных сообщений и эфемерид.
- Вычисление псевдодальностей и координат (Least Squares, Kalman).
- Multi-channel сопровождение спутников.
- Вывод данных в NMEA/UBX.
- Интеграция с внешними инструментами: GLOS, GLINT, USMET.
- Поддержка тестов, бенчмарков и observability.

## Быстрый старт

### Требования

- Rust (stable toolchain)
- Optional SDR: SoapySDR, RTL-SDR, HackRF

### Сборка

```bash
cargo build --workspace --release

Тесты

cargo test --workspace

Запуск (симулятор)

cargo run --release --bin glrx -- \
  --device sim \
  --freq 1602MHz \
  --rate 2MHz \
  --gain 40 \
  --duration 5
```

## Фазы развития

1.  IQ-ридер и DSP-примитивы
2.  FFT-based Acquisition
3.  Tracking loops (DLL/PLL/FLL) и multi-channel
4.  Декодирование навигационных сообщений и эфемерид
5.  Вычисление псевдодальностей и solver
6.  Вывод NMEA/UBX и интеграция с GLINT/USMET
7.  High-level pipeline и синхронизация времени
8.  Оптимизация: SIMD, latency
9.  Observability, тесты, валидация

## Цели

• Построить модульный GNSS ресивер на Rust.
• Обеспечить reproducibility и расширяемость.
• Позволить интеграцию с существующими инструментами анализа и хранения телеметрии.
