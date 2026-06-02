# Tracking Layer — Слежение за спутниками

Module:

```text
src/tracking/
```

## Назначение

Tracking-слой обеспечивает **непрерывную синхронизацию** со спутниковыми сигналами
после этапа acquisition. Каждый спутник обрабатывается в отдельном **tracking channel**.

## Канал отслеживания (Tracking Channel)

```rust
struct TrackingChannel {
    prn: u8,
    pll: Pll,                       // фазовая блокировка несущей
    dll: Dll,                       // блокировка задержки кода
    fll: Fll,                       // блокировка частоты (помогает ФАПЧ во время ввода сигнала)
    cn0_estimator: Cn0Estimator,
    state: ChannelState,
    prompt_history: Vec<Complex32>, // для оценки C/N₀
}
```

### Состояния канала (Channel States)

```text
IDLE
  │ ← AcquisitionResult
  ▼
FLL_LOCK   (частота захвачена, фаза нет)
  │ (переход когда FLL стабилен)
  ▼
PLL_LOCK   (фаза захвачена, данные читаемы)
  │ (переход когда синхронизированы биты)
  ▼
BIT_SYNC   (навигационные биты декодируются)
```

## Петли слежения (Tracking Loops)

### DLL — Цикл блокировки с задержкой кода (Code Delay Lock Loop)

Tracks PRN code phase.

```text
correlator_epl(signal, early, prompt, late)
    │
    ▼
dll_nelp() = (|E|² − |L|²) / (|E|² + |L|²)
    │
    ▼
Loop filter (2nd order)
    │
    ▼
Code NCO correction (samples/s)
```

**Parameters:**

- `chip_spacing`: 0.1–1.0 чипа (early-late расстояние)
- `bandwidth`: 1–5 Hz (ширина петли)
- `order`: 2 (позиция + скорость)

### PLL — Phase Lock Loop

Отслеживает фазу несущей.

```text
epl.pll_dd_atan() = atan(Q_P / |I_P|)
    │
    ▼
Loop filter (3rd order для высокой динамики)
    │
    ▼
Carrier NCO correction (Hz)
```

**Параметры:**

- `bandwidth`: 10–25 Гц
- `order`: 3 (фаза + частота + скорость частоты)
- Используется DD-atan для устранения бит-неоднозначности

### FLL — Frequency Lock Loop

Помогает PLL при первоначальном захвате.

```text
cross_product_discriminator(P_prev, P_curr)
    │
    ▼
Loop filter (1st order)
    │
    ▼
Carrier NCO correction (только частота, не фаза)
```

**FLL → PLL switching:**

- FLL активен до достижения фазового lock
- После PLL lock FLL отключается
- При потере lock: откат к FLL (или полный реacquisition)

## Оценка C/N₀

```rust
// Накапливаем 20 prompt-значений (20 мс)
prompt_history.push(epl.prompt);
if prompt_history.len() >= 20 {
    let cn0 = cn0_estimate(&prompt_history, 0.001);
    // Обычно: 35–50 дБ-Гц
}
```

**Пороговые значения:**

- `> 40 дБ-Гц` — надёжный PLL lock
- `35–40 дБ-Гц` — нестабильный PLL, возможна потеря
- `< 35 дБ-Гц` — потеря lock, нужен реacquisition

## Многоканальный

Приёмник поддерживает N параллельных каналов (8/16/32).

```text
IqBlock (2048 сэмплов, 1 мс)
    │
    ├─→ Channel[G01]: correlator_epl → DLL/PLL
    ├─→ Channel[G05]: correlator_epl → DLL/PLL
    ├─→ Channel[G12]: correlator_epl → DLL/PLL
    └─→ Channel[G20]: correlator_epl → DLL/PLL
```

Параллельная обработка каналов через Rayon или tokio::task::spawn_blocking.

## Проектирование петлевого фильтра

Типичный 2nd-order DLL фильтр:

```text
bandwidth = 2 Hz, damping = 0.707

ω_n = bandwidth * 8 * damping / (4 * damping² + 1)
τ_1 = 1 / ω_n²
τ_2 = 2 * damping / ω_n

y[k] = y[k-1] + (τ_2/τ_1 + T/τ_1) * e[k] - τ_2/τ_1 * e[k-1]
```

где `T = 0.001 с` (период интеграции), `e[k]` — выход дискриминатора.

## Файловая структура (планируемая)

```text
src/tracking/
├── mod.rs          — экспорты, TrackingState
├── channel.rs      — TrackingChannel, ChannelState
├── dll.rs          — Dll struct, loop filter
├── pll.rs          — Pll struct, loop filter
└── fll.rs          — Fll struct, cross-product discriminator
```
