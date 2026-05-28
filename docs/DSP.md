# Signal Processing Layer — DSP

Модуль: `src/signal/`

## Назначение

Этот слой предоставляет **низкоуровневые DSP-примитивы**, используемые на каждом
этапе receiver pipeline. Все примитивы намеренно независимы от конкретного спутника
или системы (GPS/GLONASS/BeiDou) — они работают с `Complex32` и не знают ничего
о навигационных данных.

---

## Структура модуля

```text
src/signal/
├── correlator/
│   ├── base.rs             — correlator_epl() — суммирование E/P/L
│   ├── discriminators.rs   — EplOutput, DLL/PLL дискриминаторы
│   ├── code_utilities.rs   — shift_code(), make_epl_replicas()
│   └── normalisation.rs    — power, normalize, cn0_estimate
├── block.rs                — SignalBlock (данные после обработки)
├── fft.rs                  — FftEngine (FFT/IFFT, кросс-корреляция)
├── filter.rs               — FIR-фильтр, оконные функции
├── mixer.rs                — NCO, Mixer (carrier wipe-off)
├── mod.rs
└── resampler.rs            — Decimator, Interpolator
```

---

## Компоненты

### Mixer / NCO (`mixer.rs`)

NCO (Numerically Controlled Oscillator) генерирует комплексную экспоненту
`exp(j·φ)`. Mixer использует NCO для частотного сдвига входного сигнала.

```text
входной IQ → Mixer (×exp(−j·2π·f_carrier·t)) → baseband IQ
```

**Ключевые свойства:**

- Фаза непрерывна между блоками (потоковый режим)
- `set_frequency()` меняет частоту без фазового скачка
- `mix_shift()` — stateless вариант для разовых операций

**Типичное применение:**

| Операция             | freq_hz              |
| -------------------- | -------------------- |
| Downconversion на IF | −IF_freq             |
| Carrier wipe-off     | −(carrier + doppler) |
| Тестовый тон         | любая                |

---

### FIR Filter (`filter.rs`)

Direct-form FIR с линией задержки `VecDeque`. Проектирование методом windowed sinc:

```text
h[n] = 2·fc · sinc(2·fc·(n − M/2)) · w[n]
```

Коэффициенты нормируются по DC-усилению (сумма = 1).

**Оконные функции:**

| Window      | Stopband | Transition band |
| ----------- | -------- | --------------- |
| Rectangular | −21 dB   | узкая           |
| Hamming     | −43 dB   | средняя         |
| Hann        | −31 dB   | средняя         |
| Blackman    | −74 dB   | широкая         |

**Состояние фильтра сохраняется между блоками** — можно вызывать `apply()`
последовательно для потока данных.

---

### Resampler (`resampler.rs`)

Децимация и интерполяция с автоматическим антиалиасинговым LPF (63 taps, Hamming,
cutoff = 0.45/factor).

```text
Decimation:     LPF → step_by(factor)
Interpolation:  zero-stuffing → LPF (×factor gain)
```

**Важно:** фильтр в `Decimator`/`Interpolator` сохраняет состояние — непрерывная
потоковая обработка корректна.

---

### FFT Engine (`fft.rs`)

Кэшированный план rustfft + scratch-буфер. Один экземпляр на размер FFT,
переиспользуется многократно.

| Метод                     | Описание                             |                       |                            |
| ------------------------- | ------------------------------------ | --------------------- | -------------------------- |
| `fft()` / `ifft()`        | Прямое/обратное ДПФ                  |                       |                            |
| `power_spectrum()`        | `                                    | X[k]                  | ²` для каждого бина        |
| `peak_bin()`              | Индекс бина с максимальной мощностью |                       |                            |
| `bin_to_freq()`           | Бин → частота в Гц                   |                       |                            |
| `cross_correlate_power()` | `                                    | IFFT(FFT(s)·FFT\*(t)) | ²` — ядро PCPS acquisition |
| `fftshift()`              | DC в центр (как numpy.fftshift)      |                       |                            |

**PCPS Acquisition** использует `cross_correlate_power()`:

```text
для каждого Doppler trial:
  смешать сигнал с trial carrier
  cross_correlate_power(iq_block, prn_code_fft)
  найти пик → code_phase + doppler
```

---

### Correlator (`correlator/`)

#### `base.rs` — `correlator_epl()`

Ядро tracking loop. Один вызов = одна когерентная интеграция (обычно 1 мс):

```text
E += s[n] × code_early[n]
P += s[n] × code_prompt[n]   ← DLL / PLL
L += s[n] × code_late[n]
```

#### `discriminators.rs` — `EplOutput`

| Дискриминатор   | Формула      | Диапазон | Применение    |
| --------------- | ------------ | -------- | ------------- | ---------- | ------------------- | ------------ | --------------------- | --- | --- | ------- | -------------- |
| `dll_nelp()`    | `(           | E        | ²−            | L          | ²)/(                | E            | ²+                    | L   | ²)` | [−1,+1] | DLL (основной) |
| `dll_ele()`     | `            | E        | −             | L          | `                   | зависит от A | DLL (ненормированный) |     |     |         |                |
| `pll_atan2()`   | `atan2(Q,I)` | (−π,π]   | PLL (с битом) |            |                     |              |                       |     |     |         |                |
| `pll_dd_atan()` | `atan(Q/     | I        | )`            | (−π/2,π/2] | PLL BPSK (без бита) |              |                       |     |     |         |                |

#### `code_utilities.rs` — `shift_code()`

Сдвиг кода на дробное число сэмплов с линейной интерполяцией:

```text
offset > 0 → задержка (Late-реплика)
offset < 0 → опережение (Early-реплика)
offset = 0 → без изменений (Prompt-реплика)
```

`make_epl_replicas(prompt, half_chip)` — удобная обёртка для создания трёх реплик
за один вызов.

#### `normalisation.rs`

| Функция                | Описание                                     |      |     |
| ---------------------- | -------------------------------------------- | ---- | --- |
| `compute_power()`      | `(1/N)·Σ                                     | s[n] | ²`  |
| `compute_rms()`        | `√compute_power()`                           |      |     |
| `normalize()`          | Привести к единичной мощности                |      |     |
| `normalize_to_power()` | Привести к заданной мощности                 |      |     |
| `scale()`              | Умножить на вещественный коэф.               |      |     |
| `scale_complex()`      | Умножить на комплексный коэф. (поворот фазы) |      |     |
| `cn0_estimate()`       | C/N₀ методом moment estimator (дБ-Гц)        |      |     |
| `cn0_estimate_iwbp()`  | C/N₀ методом NB/WB power ratio (дБ-Гц)       |      |     |

---

### SignalBlock (`block.rs`)

Блок данных **после** signal-обработки (downconversion + фильтрация), передаваемый
в acquisition/tracking.

```rust
SignalBlock {
    samples: Vec<Complex32>,   // baseband IQ
    sample_rate_hz: f64,       // может отличаться от исходной (децимация)
    center_freq_hz: f64,       // несущая до downconversion
    start_sample: u64,         // позиция в оригинальном потоке
    applied_doppler_hz: f64,   // применённый доплеровский сдвиг
}
```

---

## Типичный порядок вызовов (1 мс GPS epoch)

```rust
// 1. Carrier wipe-off
let baseband = carrier_mixer.mix(&iq_block.samples);

// 2. (Опционально) Децимация
let decimated = decimator.decimate(&baseband);

// 3. Подготовка кодовых реплик
let (early, prompt, late) = make_epl_replicas(&prn_code, half_chip_samples);

// 4. EPL-корреляция
let epl = correlator_epl(&decimated, &early, &prompt, &late);

// 5. Дискриминаторы → коррекция tracking loops
let dll_err = epl.dll_nelp();   // → DLL loop filter → code NCO
let pll_err = epl.pll_dd_atan(); // → PLL loop filter → carrier NCO

// 6. Оценка качества сигнала
prompt_history.push(epl.prompt);
let cn0 = cn0_estimate(&prompt_history, 0.001); // дБ-Гц
```

---

## Производительность (ориентиры)

Все измерения для блока 2048 сэмплов (1 мс при 2.048 Msps):

| Операция                   | Оценка       |
| -------------------------- | ------------ |
| Mixer::mix (2048)          | ~5–10 мкс    |
| FirFilter 63 taps (2048)   | ~15–30 мкс   |
| Decimator ×4 (2048)        | ~20–40 мкс   |
| FFT 2048                   | ~50–100 мкс  |
| cross_correlate_power 2048 | ~150–300 мкс |
| correlator_epl (2048)      | ~2–5 мкс     |

Запустить бенчмарки: `cargo bench --bench dsp_benchmark`
