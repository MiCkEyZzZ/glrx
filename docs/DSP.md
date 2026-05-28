# Signal Processing Layer — DSP

Module: `src/signal/`

## Назначение

Этот слой предоставляет **низкоуровневые DSP-примитивы**, используемые на каждом
этапе receiver pipeline. Все примитивы намеренно независимы от конкретного спутника
или системы (GPS/GLONASS/BeiDou) — они работают с `Complex32` и не знают ничего
о навигационных данных.

---

## Module Structure

````text
src/signal/
├── correlator/
│   ├── base.rs             # correlator_epl() — E/P/L accumulation
│   ├── code_utilities.rs   # shift_code(), make_epl_replicas()
│   ├── discriminators.rs   # EplOutput, DLL/PLL discriminators
│   ├── mod.rs
│   └── normalisation.rs    # power, normalize, cn0_estimate
├── block.rs                # SignalBlock (post-processing data container)
├── fft.rs                  # FftEngine (FFT/IFFT, cross-correlation)
├── filter.rs              # FIR filter, window functions
├── mixer.rs               # NCO, Mixer (carrier wipe-off)
├── mod.rs
└── resampler.rs           # Decimator, Interpolator

---

## Components

### Mixer / NCO (`mixer.rs`)

NCO (Numerically Controlled Oscillator) генерирует комплексную экспоненту
`exp(j·φ)`. Mixer использует NCO для частотного сдвига входного сигнала.

```text
входной IQ → Mixer (×exp(−j·2π·f_carrier·t)) → baseband IQ
````

**Ключевые свойства:**

- Фаза непрерывна между блоками (потоковый режим)
- `set_frequency()` меняет частоту без фазового скачка
- `mix_shift()` — stateless вариант для разовых операций

**Typical usage:**

| Operation           | freq_hz              |
| ------------------- | -------------------- |
| Downconversion (IF) | −IF_freq             |
| Carrier wipe-off    | −(carrier + doppler) |
| Test tone           | arbitrary frequency  |

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

| Method                    | Description                                               |
| ------------------------- | --------------------------------------------------------- |
| `fft()` / `ifft()`        | Forward / inverse DFT                                     |
| `power_spectrum()`        | `\|X[k]\|^2` for each frequency bin                       |
| `peak_bin()`              | Index of the maximum power bin                            |
| `bin_to_freq()`           | Converts FFT bin index to frequency (Hz)                  |
| `cross_correlate_power()` | `\|IFFT(FFT(s) · FFT*(t))\|^2` — core of PCPS acquisition |
| `fftshift()`              | Shifts DC component to the center (like numpy.fftshift)   |

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

| Discriminator | Formula             | Range      | Use case   |
| ------------- | ------------------- | ---------- | ---------- |
| dll_nelp      | (E² - L²)/(E² + L²) | [-1,1]     | main DLL   |
| dll_ele       | E - L               | variable   | simple DLL |
| pll_atan2     | atan2(Q, I)         | (-π,π]     | robust PLL |
| pll_dd_atan   | atan(Q/I)           | (-π/2,π/2] | BPSK PLL   |

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

| Function               | Description                                      |
| ---------------------- | ------------------------------------------------ |
| `compute_power()`      | (1/N) \* Σ abs(s[n])^2                           |
| `compute_rms()`        | sqrt(compute_power())                            |
| `normalize()`          | Normalize signal to unit power                   |
| `normalize_to_power()` | Scale signal to a target power level             |
| `scale()`              | Multiply by a real scalar coefficient            |
| `scale_complex()`      | Multiply by a complex coefficient (phase shift)  |
| `cn0_estimate()`       | C/N₀ via moment estimator (dB-Hz)                |
| `cn0_estimate_iwbp()`  | C/N₀ via narrowband/wideband power ratio (dB-Hz) |

---

### SignalBlock (`block.rs`)

A data block **after signal processing** (downconversion + filtering), passed
to acquisition and tracking stages.

```rust
SignalBlock {
    samples: Vec<Complex32>,   // baseband IQ samples
    sample_rate_hz: f64,       // may differ from original (decimation/interpolation)
    center_freq_hz: f64,       // RF carrier frequency before downconversion
    start_sample: u64,         // position in original IQ stream
    applied_doppler_hz: f64,   // Doppler shift applied during mixing
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

## Performance (benchmarks)

Все измерения для блока 2048 сэмплов (1 мс при 2.048 Msps):

| Operation                    | Estimated cost |
| ---------------------------- | -------------- |
| `Mixer::mix` (2048)          | ~5–10 µs       |
| `FirFilter` 63 taps (2048)   | ~15–30 µs      |
| `Decimator ×4` (2048)        | ~20–40 µs      |
| `FFT` 2048                   | ~50–100 µs     |
| `cross_correlate_power` 2048 | ~150–300 µs    |
| `correlator_epl` (2048)      | ~2–5 µs        |

Запустить бенчмарки: `cargo bench --bench dsp_benchmark`
