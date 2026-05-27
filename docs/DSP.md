# DSP Signal Layer

`src/signal/` содержит низкоуровневые DSP-примитивы, используемые на всех
этапах pipeline.

## Компоненты

| Модуль                         | Назначение                                   |
| ------------------------------ | -------------------------------------------- |
| `mixer.rs` / `Nco`, `Mixer`    | Numerically Controlled Oscillator, IQ-микшер |
| `filter.rs` / `FirFilter`      | FIR-фильтр (windowed-sinc), потоковый        |
| `resampler.rs`                 | Децимация и интерполяция                     |
| `fft.rs` / `FftEngine`         | FFT/IFFT, cross-correlation, power spectrum  |
| `correlator/base.rs`           | EPL-коррелятор, `EplOutput`                  |
| `correlator/code_utilities.rs` | Сдвиг кодовой реплики (дробная задержка)     |
| `correlator/discriminators.rs` | Standalone DLL/PLL/FLL дискриминаторы        |
| `correlator/normalisation.rs`  | Нормализация EPL, оценка CN0                 |
| `block.rs` / `SignalBlock`     | Блок данных после downconversion             |

## Типичный поток данных

```text
IqBlock (RF layer)
  └─► Mixer::mix()          — перенос в baseband
  └─► FirFilter::apply()    — антиалиасинг / ограничение полосы
  └─► Decimator::decimate() — опционально, снижение fs
  └─► SignalBlock            — передаётся в Acquisition / Tracking
```

## NCO и Mixer

`Nco` генерирует комплексную экспоненту `exp(j·φ)`. Фаза накапливается
между вызовами — непрерывность фазы гарантирована при блочной обработке.

```rust
let mut mixer = Mixer::new(doppler_hz, sample_rate_hz);
let baseband = mixer.mix(&iq_block.samples);
```

Для разового сдвига без сохранения состояния: `mix_shift(input, freq, fs)`.

## FIR-фильтр

`FirFilter::low_pass(cutoff_hz, fs, num_taps, window)` — проектирование
методом windowed-sinc. Поддерживает `Rectangular`, `Hamming`, `Hann`,
`Blackman`.

Внутреннее состояние (линия задержки) сохраняется между блоками:
потоковая обработка без артефактов на границах блоков.

## Децимация / Интерполяция

```rust
let mut dec = Decimator::new(4); // fs/4
let downsampled = dec.decimate(&filtered);
```

Встроенный антиалиасинговый FIR с cutoff ≈ 0.45/factor.

## FFT и cross-correlation

```rust
let mut engine = FftEngine::new(2048);
let power = engine.cross_correlate_power(&signal, &prn_replica);
let peak  = power.iter().enumerate().max_by(…);
```

Используется в PCPS acquisition (#GLRX-4).

## EPL-коррелятор

```rust
let epl = correlator_epl(&baseband, &code_early, &code_prompt, &code_late);
let dll_error = epl.dll_nelp();   // DLL дискриминатор
let pll_error = epl.pll_dd_atan(); // PLL дискриминатор
```

Длина всех массивов должна совпадать. Интервал интеграции — обычно 1 мс
(2048 сэмплов при 2.048 МГц).

## CN0 оценка

```rust
use signal::correlator::normalisation::cn0_nbp;
let cn0_linear = cn0_nbp(&epl);
let cn0_db_hz  = 10.0 * cn0_linear.log10() + 10.0 * (1000.0_f32).log10();
```
