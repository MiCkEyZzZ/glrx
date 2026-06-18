# Changelog

All notable changes to **GLRX** are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] - 0000-00-00

### Added

- **pipeline**
  - Добавлена базовая реализация `receiver.rs`

- **acquisition (FFT-based signal search)**:
  - реализован PCPS (Parallel Code Phase Search) алгоритм на основе FFT
  - добавлена частотная сетка Doppler search: ±10 kHz с шагом 500 Hz
  - реализован CFAR-based detector для адаптивного порога обнаружения пиков
  - добавлена структура результата `AcquisitionResult { prn, doppler_hz, code_phase }`
  - реализован полный поиск по всем 32 GPS PRN за один проход
  - добавлена процедура fine frequency estimation после грубого Doppler поиска
  - добавлены benchmark-метрики времени поиска (single PRN / full 32 PRN scan)

- **acquisition module integration**:
  - добавлены модули `fft_search.rs`, `detector.rs`, `mod.rs`
  - интегрирован FFT-based cross-correlation pipeline в acquisition layer
  - добавлена обработка результатов корреляции через power surface analysis
  - подготовлена архитектура расширения под multi-constellation acquisition

### Changed

- **docs**:
  - добавлены небольшие правки по коду в acquisition модуль вызванные Clippy линтером
  - улучшена документацию в `DSP.md`, `NAVIGATION.md`, `PIPELINE.md`, `TRACKING.md`
  - обновлён README.md файл добавлена схема архитектуры

## [0.1.0] - 2026-05-31

### Added

- **rf / signal / acquisition foundation (core DSP pipeline bootstrap)**:
  - реализован полный RF frontend слой с поддержкой file-based IQ источника и SDR
    abstraction layer
  - добавлен `IqSource` trait как единая абстракция потоковых и файловых источников
    IQ данных
  - реализован `FileSource` с поддержкой int8 / int16 / float32 interleaved IQ форматов
  - добавлен mock SDR источник для тестирования DSP pipeline без железа
  - реализован SPSC ring buffer streaming слой с политиками переполнения
    (DropOldest / BlockProducer / ErrorOnOverflow)
  - добавлена детекция разрывов потока и метрики стабильности входного сигнала

- **DSP core primitives (signal processing layer)**:
  - добавлены базовые DSP операции: complex mixing (NCO-based downconversion),
    FIR filtering, resampling (decimation/interpolation)
  - интегрирован FFT engine на базе rustfft с кешированием планов и оптимизированными
    scratch-буферами
  - реализованы power spectrum utilities, peak detection и cross-correlation через
    FFT domain
  - добавлены EPL correlation utilities (early/prompt/late) для GNSS tracking pipeline
  - реализованы normalization utilities (RMS / power scaling / gain control)
  - добавлены benchmark suites для всех DSP операций с throughput-метриками

- **PRN / signal generation layer**:
  - реализована генерация GPS L1 C/A PRN кодов (Gold sequences, 1023 chips)
  - добавлена таблица PRN 1–32 с корректной конфигурацией регистров сдвига
  - реализован кэш PRN последовательностей для ускорения acquisition
  - добавлена поддержка fractional chip shifting для sub-sample correlation
  - подготовлена архитектура расширения под GLONASS и Galileo PRN (stub layer)

- **streaming & metrics infrastructure**:
  - добавлен потоковый ring buffer с фиксированными слотами и lock-free consumer
    interface
  - реализована обработка backpressure и overflow policies для real-time DSP pipeline
  - добавлена система метрик входного потока (sample rate estimation, dropped samples,
    interruptions, signal power estimation)
  - интегрированы runtime diagnostics для контроля стабильности RF input

- **cargo/config.toml**:
  - добавил `target-dir` и `rustflags` для оптимизации нативного производительности

- **justfile**:
  - добавил команды для сборки проекта: `build`, `build-release`, `build-perf`,
    `build-native`

- **signal**
  - добавлен `mixer.rs`:
    - `Nco` — фазовый аккумулятор с advance(), set_frequency(), reset(), generate(n)
    - `Mixer` — потоковый, фаза непрерывна между блоками, `mix()` / `mix_inplace()`
      / `set_frequency()` / `adjust_frequency()`
      `mix_shift()` — stateless для разовых операций
    - `generate_carrier()` — генерация опорного тона
  - добавлен `filter.rs`:
    - `Window` — `Rectangular`, `Hamming`, `Hann`, `Blackman` с функцией
      `value(n, len)`
    - `FirFilter` — direct-form на `VecDeque` (задержка сохраняется между блоками),
      `apply() / apply_inplace() / apply_single() / reset()`
    - `FirFilter::low_pass()` — windowed sinc метод: `h[n] = 2·fc·sinc(2·fc·(n−M/2))·w[n]`
  - добавлен `resampler.rs`:
    - `Decimator` — автоматический anti-aliasing LPF + `step_by(factor)`, непрерывное
      состояние
    - `Interpolator` — zero-insertion + smoothing LPF с компенсацией gain `×factor`
  - добавлен `fft.rs`:
    - `FftEngine` — кешированный план rustfft, scratch-буфер аллоцируется один раз
    - `fft()` / `ifft()` (нормировка 1/N), `fft_inplace()` / `ifft_inplace()`
    - `power_spectrum()`, `power_spectrum_db()`, `peak_bin()`, `bin_to_freq()`
    - `cross_correlate_power()` — `|IFFT(FFT(s)·FFT*(t))|²`, ядро FFT-acquisition
    - `fftshift()`
  - добавлен `correlator_utils.rs`:
    - `correlate()` / `correlate_epl()` — EPL корреляция одним проходом
    - `EplOutput` с дискриминаторами: `dll_nelp()`, `dll_ele()`, `pll_atan2()`,
      `pll_dd_atan()`
    - `shift_code()` — сдвиг с линейной интерполяцией (дробные чипы)
    - `compute_power()`, `compute_rms()`, `scale()`, `normalize()`, `normalize_to_power()`
    - `cn0_estimate()` — narrow-band moment estimator
  - добавлены `benches/dsp_benchmark.rs`
    - Criterion benchmarks для всех примитивов с `Throughput::Elements`:
      - Mixer: mix, mix_inplace, mix_shift, generate_carrier
      - FIR filter: 15/31/63/127 taps × apply / apply_inplace
      - Decimation: factor 2/4/8
      - FFT: 512/1024/2048/4096 × forward / inverse / cross_correlate_power
      - Correlator: correlate_single, correlate_epl, shift_code
      - Power/norm: compute_power, normalize

- **.github**:
  - добавлен CODEOWNERS
  - добавлен cargo-blacklist.txt
  - Добавлен DISCUSSION_TEMPLATE: `feature-requests.yml`
  - Добавлен ISSUE_TEMPLATE: `bug_report.yml`, `config.yml`, `enhancement.yml`
  - Добавлен workflows: `semantic-pull-request.yml`
  - Добавлен `dependabot.yml`
  - Добавлен `FUNDING.yml`

- **iq_source**:
  - `IqSource` trait и `IqBlock` для унифицированного чтения IQ-сэмплов.

- **file**:
  - `FileSource` Читает raw interleaved I/Q в трёх форматах через `byteorder`
  - `BufReader` с буфером 1 МБ для минимизации syscall'ов
  - Looping: EOF → rewind → продолжает заполнять блок (работает даже если `n > file_samples`)
  - Seek по семплам (не байтам) — вычисляет байтовый offset сам
  - `total_samples()` и `duration_s()` через метаданные файла
  - Метрики sample rate через накопленный счётчик

- **sdr**:
  - `MockSdrSource` — детерминированный комплексный синусоид exp(j·2π·f·t) с
    псевдошумом, фаза непрерывна между блоками
  - Шаблон `SoapySource` (за feature flag sdr) — скелет с подробными комментариями
    где что подключать, enumerate() stub

- **stream**:
  - SPSC ring buffer на `Mutex<Option<Slot>>` слотах
  - Три политики переполнения: `DropOldest`, `ErrorOnOverflow`, `BlockProducer`
  - Детекция gap: если `written_at` двух соседних слотов расходится больше
    `slot_duration + 5ms` → `interruptions++` + `log::warn`
  - `StreamConsumer` реализует `IqSource` — прозрачно встраивается в pipeline
  - Исправлен баг с `wrapping_sub % (capacity+1)` — заменён на явный
    `AtomicUsize count`

- **config**:
  - `RfConfig` для настройки частоты дискретизации, центральной частоты и усиления.

- **metrics**:
  - Метрики потока (`SourceMetrics`): total samples, dropped samples, measured rate,
    interruptions, power estimate.
  - Тесты для чтения файлов, нормализации сэмплов и проверки метрик.

- **docs**:
  - добавил описание `ARCHITECTURE`
  - добавил описание `DSP`
  - добавил описание `NAVIGATION`
  - добавил описание `PIPELINE`
  - добавил описание `TRACKING`

### Changed

- Комментарии и документация переведены на английский язык.
