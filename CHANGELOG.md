# Changelog

All notable changes to **GLRX** are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] — 00-00-0000

### Added

- **carrier tracking (PLL/FLL)** сразу после пункта про FLL-assisted PLL contour:
- Issue #GLRX-8 completed: robust FLL acquisition path for large initial Doppler
  errors
  - Added free-function cross-product discriminator:
    `cross_product_discriminator(prev, curr, period_s) -> f64`
  - Added explicit FLL → PLL handoff API:
    `Fll::complete_handoff() -> f64`, with `update()` becoming no-op after `PllLock`
  - Added automatic bandwidth scheduling:
    wide bandwidth at start, narrow bandwidth in lock, with integrator preservation
    via `set_integrator`
  - Reduced FLL state machine to three states only:
    `Searching / FllLock / PllLock`
  - Added transient-response tests for ±3 kHz initial frequency error
  - Added integration sanity test with real `correlator_epl` Prompt output
- **carrier tracking (PLL/FLL)**
  - Полностью заменён черновой `tracking/pll.rs` (упрощённый q·sign(I) дискриминатор
    без фильтра и без FLL) на полноценную PLL/FLL архитектуру уровня GNSS receiver.
  - Добавлен FLL-assisted PLL контур:
    - FLL на основе cross-product дискриминатора частоты (`fll_cross_product_discriminator`)
    - широкий диапазон захвата в `FllLock`
    - переход в PLL при стабильной частотной ошибке (`fll_to_pll_stable_epochs`)
  - Добавлен PLL Costas loop (decision-directed):
    - используется `EplOutput::pll_dd_atan()` (atan(Q/I), устранение 180°-неоднозначности
      навигационного бита)
    - подтверждается тестом `test_pll_lock_costas_removes_180_degree_bit_ambiguity`
  - Реализован петлевой фильтр 3-го порядка (`PllLoopFilter`):
    - коэффициенты `a1, a2, a3` вычисляются через шумовую полосу `Bₗ`
    - используется модель Уильямса (ωₙ = Bₗ / 0.7845)
    - поддерживает фазу + частоту + ускорение частоты (jerk-robust tracking)
  - Добавлен coherent accumulation (`CoherentAccumulator`):
    - интеграция 1–20 мс
    - защита от выхода за диапазон
    - выдача дискриминатора только после накопления окна
  - Добавлена детекция loss-of-lock (`LockDetector`):
    - анализ дисперсии фазовой ошибки в скользящем окне
    - контроль C/N₀ через `cn0_estimate`
    - переход в `LockLost` при деградации сигнала
  - Добавлены состояния трекинга:
    - `Searching`
    - `FllLock`
    - `PllLock`
    - `LockLost`
  - Добавлены метрики бенчмарков:
    - `time_to_lock_ms` — время захвата PLL
    - `steady_state_phase_error_rad` — σ фазовой ошибки
    - `cn0_db_hz` — оценка C/N₀
  - Добавлена возможность повторного захвата через `Pll::reset(initial_doppler_hz)`
    без пересоздания объекта
  - Добавлена интеграция с существующим correlator:
    - прямое использование `correlator_epl` (E/P/L)
    - тестирование PLL на реальном Prompt выходе коррелятора
- **tracking**
  - Добавлен DLL (`dll.rs`) для отслеживания фазы PRN-кода.
  - Добавлен выбор типа дискриминатора через `DllDiscriminatorKind`:
    - `Nelp` использует `EplOutput::dll_nelp()`
    - `Ele` использует `EplOutput::dll_ele()`
  - Добавлена нормализация выхода дискриминатора в чипы через `discriminate()` с
    делением на `2 × half_chip_spacing`.
  - Добавлен петлевой фильтр второго порядка (`DllLoopFilter`) с расчётом `tau1`
    и `tau2` по формулам из `docs/TRACKING.md`.
  - Добавлены интеграционные тесты с реальным PRN-кодом через `correlator_epl` и
    `make_epl_replicas`.

### Changed

- FLL now narrows bandwidth automatically after consecutive stable epochs and
  preserves loop continuity during wide → narrow transition
- FLL handoff to PLL is now explicit and deterministic through
  `ready_for_pll` / `complete_handoff()`
- Упрощён поток FLL→PLL переключения:
  - интегратор частоты сохраняется при переходе (`switch_to_pll`)
  - исключён скачок частоты при смене полосы фильтра
- Обновлён `PllLoopFilter`:
  - единая реализация фильтра используется и для FLL, и для PLL (разная полоса)
- Усилен контроль численной стабильности:
  - проверка `is_finite()` в длительных прогонных тестах PLL
- Улучшена структура тестов:
  - добавлены тесты на Costas lock, loss-of-lock, устойчивость и интеграцию с коррелятором

### Fixed

- Исправлена формула обновления интегратора в DLL: `e · T / τ₁` вместо
  `e · T · τ₁`.
- Приведены комментарии и тесты петлевого фильтра в соответствие с формулами трекинга.
- Исправлена логика перехода FLL→PLL: теперь переключение происходит только при
  N стабильных эпохах
- Исправлено отсутствие сохранения интегратора при переходе в PLL (устранён
  frequency jump)
- Исправлена и стабилизирована детекция loss-of-lock на основе корректной выборки
  окна фазовой ошибки
- Приведены тесты PLL в соответствие с моделью coherent accumulation (1–20 ms windowing)

## [0.2.0] - 2026-06-19

### Added

- **acquisition**
  - в `verifier.rs` в ф-ях `precompute_prn()` и `precompute_all()` убрал лишниее
    клонирование
  - в ф-ии `single_verify()` исправил сравнение при получении `doppler_ok` с `doppler_fine_hz`
    на `doppler_coarse_hz`. Потому что второй проход строится вокруг: `doppler_coarse_hz`,
    а не `doppler_fine_hz`. Именно coarse является центром narrow-search.
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
- **acquisition**
  - Параллельный поиск — теперь реально подключён. Раньше: `parallel_search`
    существовала как функция рядом с кодом, но `Receiver::run_acquisition_epoch`
    гонял PRN в обычном `for-цикле` через `self.verifier.verify_prn(...)`.
    Теперь Логика двойной верификации вынесена в чистую функцию `verify_prn_pure(signal,
prn, block_size, sample_rate_hz, config, cache)` — без `&mut self`, без побочных
    эффектов, безопасна для вызова из разных потоков Rayon одновременно.
    `verify_all_parallel()` — публичная функция, гоняет `verify_prn_pure` через
    `prns.par_iter().map(...) (за #[cfg(feature = "rayon")]`, с идентичным
    `sequential fallback` без фичи).
    `Receiver::run_acquisition_epoch` теперь вызывает именно её (строка с
    `let parallel_output = verify_all_parallel(...))`, а не последовательный цикл.
    Добавлен тест `acquisition_epoch_searches_all_32_prns`, который проверяет, что
    за одну эпоху реально проверяются все 32 PRN одним параллельным вызовом.
  - Метрика false positive — переименована и задокументирована честно
    - `false_alarm_rate()` помечена `#[deprecated]` с явным объяснением, почему
      имя было обманчивым: у нас нет ground truth о том, был ли спутник реально
      в небе. Новый метод marginal_rate() — то же вычисление
      (marginal / (confirmed + marginal)), но названо в соответствии с тем, что
      оно на самом деле измеряет: долю случаев, когда широкий проход дал пик, а
      узкий уточняющий не подтвердил.
    - Старый метод оставлен как алиас для обратной совместимости, но с предупреждением
      компилятора при использовании.
  - Добавлено для связности:
    - `VerifierStats::merge()` — объединение статистики (нужно для агрегации
      параллельного прогона в общий счётчик `Receiver`)
    - `AcquisitionVerifier::absorb_stats()` — `Receiver` теперь вызывает это после
      каждой параллельной эпохи, поэтому acquisition_stats().verifier_stats.
      `total_calls` реально показывает 32 после первой эпохи, а не 0 (это покрыто
      тестом `acquisition_summary_reflects_parallel_stats`)

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
