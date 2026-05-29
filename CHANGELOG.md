# Changelog

All notable changes to **GLRX** are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] — 0000-00-00

### Added

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
  - `FileSource` для чтения IQ из бинарных файлов (int8, int16, float32) с нормализацией.

- **sdr**:
  - `MockSdrSource` для тестов и CI без реального SDR.
  - Шаблон `SoapySource` для работы с SDR через SoapySDR (подключение RTL-SDR, HackRF,
    USRP и др.).

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
