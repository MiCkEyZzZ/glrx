# Changelog

All notable changes to **Glrx** are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] — 0000-00-00

### Added

**.github**:

- добавлен CODEOWNERS
- добавлен cargo-blacklist.txt

**iq_source**:

- `IqSource` trait и `IqBlock` для унифицированного чтения IQ-сэмплов.

**file**:

- `FileSource` для чтения IQ из бинарных файлов (int8, int16, float32) с нормализацией.

**sdr**:

- `MockSdrSource` для тестов и CI без реального SDR.
- Шаблон `SoapySource` для работы с SDR через SoapySDR (подключение RTL-SDR, HackRF,
  USRP и др.).

**config**:

- `RfConfig` для настройки частоты дискретизации, центральной частоты и усиления.

**metrics**:

- Метрики потока (`SourceMetrics`): total samples, dropped samples, measured rate,
  interruptions, power estimate.
- Тесты для чтения файлов, нормализации сэмплов и проверки метрик.

**docs**:

- добавил описание `ARCHITECTURE`
- добавил описание `DSP`
- добавил описание `NAVIGATION`
- добавил описание `PIPELINE`
- добавил описание `TRACKING`

### Changed

- Комментарии и документация переведены на русский язык для большей наглядности.
