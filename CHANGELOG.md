# Changelog

All notable changes to **Glrx** are documented in this file.

## [Unreleased] — 00-00-0000

### Added

- `IqSource` trait и `IqBlock` для унифицированного чтения IQ-сэмплов.
- `FileSource` для чтения IQ из бинарных файлов (int8, int16, float32) с нормализацией.
- `MockSdrSource` для тестов и CI без реального SDR.
- Шаблон `SoapySource` для работы с SDR через SoapySDR (подключение RTL-SDR, HackRF, USRP и др.).
- `RfConfig` для настройки частоты дискретизации, центральной частоты и усиления.
- Метрики потока (`SourceMetrics`): total samples, dropped samples, measured rate, interruptions, power estimate.
- Тесты для чтения файлов, нормализации сэмплов и проверки метрик.

### Changed

- Комментарии и документация переведены на русский язык для большей наглядности.
