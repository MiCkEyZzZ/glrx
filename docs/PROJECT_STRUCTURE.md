# Project structure

```text
glrx
├── docs
│   ├── ARCHITECTURE.md
│   ├── DSP.md
│   ├── INTEGRATION.md
│   ├── NAVIGATION.md
│   ├── PIPELINE.md
│   ├── PROJECT_STRUCTURE.md
│   ├── QUICK_START.md
│   ├── ROADMAP.md
│   └── TRACKING.md
├── examples                       # примеры использования ресивера
├── scripts                        # вспомогательные скрипты (сборка, тесты)
├── src
│   ├── acquisition                # поиск спутников
│   │   ├── correlator.rs
│   │   ├── detector.rs
│   │   ├── fft_search.rs
│   │   └── mod.rs
│   ├── config
│   │   ├── receiver_config.rs      # конфигурация ресивера
│   │   └── mod.rs
│   ├── interfaces                  # абстракции для интеграции с другими системами
│   │   ├── mod.rs
│   │   ├── glos.rs                 # чтение/запись .glos
│   │   ├── glint.rs                # отправка observables
│   │   └── usmet.rs                # сохранение телеметрии
│   ├── navigation                  # декодирование навигационных сообщений
│   │   ├── mod.rs
│   │   ├── ephemeris.rs
│   │   ├── frame_decoder.rs
│   │   └── nav_data.rs
│   ├── observables                # измерения (CN0, Doppler, псевдодальности)
│   │   ├── mod.rs
│   │   ├── pseudorange.rs
│   │   ├── doppler.rs
│   │   └── cn0.rs
│   ├── output                     # вывод данных / интерфейсы
│   │   ├── mod.rs
│   │   ├── nmea.rs
│   │   ├── ubx.rs
│   │   └── telemetry.rs           # интеграция с GLINT/USMET
│   ├── pipeline                   # high-level pipeline
│   │   ├── mod.rs
│   │   └── receiver.rs            # соединяет acquisition → tracking → solver
│   ├── rf                         # IQ потоки / SDR / файлы
│   │   ├── config.rs
│   │   ├── error.rs
│   │   ├── file.rs
│   │   ├── format.rs
│   │   ├── iq_source.rs
│   │   ├── metrics.rs
│   │   ├── mod.rs
│   │   ├── normalise.rs
│   │   ├── sdr.rs
│   │   └── stream.rs
│   ├── signal                     # базовая обработка сигналов (DSP)
│   │   ├── correlator
│   │   │   ├── base.rs
│   │   │   ├── code_utilities.rs
│   │   │   ├── correlator_utils.rs
│   │   │   ├── mod.rs
│   │   │   └── normalisation.rs
│   │   ├── fft.rs
│   │   ├── filter.rs
│   │   ├── mixer.rs
│   │   ├── mod.rs
│   │   └── resampler.rs
│   ├── solver                     # вычисление позиции / навигационное решение
│   │   ├── mod.rs
│   │   ├── least_squares.rs
│   │   └── kalman.rs
│   ├── tracking                   # tracking loops
│   │   ├── mod.rs
│   │   ├── dll.rs
│   │   ├── pll.rs
│   │   ├── fll.rs
│   │   └── channel.rs             # channel abstraction
│   ├── utils                      # вспомогательные функции
│   │   ├── mod.rs
│   │   ├── logger.rs
│   │   └── timing.rs
│   ├── lib.rs                     # публичная библиотека
│   └── main.rs                    # CLI / entrypoint
├── tests
├── .editorconfig
├── .gitignore
├── AUTHOR
├── BUGS
├── Cargo.lock
├── Cargo.toml
├── CHANGELOG
├── clippy.toml
├── CODE_OF_CONDUCT.md
├── CONTRIBUTING.md
├── deny.toml
├── INSTALL
├── LICENSE.APACHE
├── LICENSE.MIT
├── Makefile
├── README.md
├── rust-toolchain.toml
├── rustfmt.toml
├── SECURITY.md
└── taplo.tomls
```
