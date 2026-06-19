# Структура проекта

```text
glrx
├── .cargo
│   └── config.toml
├── .config
│   ├── config.toml
│   └── nextest.toml
├── .github
│   ├── DISCUSSION_TEMPLATE
│   │   └── feature-requests.yml
│   ├── ISSUE_TEMPLATE
│   │   ├── bug_report.yml
│   │   ├── config.yml
│   │   └── enhancement.yml
│   ├── workflows
│   │   └── semantic-pull-request.yml
│   ├── cargo-blacklist.txt
│   ├── CODEOWNERS
│   ├── dependabot.yml
│   ├── FUNDING.yml
│   └── pull_request_template.md
├── benches
│   ├── benches
│   ├── target
│   ├── Cargo.lock
│   ├── Cargo.toml
│   └── README.md
├── docs
│   ├── DSP.md
│   ├── NAVIGATION.md
│   ├── PIPELINE.md
│   ├── PROJECT_STRUCTURE.md
│   ├── ROADMAP.md
│   └── TRACKING.md
├── examples                       # примеры использования ресивера
│   ├── src
│   ├── target
│   ├── Cargo.lock
│   ├── Cargo.toml
│   └── README.md
├── scripts                        # вспомогательные скрипты (сборка, тесты)
├── src
│   ├── acquisition                # поиск спутников
│   │   ├── correlator.rs
│   │   ├── detector.rs
│   │   ├── fft_search.rs
│   │   ├── mod.rs
│   │   └── verifier.rs
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
│   │   │   ├── discriminators.rs
│   │   │   ├── mod.rs
│   │   │   └── normalisation.rs
│   │   ├── block.rs
│   │   ├── fft.rs
│   │   ├── filter.rs
│   │   ├── mixer.rs
│   │   ├── mod.rs
│   │   ├── prn_code.rs
│   │   └── resampler.rs
│   ├── tracking
│   │   ├── dll.rs
│   │   └── mod.rs
│   ├── lib.rs                     # публичная библиотека
│   └── main.rs                    # CLI / entrypoint
├── tests
│   ├── fixtures
│   │   ├── expected
│   │   ├── iq
│   │   └── nav
│   ├── integration
│   └── system
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
├── justfile
├── LICENSE.APACHE
├── LICENSE.MIT
├── README.md
├── rust-toolchain.toml
├── rustfmt.toml
├── SECURITY.md
└── taplo.tomls
```
