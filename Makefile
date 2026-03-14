# The Glrx Makefile

BUILD_TARGET := $(shell test -f .cargo/config.toml && grep -E '^\s*target\s*=' .cargo/config.toml | head -1 | cut -d'"' -f2)
TARGET_ARG   := $(if $(BUILD_TARGET),--target $(BUILD_TARGET),)
TARGET_DIR   := target/$(if $(BUILD_TARGET),$(BUILD_TARGET)/,)

##@ Build
.PHONY: build build-release
build: ## Сборка debug
	cargo build $(TARGET_ARG)

build-release: ## Сборка релизной версии
	cargo build --release $(TARGET_ARG)

##@ Test
.PHONY: check clippy clippy-ci nextest test miri miri-test test-all nextest-all

check: ## Cargo проверка
	cargo check

clippy: ## Clippy (рассматривать предупреждения как ошибки)
	cargo clippy -- -D warnings

clippy-ci: ## Clippy как в CI: все таргеты и все фичи, warnings -> error
	cargo clippy --all-targets --all-features -- -D warnings

# Параметризованный тест - запускает тесты корневого проекта или конкретного крейта
test: ## Запуск тестов. Использование: make test [CRATE=glos-core]
ifdef CRATE
	@echo "Running tests for crate: $(CRATE)"
	cargo test -p $(CRATE)
else
	@echo "Running tests for root project"
	cargo test
endif

# Параметризованный nextest
nextest: ## Nextest. Использование: make nextest [CRATE=glos-core]
ifdef CRATE
	@echo "Running nextest for crate: $(CRATE)"
	cargo nextest run -p $(CRATE)
else
	@echo "Running nextest for root project"
	cargo nextest run
endif

test-all: ## Полный набор тестов (root + все подкрейты)
	@echo "Running all tests (root + crates)..."
	cargo test
	@for c in $(CRATES); do \
		echo ""; \
		echo "Running tests for crate: $$c"; \
		cargo test -p $$c; \
	done

# Запуск nextest везде
nextest-all: ## Nextest везде (root + все подкрейты)
	@echo "Running nextest everywhere (root + crates)..."
	cargo nextest run
	@for c in $(CRATES); do \
		echo ""; \
		echo "Running nextest for crate: $$c"; \
		cargo nextest run -p $$c; \
	done

miri: ## Запустите все тесты в Miri
	cargo miri test

miri-test: ## Запустите определенный тест в Miri. Использование: make miri-test TEST="модуль::имя_теста"
	cargo miri test $(TEST)
