# =============================================================================
# GLRX Development Commands
# =============================================================================
#
# Unified interface for:
# - formatting
# - linting
# - testing
# - documentation
# - embedded / no_std validation
# - dependency auditing
# - CI simulation
#
# Usage:
#   just <recipe>
#
# Examples:
#   just check
#   just test
#   just ci
#   just bench
#
# =============================================================================

set shell := ["bash", "-ceuo", "pipefail"]

# =============================================================================
# Default
# =============================================================================

default:
    just help

help:
    @just --list

# =========================================================
# Build modes
# =========================================================

build:
    cargo build --workspace

build-release:
    cargo build --release

build-perf:
    cargo build --profile perf

build-native:
    RUSTFLAGS="-C target-cpu=native" cargo build --release

# =============================================================================
# Formatting
# =============================================================================

fmt:
    cargo fmt --all
    taplo fmt

fmt-rust:
    cargo fmt --all

fmt-toml:
    taplo fmt

fmt-check:
    cargo fmt --all -- --check
    taplo fmt --check

toml-check:
    taplo check

# =============================================================================
# Cargo checks
# =============================================================================

check:
    cargo check --workspace --all-targets --locked

check-all-features:
    cargo check --workspace --all-features --all-targets --locked

check-no-std:
    cargo check --workspace --no-default-features --locked

# =============================================================================
# Linting
# =============================================================================

lint:
    cargo clippy \
        --workspace \
        --all-targets \
        --all-features \
        --locked \
        -- \
        -D warnings

clippy-pedantic:
    cargo clippy --all-targets --all-features -- \
        -W clippy::pedantic

# =============================================================================
# Documentation
# =============================================================================

doc:
    RUSTDOCFLAGS="-D warnings" \
    cargo doc --workspace --all-features --no-deps --locked

docsrs:
    RUSTDOCFLAGS="--cfg docsrs -D warnings" \
    cargo +nightly doc \
        --workspace \
        --all-features \
        --no-deps \
        --locked

# =============================================================================
# Tests
# =============================================================================
#
# Runs:
# - unit tests
# - integration tests
# - deterministic property tests
#
# Property tests are compiled automatically on host targets.
#
# =============================================================================

test:
    cargo test --workspace --locked

test-all-features:
    cargo test --workspace --all-features --locked

test-release:
    cargo test --workspace --release --locked

test-doc:
    cargo test --doc --workspace

next:
    cargo nextest run --workspace

# =============================================================================
# Benchmarks & Validation
# =============================================================================

bench:
    cargo +nightly bench --workspace

miri:
    cargo +nightly miri test --workspace

# =============================================================================
# Dependency & Security Checks
# =============================================================================

deny:
    cargo deny check

unused-deps:
    cargo machete

# =============================================================================
# Release Validation
# =============================================================================

release-check:
    cargo publish --dry-run

# =============================================================================
# CI Aggregate
# =============================================================================

ci:
    just fmt-check
    just toml-check
    just lint
    just check
    just test
    just doc

# =============================================================================
# Cleanup
# =============================================================================

clean:
    cargo clean
