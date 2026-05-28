# The GLRX development commands (Justfile)
#
# Purpose:
#   Unified interface for formatting, linting, testing, documentation,
#   embedded validation, feature-matrix checks, and CI simulation.
#
# Usage:
#   just <recipe>

set shell := ["bash", "-ceuo", "pipefail"]

# =============================================================================
# Default
# =============================================================================

default:
    just help

help:
    @just --list

# =============================================================================
# Formatting
# =============================================================================

fmt:
    cargo fmt --all
    taplo fmt

fmt-toml:
    taplo fmt

fmt-all: fmt fmt-toml

fmt-check:
    cargo fmt --all -- --check
    taplo fmt --check

# =============================================================================
# Cargo checks
# =============================================================================

check:
    cargo check --workspace --all-targets --locked

check-all-features:
    cargo check --workspace --all-features --locked

check-std:
    cargo check --workspace --features --locked

# =============================================================================
# Linting
# =============================================================================

lint:
    cargo clippy --workspace --all-targets --all-features --locked -- -D warnings

# =============================================================================
# Documentation
# =============================================================================

doc:
    RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps --all-features --locked

docsrs:
    RUSTDOCFLAGS="--cfg docsrs -D warnings" cargo +nightly doc --workspace --all-features --no-deps

# =============================================================================
# Tests
# =============================================================================
#
# Run the full test suite (unit + integration + determenistic property tests).
# proptest-based tests in prop_test.rs are compiled automatically on host

test:
    cargo test --workspace --all-features --locked

# =============================================================================
# Advanced validation
# =============================================================================

deny:
    cargo deny check

audit:
    cargo audit

release-check:
    cargo publish --dry-run

# =============================================================================
# Cleanup
# =============================================================================

clean:
    cargo clean
