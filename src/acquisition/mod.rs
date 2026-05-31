//! Acquisition layer — satellite signal search.
//!
//! # Algorithm
//!
//! GLRX uses the **Parallel Code Search (PCPS)** algorithm:
//!
//! ```text
//! for each Doppler trial f_d:
//!   wiped  = signal × exp(−j·2π·f_d·t)
//!   power  = |IFFT(FFT(wiped) × conj(FFT(prn)))|²
//!   peak   = argmax(power)  →  code_phase
//! ```
//!
//! The 2D search surface (Doppler × `code_phase`) is evaluated for each PRN
//! and the strongest peak above a detection threshold is declared acquired.

pub mod correlator;
