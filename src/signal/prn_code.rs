//! PRN (Pseudo-Random Noise) code generation for GNSS signals.

use std::collections::HashMap;

/// G2 tap pairs for PRN 1-32. Index 0 -> PRN 1.
/// Format: (tap_a, tap_b) where bits are numbered 1-20.
const G2_TAPS: [(u8, u8); 32] = [
    (2, 6),  // PRN  1
    (3, 7),  // PRN  2
    (4, 8),  // PRN  3
    (5, 9),  // PRN  4
    (1, 9),  // PRN  5
    (2, 10), // PRN  6
    (1, 8),  // PRN  7
    (2, 9),  // PRN  8
    (3, 10), // PRN  9
    (2, 3),  // PRN 10
    (3, 4),  // PRN 11
    (5, 6),  // PRN 12
    (6, 7),  // PRN 13
    (7, 8),  // PRN 14
    (8, 9),  // PRN 15
    (9, 10), // PRN 16
    (1, 4),  // PRN 17
    (2, 5),  // PRN 18
    (3, 6),  // PRN 19
    (4, 7),  // PRN 20
    (5, 8),  // PRN 21
    (6, 9),  // PRN 22
    (1, 3),  // PRN 23
    (4, 6),  // PRN 24
    (5, 7),  // PRN 25
    (6, 8),  // PRN 26
    (7, 9),  // PRN 27
    (8, 10), // PRN 28
    (1, 6),  // PRN 29
    (2, 7),  // PRN 30
    (3, 8),  // PRN 31
    (4, 9),  // PRN 32
];

/// Length of GPS L1 C/A code in chips.
pub const GPS_CODE_LENGTH: usize = 1023;

/// GPS chip rate in chips/second.
pub const GPS_CHIP_RATE_HZ: f64 = 1_023_000.0;

/// Support GNSS constellations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GnssSystem {
    /// GPS (USA) - L1 C/A, 1023-chip Gold codes, chip rate 1.023 Mchip/s
    Gps,

    /// GLONASS (Russia) - FDMA, 511-chip M-sequence. Not yet implemented
    Glonass,

    /// Galileo (EU) - E1 OS primary code. Not yet implemented
    Galileo,
}

/// Pre-computed PRN code cache.
///
/// Generates all GPS codes once at construction and stores them.
/// Subsequent calls to `get_gps` or `resample_gps` are allocation-free
/// (except for the returned resampled vector).
pub struct PrnCodeCache {
    gps: HashMap<u8, Vec<i8>>,
}

impl PrnCodeCache {
    /// Build the cache for GPS PRN 1-32.
    ///
    /// Generation is O(32 * 1023) ≈ 33k operations — negligible at startup
    pub fn new() -> Self {
        let mut gps = HashMap::with_capacity(32);

        for prn in 1u8..=32 {
            gps.insert(prn, generate_gps_ca(prn));
        }

        Self { gps }
    }

    /// Return the ±1 chip code for GPS `prn` (1-32).
    pub fn get_gps(
        &self,
        prn: u8,
    ) -> Option<&[i8]> {
        self.gps.get(&prn).map(Vec::as_slice)
    }

    /// GLONASS ranging code - **not yet implemented**.
    ///
    /// GLONASS uses 511-chip M-sequences (x⁹ + x⁵ + 1) identical for all
    /// satellites (FDMA distinguishes satellites by carrier frequency).
    pub fn get_glonass(
        &self,
        _slot: u8,
    ) -> Option<&[i8]> {
        unimplemented!() // TODO GLRX-3 extension: implement GLONASS M-sequence
    }

    /// Galileo E1 OS primary code — **not yet implemented**.
    ///
    /// Galileo uses memory codes (not shift-register generated) defined
    /// in the Galileo OS SIS ICD.
    pub fn get_galileo_e1(
        &self,
        _svid: u8,
    ) -> Option<&[i8]> {
        unimplemented!() // TODO GLRX-3 extension: implement Galileo E1B/E1C codes
    }

    /// Resample GPS `prn` to exactly `n_samples` per code period.
    ///
    /// Uses **nearest-neighbour** chip selection:
    ///
    /// ```text
    /// chip_index = floor(sample_index × 1023 / n_samples)
    /// ```
    ///
    /// This is the correct approach for GNSS acquisition where the
    /// resampled code will be correlated against an IQ block of the same
    /// length.
    ///
    /// # Returns
    ///
    /// `None` if `prn` is not in 1..=32.
    pub fn resample_gps(
        &self,
        prn: u8,
        n_samples: usize,
    ) -> Option<Vec<f32>> {
        let chips = self.get_gps(prn)?;
        let n_chips = GPS_CODE_LENGTH;
        let out = (0..n_samples)
            .map(|i| {
                let chip_idx = (i * n_chips) / n_samples;

                chips[chip_idx] as f32
            })
            .collect();

        Some(out)
    }

    /// Resample GPS `prn` with sub-chip resolution using the current
    /// **code phase offset** `phase_offset_chips` (fractional).
    ///
    /// # Arguments
    ///
    /// * `prn` — satellite PRN 1–32
    /// * `n_samples` — number of output samples
    /// * `phase_offset_chips` — fractional chip offset to apply (0.0–1023.0)
    ///
    /// Wraps around modulo `GPS_CODE_LENGTH`.
    pub fn resample_gps_with_phase(
        &self,
        prn: u8,
        n_samples: usize,
        phase_offset_chip: f64,
    ) -> Option<Vec<f32>> {
        let chips = self.get_gps(prn)?;
        let n_chips = GPS_CODE_LENGTH as f64;
        let out = (0..n_samples)
            .map(|i| {
                let chip_f = i as f64 * n_chips / n_samples as f64 + phase_offset_chip;
                let chip_idx = chip_f.floor() as usize % GPS_CODE_LENGTH;

                chips[chip_idx] as f32
            })
            .collect();

        Some(out)
    }
}

impl Default for PrnCodeCache {
    fn default() -> Self {
        Self::new()
    }
}

/// Generate the GPS L1 C/A Gold code for `prn` (1-indexed, 1-32).
///
/// Returns a `Vec<i8>` of length 1023 with values ±1.
///
/// # Algorithm
///
/// 1. Initialise G1 and G2 registers to all-ones.
/// 2. For each chip: output = G1[10] XOR G2[tap_a] XOR G2[tap_b]
/// 3. Advance both registers using their feedback polynomials.
/// 4. Convert 0 → +1, 1 → −1 (NRZ encoding).
///
/// # Panic
///
/// Panics if `prn < 1 || prn > 32`
pub fn generate_gps_ca(prn: u8) -> Vec<i8> {
    assert!(
        prn >= 1 && prn <= 32,
        "GPS PRN must be in range 1..=32, got {}",
        prn
    );

    let (tap_a, tap_b) = G2_TAPS[(prn - 1) as usize];
    let ta = (tap_a - 1) as usize; // convert 1-indexed to 0-indexed
    let tb = (tap_b - 1) as usize;

    // Registers: bit 0 = stage 1 (first input), bit 9 = stage 10 (output)
    let mut g1: [u8; 10] = [1; 10];
    let mut g2: [u8; 10] = [1; 10];
    let mut code = Vec::with_capacity(GPS_CODE_LENGTH);

    for _ in 0..GPS_CODE_LENGTH {
        // C/A chip = G1[10] XOR G2[tap_a] XOR G2[tap_b]
        let chip = g1[9] ^ g2[ta] ^ g2[tb];
        // NRZ: 0 -> +1, -> -1
        code.push(if chip == 0 { 1i8 } else { -1i8 });

        // Advance G1: feedback from taps 3 and 10 (1-indexed -> 2, 9)
        let g1_fb = g1[2] ^ g1[9];

        g1.rotate_right(1);
        g1[0] = g1_fb;

        //  Advance G2: feedback from taps 2, 3, 6, 8, 9, 10 (-> 1, 2, 5, 7, 8, 9)
        let g2_fb = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9];

        g2.rotate_right(1);
        g2[0] = g2_fb;
    }

    code
}

/// Compute the **circular auto-correlation** of a ±1 code at lag 0.
///
/// For a maximal-length code of length N, the auto-correlation is N at
/// lag 0 and -1 all other lags. Gold codes are slightly worse (-1 or -64 for
/// GPS C/A) but follow the same principle.
pub fn autocorrelation_at_zero(code: &[i8]) -> i64 {
    code.iter().map(|&c| c as i64 * c as i64).sum()
}

/// Compute the full circular **cross-correlation** between two ±1 codes.
///
/// Returns a vector of length `code_a.len()` where index `k` is the
/// correlation at lag `k` chips.
///
/// # Panics
///
/// Panics if `code_a.len() != code_b.len()`.
pub fn circular_cross_correlation(
    code_a: &[i8],
    code_b: &[i8],
) -> Vec<i32> {
    let n = code_a.len();

    assert_eq!(n, code_b.len(), "codes must have equal length");

    (0..n)
        .map(|lag| {
            (0..n)
                .map(|i| code_a[i] as i32 * code_b[(i + lag) % n] as i32)
                .sum()
        })
        .collect()
}

/// Compute autocorrelation for a single lag efficiently.
///
/// Equivalent to one element of `circular_cross_correlation(code, code)`.
pub fn autocorrelation_at_lag(
    code: &[i8],
    lag: usize,
) -> i32 {
    let n = code.len();
    (0..n)
        .map(|i| code[i] as i32 * code[(i + lag) % n] as i32)
        .sum()
}
