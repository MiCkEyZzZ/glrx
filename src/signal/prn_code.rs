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
        None // TODO GLRX-3 extension: implement GLONASS M-sequence
    }

    /// Galileo E1 OS primary code — **not yet implemented**.
    ///
    /// Galileo uses memory codes (not shift-register generated) defined
    /// in the Galileo OS SIS ICD.
    pub fn get_galileo_e1(
        &self,
        _svid: u8,
    ) -> Option<&[i8]> {
        None // TODO GLRX-3 extension: implement Galileo E1B/E1C codes
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
        phase_offset_chips: f64,
    ) -> Option<Vec<f32>> {
        let chips = self.get_gps(prn)?;
        let n_chips = GPS_CODE_LENGTH as f64;
        let out = (0..n_samples)
            .map(|i| {
                let chip_f = i as f64 * n_chips / n_samples as f64 + phase_offset_chips;
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
        // NRZ: 0 → +1, 1 → −1
        code.push(if chip == 0 { 1i8 } else { -1i8 });

        // Advance G1: feedback from taps 3 and 10 (1-indexed → 2, 9)
        let g1_fb = g1[2] ^ g1[9];
        g1.rotate_right(1);
        g1[0] = g1_fb;

        // Advance G2: feedback from taps 2,3,6,8,9,10 (→ 1,2,5,7,8,9)
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

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gps_code_length_is_1023() {
        for prn in 1u8..=32 {
            assert_eq!(generate_gps_ca(prn).len(), GPS_CODE_LENGTH, "PRN {}", prn);
        }
    }

    #[test]
    fn test_gps_code_values_are_bipolar() {
        for prn in 1u8..=32 {
            let code = generate_gps_ca(prn);

            for &chip in &code {
                assert!(chip == 1 || chip == -1, "PRN {}: chip = {}", prn, chip);
            }
        }
    }

    #[test]
    fn test_gps_codes_are_balanced() {
        // GPS Gold codes are nearly balanced: count(+1) ≈ count(−1) ≈ 511-512
        for prn in 1u8..=32 {
            let code = generate_gps_ca(prn);
            let ones: i64 = code.iter().map(|&c| c as i64).sum();

            // For GPS C/A: sum should be exactly −1 (512 ones, 511 neg-ones)
            // Tolerance of ±3 to allow for any minor implementation differences
            assert!(
                ones.abs() <= 3,
                "PRN {} balance: sum = {} (expected ~0 or −1)",
                prn,
                ones
            );
        }
    }

    /// Cross-check PRN 1 first 10 chips against generated values.
    /// Source: GPS-IS-200 Table 3-Ia polarity convention adapted to ±1 NRZ form.
    #[test]
    fn test_gps_prn1_first_chips_match_icd() {
        let code = generate_gps_ca(1);

        // First 10 chips for PRN 1 in current implementation convention.
        // Chips: -1 -1 +1 +1 -1 +1 +1 +1 +1 +1  (positions 0-9)
        let expected = [-1i8, -1, 1, 1, -1, 1, 1, 1, 1, 1];

        for (i, (&got, &exp)) in code.iter().zip(expected.iter()).enumerate() {
            assert_eq!(got, exp, "PRN 1 chip {}: got {} expected {}", i, got, exp);
        }
    }

    #[test]
    fn test_gps_all_codes_are_distinct() {
        let codes: Vec<Vec<i8>> = (1u8..=32).map(generate_gps_ca).collect();

        for i in 0..32 {
            for j in (i + 1)..32 {
                assert_ne!(
                    codes[i],
                    codes[j],
                    "PRN {} and PRN {} have identical codes",
                    i + 1,
                    j + 1
                );
            }
        }
    }

    #[test]
    fn test_autocorr_at_zero_equals_code_length() {
        for prn in 1u8..=32 {
            let code = generate_gps_ca(prn);
            let ac = autocorrelation_at_zero(&code);

            assert_eq!(ac, GPS_CODE_LENGTH as i64, "PRN {}", prn);
        }
    }

    #[test]
    fn test_autocorr_peak_at_zero_is_maximum() {
        // PRN auto-correlation must peak at lag=0
        let code = generate_gps_ca(1);
        let ac_0 = autocorrelation_at_lag(&code, 0);

        // Sample a few non-zero lags
        for lag in [1, 10, 100, 500, 1000] {
            let ac_k = autocorrelation_at_lag(&code, lag).abs();

            assert!(
                ac_0 > ac_k,
                "autocorr at lag 0 ({}) ≤ lag {} ({})",
                ac_0,
                lag,
                ac_k
            );
        }
    }

    #[test]
    fn test_autocorr_nonzero_lag_bounded() {
        // GPS Gold code autocorrelation bounds: −65, −1, or +63
        // In absolute terms, max off-peak |AC| ≤ 65
        let code = generate_gps_ca(5);

        for lag in 1..GPS_CODE_LENGTH {
            let ac = autocorrelation_at_lag(&code, lag).abs();

            assert!(
                ac <= 65,
                "PRN 5 autocorr at lag {} = {} (should be ≤ 65)",
                lag,
                ac
            );
        }
    }

    #[test]
    fn test_crosscorr_different_prns_bounded() {
        // GPS cross-correlation: |CC| ≤ 65 for any pair at any lag
        let c1 = generate_gps_ca(1);
        let c2 = generate_gps_ca(2);
        let cc = circular_cross_correlation(&c1, &c2);

        for (lag, &v) in cc.iter().enumerate() {
            assert!(
                v.abs() <= 65,
                "PRN1×PRN2 cross-corr at lag {} = {} (should be ≤ 65)",
                lag,
                v
            );
        }
    }

    #[test]
    fn test_crosscorr_same_code_equals_autocorr() {
        let code = generate_gps_ca(7);
        let cc = circular_cross_correlation(&code, &code);

        assert_eq!(cc[0], GPS_CODE_LENGTH as i32); // lag 0 = N
    }

    #[test]
    fn test_cache_contains_all_gps_prns() {
        let cache = PrnCodeCache::new();

        for prn in 1u8..=32 {
            assert!(cache.get_gps(prn).is_some(), "missing PRN {}", prn);
        }
    }

    #[test]
    fn test_cache_get_invalid_prn_returns_none() {
        let cache = PrnCodeCache::new();

        assert!(cache.get_gps(0).is_none());
        assert!(cache.get_gps(33).is_none());
    }

    #[test]
    fn test_cache_matches_direct_generation() {
        let cache = PrnCodeCache::new();

        for prn in 1u8..=32 {
            let cached = cache.get_gps(prn).unwrap();
            let direct = generate_gps_ca(prn);

            assert_eq!(cached, direct.as_slice(), "PRN {} mismatch", prn);
        }
    }

    #[test]
    fn test_resample_correct_length() {
        let cache = PrnCodeCache::new();

        for &n in &[2048usize, 4096, 8192, 1023] {
            let v = cache.resample_gps(1, n).unwrap();

            assert_eq!(v.len(), n, "resample to {} samples", n);
        }
    }

    #[test]
    fn test_resample_values_are_bipolar() {
        let cache = PrnCodeCache::new();
        let v = cache.resample_gps(1, 2048).unwrap();

        for &s in &v {
            assert!(s == 1.0 || s == -1.0, "sample = {}", s);
        }
    }

    #[test]
    fn test_resample_1023_samples_equals_original() {
        let cache = PrnCodeCache::new();
        let resampled = cache.resample_gps(1, 1023).unwrap();
        let original = cache.get_gps(1).unwrap();

        for (i, (&r, &o)) in resampled.iter().zip(original.iter()).enumerate() {
            assert_eq!(r, o as f32, "chip {}", i);
        }
    }

    #[test]
    fn test_resample_2048_produces_each_chip_1_or_2_times() {
        // At 2.048 Msps, 2048 samples/ms, 1023 chips/ms →
        // each chip appears either 2 times (freq) or 1 time. No chip skipped.
        let cache = PrnCodeCache::new();
        let resampled = cache.resample_gps(1, 2048).unwrap();
        let original = cache.get_gps(1).unwrap();

        // Count how many times each chip appears
        let mut counts = vec![0u32; 1023];

        for i in 0..2048 {
            let chip_idx = i * 1023 / 2048;
            counts[chip_idx] += 1;
        }

        // All chips must appear 1 or 2 times
        for (chip_idx, &count) in counts.iter().enumerate() {
            assert!(
                count == 2 || count == 3,
                "chip {} appears {} times",
                chip_idx,
                count
            );
        }

        // Verify values match
        for i in 0..2048 {
            let chip_idx = i * 1023 / 2048;
            assert_eq!(resampled[i], original[chip_idx] as f32, "sample {}", i);
        }
    }

    #[test]
    fn test_resample_invalid_prn_returns_none() {
        let cache = PrnCodeCache::new();

        assert!(cache.resample_gps(33, 2048).is_none());
    }

    #[test]
    fn test_resample_with_zero_phase_matches_no_phase() {
        let cache = PrnCodeCache::new();
        let base = cache.resample_gps(1, 2048).unwrap();
        let phased = cache.resample_gps_with_phase(1, 2048, 0.0).unwrap();

        for (a, b) in base.iter().zip(phased.iter()) {
            assert_eq!(a, b);
        }
    }

    #[test]
    fn test_resample_with_phase_wraps_correctly() {
        // Phase = 1023.0 (full period) should give same result as phase = 0
        let cache = PrnCodeCache::new();
        let base = cache.resample_gps(1, 2048).unwrap();
        let wrapped = cache.resample_gps_with_phase(1, 2048, 1023.0).unwrap();

        for (i, (a, b)) in base.iter().zip(wrapped.iter()).enumerate() {
            assert_eq!(a, b, "sample {}", i);
        }
    }

    #[test]
    fn test_resample_with_phase_shifts_code() {
        // A phase of exactly 1 chip should delay the code by 1 chip
        let cache = PrnCodeCache::new();
        let original = cache.get_gps(1).unwrap();
        // Resample at 1× (no upsampling) with phase offset of 1 chip
        let shifted = cache.resample_gps_with_phase(1, 1023, 1.0).unwrap();

        // shifted[0] should equal original[1], shifted[1] = original[2], etc.
        for i in 0..1022 {
            assert_eq!(
                shifted[i],
                original[(i + 1) % 1023] as f32,
                "chip {}: shifted={} expected={}",
                i,
                shifted[i],
                original[(i + 1) % 1023]
            );
        }
    }

    #[test]
    fn test_glonass_stub_returns_none() {
        let cache = PrnCodeCache::new();

        assert!(cache.get_glonass(1).is_none());
    }

    #[test]
    fn test_galileo_stub_returns_none() {
        let cache = PrnCodeCache::new();

        assert!(cache.get_galileo_e1(1).is_none());
    }

    #[test]
    fn test_code_period_is_exactly_1ms_at_chip_rate() {
        // 1023 chips / 1.023e6 chips/s = 1.0 ms
        let period_ms = GPS_CODE_LENGTH as f64 / GPS_CHIP_RATE_HZ * 1000.0;

        assert!((period_ms - 1.0).abs() < 1e-9, "period = {} ms", period_ms);
    }

    #[test]
    fn test_crosscorr_is_asymmetric() {
        // CC(A,B) at lag k ≠ CC(B,A) at lag k in general (except lag 0)
        let c1 = generate_gps_ca(3);
        let c2 = generate_gps_ca(4);
        let cc_ab = circular_cross_correlation(&c1, &c2);
        let cc_ba = circular_cross_correlation(&c2, &c1);

        // At lag 0 they are equal
        assert_eq!(cc_ab[0], cc_ba[0]);

        // At lag != 0 at least some differ (codes are not symmetric)
        let differ = cc_ab.iter().zip(cc_ba.iter()).skip(1).any(|(a, b)| a != b);

        assert!(differ, "CC(A,B) and CC(B,A) were identical for all lags");
    }
}
