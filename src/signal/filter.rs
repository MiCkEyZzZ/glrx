use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Window {
    Rectangular,
    Hamming,
    Hann,
    Blackman,
}

impl Window {
    pub fn value(
        self,
        n: usize,
        len: usize,
    ) -> f64 {
        if len == 1 {
            return 1.0;
        }

        let m = (len - 1) as f64;
        let x = 2.0 * PI * n as f64 / m;

        match self {
            Window::Rectangular => 1.0,
            Window::Hamming => 0.54 - 0.46 * x.cos(),
            Window::Hann => 0.5 * (1.0 - x.cos()),
            Window::Blackman => 0.42 - 0.5 * x.cos() + 0.08 * (2.0 * x).cos(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // const FS: f64 = 2_048_000.0;

    #[test]
    fn test_rectangular_window_is_all_ones() {
        for n in 0..32 {
            assert_eq!(Window::Rectangular.value(n, 32), 1.0);
        }
    }

    #[test]
    fn test_haming_window_symmetry() {
        let len = 64;

        for n in 0..len / 2 {
            let w1 = Window::Hamming.value(n, len);
            let w2 = Window::Hamming.value(len - 1 - n, len);

            assert!((w1 - w2).abs() < 1e-10, "n={}: {} vs {}", n, w1, w2);
        }
    }

    #[test]
    fn test_hamming_window_endpoints() {
        let len = 64;

        // Hamming: w[0] = 0.54 - 0.46 = 0.08
        assert!((Window::Hamming.value(0, len) - 0.08).abs() < 1e-10);
        assert!((Window::Hamming.value(len - 1, len) - 0.08).abs() < 1e-10);
    }

    #[test]
    fn test_blackman_window_center_near_one() {
        let len = 65;
        let center = Window::Blackman.value(len / 2, len);

        assert!((center - 1.0).abs() < 1e-9, "center={}", center);
    }
}
