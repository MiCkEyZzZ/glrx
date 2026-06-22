use glrx::tracking::pll::Pll;
use num_complex::Complex32;

fn main() {
    let mut pll = Pll::with_defaults(2000.0);

    println!("=== ТЕСТ ПЕРЕХОДНОЙ ХАРАКТЕРИСТИКИ ===");

    for epoch in 0..100 {
        // резкий step на фазе → PLL должен восстановиться
        let prompt = if epoch < 30 {
            Complex32::new(1.0, 0.0)
        } else {
            Complex32::new(0.3, 0.95) // резкий фазовый сдвиг
        };

        let out = pll.update(prompt);

        println!(
            "эпоха={:03} частота={:10.3} фаза={:.4}",
            epoch, out.carrier_freq_hz, out.carrier_phase_rad
        );
    }
}
