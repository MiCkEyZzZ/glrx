use glrx::tracking::pll::{Pll, PllState};
use num_complex::Complex32;

fn main() {
    env_logger::init();

    let mut pll = Pll::new(
        glrx::tracking::pll::PllConfig {
            pll_bandwidth_hz: 12.0,
            fll_bandwidth_hz: 80.0,
            integration_ms: 1,
            fll_to_pll_stable_epochs: 4,
            fll_stable_threshold_hz: 15.0,
            ..Default::default()
        },
        3000.0, // начальный доплер
    );

    println!("=== ТЕСТ НИСХОДЯЩЕГО ДРЕЙФА PLL + ПЕРЕКЛЮЧЕНИЯ НАВИГАЦИОННЫХ БИТОВ ===");

    for epoch in 0..150 {
        let drift = -0.015 * (epoch as f32); // вниз по частоте

        // имитация BPSK навигационного бита (±1)
        let nav_bit = if epoch % 20 < 10 { 1.0 } else { -1.0 };

        let prompt = Complex32::new(nav_bit, 0.05 + drift.sin() * 0.1);

        let out = pll.update(prompt);

        println!(
            "эпоха={:03} состояние={:?} частота={:10.3} дискр={:.6}",
            epoch, out.state, out.carrier_freq_hz, out.discriminator_output
        );

        if out.state == PllState::LockLost {
            println!("!!! ПОТЕРЯ ЗАХВАТА !!!");
            break;
        }
    }

    println!("\nконечное состояние = {:?}", pll.state());
}
