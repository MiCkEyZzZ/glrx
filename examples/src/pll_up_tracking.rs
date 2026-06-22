use glrx::tracking::pll::{Pll, PllState};
use num_complex::Complex32;

fn main() {
    env_logger::init();

    let mut pll = Pll::new(
        glrx::tracking::pll::PllConfig {
            pll_bandwidth_hz: 18.0,
            fll_bandwidth_hz: 120.0,
            integration_ms: 1,
            fll_to_pll_stable_epochs: 5,
            fll_stable_threshold_hz: 20.0,
            ..Default::default()
        },
        1000.0, // initial Doppler
    );

    println!("=== ТЕСТ ВОСХОДЯЩЕГО ДРЕЙФА PLL ===");

    for epoch in 0..120 {
        // имитация сигнала: небольшая фазовая ошибка + дрейф вверх
        let phase_noise = 0.05;
        let prompt = Complex32::new(1.0, 0.1 + phase_noise * ((epoch as f32) * 0.02).sin());

        let out = pll.update(prompt);

        println!(
            "эпоха={:03} состояние={:?} частота={:10.3} ошибка={:.6}",
            epoch, out.state, out.carrier_freq_hz, out.discriminator_output
        );

        if out.state == PllState::PllLock && epoch > 10 {
            println!("→ ЗАХВАТ PLL ВЫПОЛНЕН");
        }
    }

    println!("\nконечная частота: {:.3} Гц", pll.carrier_freq_hz());
}
