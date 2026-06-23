use glrx::tracking::pll::{LockDetectorConfig, Pll, PllConfig, PllState};
use num_complex::Complex32;

fn main() {
    env_logger::init();

    let mut pll = Pll::new(
        PllConfig {
            bandwidth_hz: 12.0,
            integration_ms: 1,
            lock_detector: LockDetectorConfig {
                window_size: 50,
                phase_std_threshold_rad: 0.6,
                cn0_threshold_db_hz: 30.0,
                min_samples: 10,
            },
            output_clamp_hz: 5000.0,
        },
        3000.0, // Doppler после handoff от FLL
    );

    println!("=== PLL: carrier tracking with navigation bit transitions ===");

    for epoch in 0..150 {
        // Медленный дрейф частоты
        let drift = -0.015 * epoch as f32;

        // GPS навигационный бит меняется каждые 20 мс
        let nav_bit = if epoch % 20 < 10 { 1.0 } else { -1.0 };

        // Небольшая фазовая ошибка
        let prompt = Complex32::new(nav_bit, 0.05 + drift.sin() * 0.1);

        let out = pll.update(prompt);

        println!(
            "epoch={:03} state={:?} freq={:10.3} Hz phase_err={:+.6} rad",
            epoch, out.state, out.carrier_freq_hz, out.discriminator_output
        );

        if out.state == PllState::LockLost {
            println!("!!! LOCK LOST !!!");
            break;
        }
    }

    println!();
    println!("Final state: {:?}", pll.state());

    let metrics = pll.benchmark_metrics();

    println!("Time to lock: {:?} ms", metrics.time_to_lock_ms);

    println!("Phase std: {:.6} rad", metrics.steady_state_phase_error_rad);

    println!("C/N0: {:?} dB-Hz", metrics.cn0_db_hz);
}
