use glrx::tracking::pll::{LockDetectorConfig, Pll, PllConfig, PllState};
use num_complex::Complex32;

fn main() {
    env_logger::init();

    let mut pll = Pll::new(
        PllConfig {
            bandwidth_hz: 18.0,
            integration_ms: 1,
            lock_detector: LockDetectorConfig {
                window_size: 50,
                phase_std_threshold_rad: 0.6,
                cn0_threshold_db_hz: 30.0,
                min_samples: 10,
            },
            output_clamp_hz: 5000.0,
        },
        1000.0, // Doppler после acquisition/FLL
    );

    println!("=== PLL ASCENDING DRIFT TEST ===");

    let mut lock_reported = false;

    for epoch in 0..120 {
        // Медленно меняющаяся фазовая ошибка.
        let phase_noise = 0.05_f32;
        let phase_offset = 0.1 + phase_noise * (epoch as f32 * 0.02).sin();

        let prompt = Complex32::new(1.0, phase_offset);

        let out = pll.update(prompt);

        println!(
            "epoch={:03} state={:?} freq={:10.3} Hz phase_err={:+.6} rad",
            epoch, out.state, out.carrier_freq_hz, out.discriminator_output
        );

        if !lock_reported && out.state == PllState::PllLock {
            println!("→ PLL LOCK ACQUIRED");
            lock_reported = true;
        }

        if out.state == PllState::LockLost {
            println!("→ PLL LOCK LOST");
            break;
        }
    }

    println!();
    println!("Final frequency: {:.3} Hz", pll.carrier_freq_hz());

    let metrics = pll.benchmark_metrics();

    println!("Time to lock: {:?} ms", metrics.time_to_lock_ms);
    println!(
        "Phase std deviation: {:.6} rad",
        metrics.steady_state_phase_error_rad
    );
    println!("C/N0 estimate: {:?} dB-Hz", metrics.cn0_db_hz);
}
