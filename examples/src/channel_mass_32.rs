use glrx::acquisition::verifier::AcquisitionResult;
use glrx::signal::correlator::discriminators::EplOutput;
use glrx::tracking::channel::{ChannelBank, ChannelBankConfig};
use num_complex::Complex32;

fn main() {
    let mut bank = ChannelBank::new(ChannelBankConfig {
        num_channels: 32,
        ..Default::default()
    });

    for prn in 1..=32 {
        bank.allocate(&AcquisitionResult {
            prn,
            doppler_hz: prn as f64 * 50.0,
            code_phase_samples: 0,
            code_phase_chips: 0.0,
            cn0_db_hz: 45.0,
            peak_to_noise: 100.0,
        });
    }

    println!("=== MASS TRACKING 32 CH ===");

    for epoch in 0..200 {
        bank.update_all(|prn| EplOutput {
            early: Complex32::new(1.0, 0.0),
            prompt: Complex32::new((prn as f32).sin(), 0.01 * epoch as f32),
            late: Complex32::new(1.0, 0.0),
        });

        if epoch % 10 == 0 {
            let m = bank.metrics();
            println!(
                "epoch={} active={} phase_locked={}",
                epoch, m.active_channels, m.phase_locked_channels
            );
        }
    }
}
