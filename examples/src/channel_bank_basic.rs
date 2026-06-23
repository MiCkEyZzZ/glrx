use glrx::acquisition::verifier::AcquisitionResult;
use glrx::signal::correlator::discriminators::EplOutput;
use glrx::tracking::channel::{ChannelBank, ChannelBankConfig};
use num_complex::Complex32;

fn acq(prn: u8) -> AcquisitionResult {
    AcquisitionResult {
        prn,
        doppler_hz: 200.0 * prn as f64,
        code_phase_samples: 0,
        code_phase_chips: 0.0,
        cn0_db_hz: 42.0,
        peak_to_noise: 80.0,
    }
}

fn main() {
    let mut bank = ChannelBank::new(ChannelBankConfig {
        num_channels: 4,
        ..Default::default()
    });

    for prn in 1..=4 {
        bank.allocate(&acq(prn));
    }

    println!("=== BANK BASIC ===");

    for epoch in 0..60 {
        bank.update_all(|prn| EplOutput {
            early: Complex32::new(1.0, 0.0),
            prompt: Complex32::new(prn as f32, 0.01 * epoch as f32),
            late: Complex32::new(1.0, 0.0),
        });

        let m = bank.metrics();

        println!(
            "epoch={:03} active={} phase_locked={}",
            epoch, m.active_channels, m.phase_locked_channels
        );
    }
}
