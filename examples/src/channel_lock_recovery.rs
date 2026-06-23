use glrx::acquisition::verifier::AcquisitionResult;
use glrx::signal::correlator::discriminators::EplOutput;
use glrx::tracking::channel::{ChannelBank, ChannelBankConfig, ChannelState};
use num_complex::Complex32;

fn main() {
    let mut bank = ChannelBank::new(ChannelBankConfig {
        num_channels: 2,
        ..Default::default()
    });

    bank.allocate(&AcquisitionResult {
        prn: 11,
        doppler_hz: 100.0,
        code_phase_samples: 0,
        code_phase_chips: 0.0,
        cn0_db_hz: 40.0,
        peak_to_noise: 80.0,
    });

    println!("=== LOCK LOSS SIMULATION ===");

    for epoch in 0..100 {
        bank.update_all(|_| EplOutput {
            early: Complex32::new(0.0, 0.0),
            prompt: Complex32::new(0.0, 0.0),
            late: Complex32::new(0.0, 0.0),
        });

        let metrics = bank.metrics();

        println!(
            "epoch={:03} active={} phase={} lost={}",
            epoch,
            metrics.active_channels,
            metrics.phase_locked_channels,
            bank.channels()
                .filter(|c| c.state == ChannelState::LockLost)
                .count()
        );
    }

    let reaped = bank.reap_lost();
    println!("reaped PRNs: {:?}", reaped);
}
