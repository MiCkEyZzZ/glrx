use glrx::acquisition::verifier::AcquisitionResult;
use glrx::signal::correlator::discriminators::EplOutput;
use glrx::tracking::channel::{ChannelConfig, ChannelState, TrackingChannel};
use num_complex::Complex32;

fn main() {
    let acq = AcquisitionResult {
        prn: 7,
        doppler_hz: 500.0,
        code_phase_samples: 0,
        code_phase_chips: 12.3,
        cn0_db_hz: 45.0,
        peak_to_noise: 100.0,
    };

    let mut ch = TrackingChannel::allocate(&acq, ChannelConfig::default());

    println!("=== SINGLE CHANNEL TRACKING ===");

    for epoch in 0..80 {
        let epl = EplOutput {
            early: Complex32::new(1.0, 0.0),
            prompt: Complex32::new(1.0, 0.05 * (epoch as f32 * 0.1).sin()),
            late: Complex32::new(1.0, 0.0),
        };

        let out = ch.update(&epl);

        println!(
            "epoch={:03} state={:?} cn0={:?}",
            epoch, out.state, out.cn0_db_hz
        );

        if out.state == ChannelState::PhaseLock {
            println!("→ PHASE LOCK ACHIEVED");
            break;
        }
    }
}
