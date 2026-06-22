use glrx::signal::correlator::discriminators::EplOutput;
use glrx::tracking::dll::{Dll, DllConfig};
use num_complex::Complex32;

fn main() {
    let mut dll = Dll::new(DllConfig::default());

    for epoch in 0..100 {
        let epl = EplOutput {
            early: Complex32::new(0.6, 0.0),
            prompt: Complex32::new(1.0, 0.0),
            late: Complex32::new(0.8, 0.0),
        };

        let out = dll.update(&epl);

        println!("эпоха={} частота_кода={:.3}", epoch, out.chip_freq_hz);
    }
}
