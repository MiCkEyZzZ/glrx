use num_complex::Complex32;

#[derive(Debug, Clone)]
pub struct EplOutput {
    pub early: Complex32,
    pub prompt: Complex32,
    pub late: Complex32,
}

impl EplOutput {
    pub fn dll_nelp(&self) -> f32 {
        let pe = self.early.norm_sqr();
        let pl = self.late.norm_sqr();
        let denom = pe + pl;

        if denom < f32::EPSILON {
            0.0
        } else {
            (pe - pl) / denom
        }
    }

    pub fn dll_ele(&self) -> f32 {
        self.early.norm() - self.late.norm()
    }

    pub fn pll_atan2(&self) -> f32 {
        self.prompt.im.atan2(self.prompt.re)
    }

    pub fn pll_dd_atan(&self) -> f32 {
        let i = self.prompt.re;
        let q = self.prompt.im;

        (q / i.abs().max(f32::EPSILON)).atan()
    }

    pub fn prompt_power(&self) -> f32 {
        self.prompt.norm_sqr()
    }

    pub fn prompt_i(&self) -> f32 {
        self.prompt.re
    }

    pub fn prompt_q(&self) -> f32 {
        self.prompt.im
    }
}
