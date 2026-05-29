//! Корреляционныцй подмодуль: EPL-коррелятор, дискриминатор, утилиты кода,
//! нормализации и оценка C/N₀.
//!
//! # Структура
//!
//! ```text
//! signal::correlator
//! ├── base            — correlator_epl(): суммирование E/P/L
//! ├── discriminators  — EplOutput + DLL/PLL дискриминаторы
//! ├── code_utilities  — shift_code(), make_epl_replicas()
//! └── normalisation   — compute_power, normalize, cn0_estimate
//! ```
//!
//! # Типичный порядок вызовов
//!
//! ```text
//! let (early, prompt, late) = make_epl_replicas(&prn_code, half_chip);
//! let signal_bb = mixer.mix(&iq_block);
//! let epl = correlator_epl(&signal_bb, &early, &prompt, &late);
//! let dll_err = epl.dll_nelp();
//! let pll_err = epl.pll_dd_atan();
//! let cn0_db = cn0_estimate(&prompt_history, T_coh);
//! ```

pub mod base;
pub mod code_utilities;
pub mod discriminators;
pub mod normalisation;
