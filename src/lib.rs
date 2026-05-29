//! GLRX - GNSS software-defined receiver.
//!
//! # Pipeline overview
//!
//! ```text
//! IqSource → Signal → Acquisition → Tracking → Navigation → Solver → Output
//! ```

#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![deny(clippy::all)]
#![deny(clippy::pedantic)]
#![allow(clippy::similar_names)]
#![warn(clippy::must_use_candidate)]
#![warn(clippy::missing_const_for_fn)]
#![warn(clippy::semicolon_if_nothing_returned)]
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::module_name_repetitions
)]

////////////////////////////////////////////////////////////////////////////////
// Публичные модули
////////////////////////////////////////////////////////////////////////////////

pub mod rf;
pub mod signal;

pub use rf::*;
pub use signal::*;
