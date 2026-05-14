//!

#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![deny(clippy::all)]
#![deny(clippy::pedantic)]
#![allow(clippy::similar_names)]
#![warn(clippy::must_use_candidate)]
#![warn(clippy::missing_const_for_fn)]
#![warn(clippy::semicolon_if_nothing_returned)]

////////////////////////////////////////////////////////////////////////////////
// Публичные модули
////////////////////////////////////////////////////////////////////////////////

pub mod rf;
pub mod signal;

pub use rf::*;
pub use signal::*;
