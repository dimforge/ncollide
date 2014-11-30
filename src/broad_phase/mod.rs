//! Broad phases.

#[doc(inline)]
pub use self::broad_phase::{BroadPhase, ProximityFilter};
pub use self::dbvt_broad_phase::DBVTBroadPhase;
#[doc(hidden)]
pub use self::proximity_signal::{ProximitySignal, ProximitySignalHandler};

#[doc(hidden)]
pub mod broad_phase;
#[doc(hidden)]
pub mod proximity_signal;
mod dbvt_broad_phase;
