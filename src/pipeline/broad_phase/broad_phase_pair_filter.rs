use na::RealField;
use std::any::Any;

/// A signal handler for contact detection.
pub trait BroadPhasePairFilter<N: RealField, Object, Handle>: Any + Send + Sync {
    /// Activate an action for when two objects start or stop to be close to each other.
    fn is_pair_valid(&self, b1: Object, b2: Object, h1: Handle, h2: Handle) -> bool;
}