use crate::pipeline::object::CollisionObjectSet;
use na::RealField;
use std::any::Any;

/// A signal handler for contact detection.
pub trait BroadPhasePairFilter<N: RealField, Set: CollisionObjectSet<N>>: Any + Send + Sync
{
    /// Activate an action for when two objects start or stop to be close to each other.
    fn is_pair_valid(&self, h1: Set::CollisionObjectHandle, h2: Set::CollisionObjectHandle, s: &Set) -> bool;
}

impl<N: RealField, Set: CollisionObjectSet<N>> BroadPhasePairFilter<N,Set> for () {
    fn is_pair_valid(&self, _: Set::CollisionObjectHandle, _: Set::CollisionObjectHandle, _: &Set) -> bool {
        true
    }
}
