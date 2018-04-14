use std::any::Any;

use geometry::query::Proximity;
use geometry::shape::Shape;
use math::Point;

/// Trait implemented by algorithms that determine if two objects are in close proximity.
pub trait ProximityDetector<N: Real, M>: Any + Send + Sync {
    /// Runs the proximity detection on two objects. It is assumed that the same proximity detector
    /// (the same structure) is always used with the same pair of object.
    fn update(
        &mut self,
        dispatcher: &ProximityDispatcher<P, M>,
        ma: &Isometry<N>,
        a: &Shape<N>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        margin: N,
    ) -> bool;

    /// The number of collision detected during the last update.
    fn proximity(&self) -> Proximity;
}

pub type ProximityAlgorithm<P, M> = Box<ProximityDetector<P, M>>;

pub trait ProximityDispatcher<N: Real, M>: Any + Send + Sync {
    /// Allocate a collision algorithm corresponding to the given pair of shapes.
    fn get_proximity_algorithm(
        &self,
        a: &Shape<N>,
        b: &Shape<N>,
    ) -> Option<ProximityAlgorithm<P, M>>;
}
