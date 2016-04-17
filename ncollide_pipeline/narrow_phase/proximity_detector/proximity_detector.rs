use queries::geometry::Proximity;
use entities::inspection::{Repr, ReprDesc};
use math::{Point, Vector};

/// Trait implemented by algorithms that determine if two objects are in close proximity.
pub trait ProximityDetector<P: Point, M> {
    /// Runs the proximity detection on two objects. It is assumed that the same proximity detector
    /// (the same structure) is always used with the same pair of object.
    // FIXME: use ReprDesc instead of Repr (to avoid useless virtual method calls) ?
    fn update(&mut self,
              dispatcher: &ProximityDispatcher<P, M>,
              ma:         &M,
              a:          &Repr<P, M>,
              mb:         &M,
              b:          &Repr<P, M>,
              margin:     <P::Vect as Vector>::Scalar)
              -> bool;

    /// The number of collision detected during the last update.
    fn proximity(&self) -> Proximity;
}

pub type ProximityAlgorithm<P, M> = Box<ProximityDetector<P, M> + 'static>;

pub trait ProximityDispatcher<P: Point, M> {
    /// Allocate a collision algorithm corresponding to the given pair of shapes.
    fn get_proximity_algorithm(&self, a: &ReprDesc<P, M>, b: &ReprDesc<P, M>) -> Option<ProximityAlgorithm<P, M>>;
}
