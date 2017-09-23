use geometry::query::Proximity;
use geometry::shape::Shape;
use math::Point;

/// Trait implemented by algorithms that determine if two objects are in close proximity.
pub trait ProximityDetector<P: Point, M> : Sync + Send {
    /// Runs the proximity detection on two objects. It is assumed that the same proximity detector
    /// (the same structure) is always used with the same pair of object.
    fn update(&mut self,
              dispatcher: &ProximityDispatcher<P, M>,
              ma:         &M,
              a:          &Shape<P, M>,
              mb:         &M,
              b:          &Shape<P, M>,
              margin:     P::Real)
              -> bool;

    /// The number of collision detected during the last update.
    fn proximity(&self) -> Proximity;
}

pub type ProximityAlgorithm<P, M> = Box<ProximityDetector<P, M> + 'static>;

pub trait ProximityDispatcher<P: Point, M> : Sync + Send {
    /// Allocate a collision algorithm corresponding to the given pair of shapes.
    fn get_proximity_algorithm(&self, a: &Shape<P, M>, b: &Shape<P, M>) -> Option<ProximityAlgorithm<P, M>>;
}
