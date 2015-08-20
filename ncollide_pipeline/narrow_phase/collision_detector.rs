use entities::inspection::{Repr, ReprDesc};
use queries::geometry::Contact;
use math::Point;

/// Trait of the algorithms executed during the so-called Narrow Phase.
///
/// The goal of the narrow phase is to determine exactly if two objects collide. If there is
/// collision, it must be able to compute the exact contact point(s), normal and penetration depth
/// in order to give enough informations to the constraint solver.
///
/// # Arguments
/// * `G1`- the type of the first object involved on the collision detection.
/// * `G2`- the type of the second object involved on the collision detection.
pub trait CollisionDetector<P: Point, M> {
    /// Runs the collision detection on two objects. It is assumed that the same
    /// collision detector (the same structure) is always used with the same
    /// pair of object.
    // FIXME: use ReprDesc instead of Repr (to avoid useless virtual method calls) ?
    fn update(&mut self, &CollisionDispatcher<P, M>, &M, &Repr<P, M>, &M, &Repr<P, M>) -> bool;

    /// The number of collision detected during the last update.
    fn num_colls(&self) -> usize;

    /// Collects the collisions detected during the last update.
    fn colls(&self, &mut Vec<Contact<P>>);
}

pub type CollisionAlgorithm<P, M> = Box<CollisionDetector<P, M> + 'static>;

pub trait CollisionDispatcher<P, M> {
    /// Allocate a collision algorithm corresponding to the given pair of shapes.
    fn get_collision_algorithm(&self, a: &ReprDesc<P, M>, b: &ReprDesc<P, M>) -> Option<CollisionAlgorithm<P, M>>;
}
