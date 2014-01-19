use contact::Contact;
use math::{N, V, M};

/// Trait of the algorithms executed during the so-called Narrow Phase.
///
/// The goal of the narrow phase is to determine exactly if two objects collide. If there is
/// collision, it must be able to compute the exact contact point(s), normal and penetration depth
/// in order to give enough informations to the constraint solver.
///
/// # Arguments
/// * `G1`- the type of the first object involved on the collision detection.
/// * `G2`- the type of the second object involved on the collision detection.
pub trait CollisionDetector<G1, G2> {
    /// Runs the collision detection on two objects. It is assumed that the same
    /// collision detector (the same structure) is always used with the same
    /// pair of object.
    fn update(&mut self, &M, &G1, &M, &G2);

    /// The number of collision detected during the last update.
    fn num_colls(&self) -> uint;

    /// Collects the collisions detected during the last update.
    fn colls(&self, &mut ~[Contact]);

    /// Computes the time of impact of two objects.
    ///
    /// # Arguments
    /// * `m1`   - the first object transform.
    /// * `dir`  - the first object displacement direction.
    /// * `dist` - the first object displacement distance.
    /// * `g1`   - the first object.
    /// * `m2`   - the second object transform.
    /// * `g2`   - the second object.
    fn toi(unused_self: Option<Self>, m1: &M, dir: &V, dist: &N, g1: &G1, m2: &M, g2: &G2) -> Option<N>;
}
