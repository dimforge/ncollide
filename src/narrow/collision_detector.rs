use std::mem;
use math::{Scalar, Vect, Matrix};

/// Geometric description of a contact.
#[deriving(Show, PartialEq, Clone, Encodable, Decodable)]
pub struct Contact {
    /// Position of the contact on the first object. The position is expressed in world space.
    pub world1: Vect,

    /// Position of the contact on the second object. The position is expressed in world space.
    pub world2: Vect,

    /// Contact normal
    pub normal: Vect,

    /// Penetration depth
    pub depth:  Scalar
}

impl Contact {
    /// Creates a new contact.
    #[inline]
    pub fn new(world1: Vect, world2: Vect, normal: Vect, depth: Scalar) -> Contact {
        Contact {
            world1: world1,
            world2: world2,
            normal: normal,
            depth:  depth
        }
    }
}

impl Contact {
    /// Reverts the contact normal and swaps `world1` and `world2`.
    #[inline]
    pub fn flip(&mut self) {
        mem::swap(&mut self.world1, &mut self.world2);
        self.normal = -self.normal;
    }
}

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
    fn update(&mut self, &Matrix, &G1, &Matrix, &G2);

    /// The number of collision detected during the last update.
    fn num_colls(&self) -> uint;

    /// Collects the collisions detected during the last update.
    fn colls(&self, &mut Vec<Contact>);

    /// Computes the time of impact of two objects.
    ///
    /// # Arguments
    /// * `m1`   - the first object transform.
    /// * `dir`  - the first object displacement direction.
    /// * `dist` - the first object displacement distance.
    /// * `g1`   - the first object.
    /// * `m2`   - the second object transform.
    /// * `g2`   - the second object.
    fn toi(unused_self: Option<Self>, m1: &Matrix, dir: &Vect, dist: &Scalar, g1: &G1, m2: &Matrix, g2: &G2) -> Option<Scalar>;
}
