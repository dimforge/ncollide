use std::mem;

/// Shapeetric description of a contact.
#[deriving(Show, PartialEq, Clone, Encodable, Decodable)]
pub struct Contact<N, P, V> {
    /// Position of the contact on the first object. The position is expressed in world space.
    pub world1: P,

    /// Position of the contact on the second object. The position is expressed in world space.
    pub world2: P,

    /// Contact normal
    pub normal: V,

    /// Penetration depth
    pub depth:  N
}

impl<N, P, V> Contact<N, P, V> {
    /// Creates a new contact.
    #[inline]
    pub fn new(world1: P, world2: P, normal: V, depth: N) -> Contact<N, P, V> {
        Contact {
            world1: world1,
            world2: world2,
            normal: normal,
            depth:  depth
        }
    }
}

impl<N, P, V: Neg<V>> Contact<N, P, V> {
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
pub trait CollisionDetector<N, P, V, M, G1, G2> {
    /// Runs the collision detection on two objects. It is assumed that the same
    /// collision detector (the same structure) is always used with the same
    /// pair of object.
    fn update(&mut self, &M, &G1, &M, &G2);

    /// The number of collision detected during the last update.
    fn num_colls(&self) -> uint;

    /// Collects the collisions detected during the last update.
    fn colls(&self, &mut Vec<Contact<N, P, V>>);
}
