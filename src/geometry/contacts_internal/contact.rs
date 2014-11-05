use std::mem;

/// Geometric description of a contact.
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
