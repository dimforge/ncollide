use std::mem;
use math::{Point, Vect};

/// Geometric description of a contact.
#[derive(Debug, PartialEq, Clone, RustcEncodable, RustcDecodable)]
pub struct Contact<P: Point> {
    /// Position of the contact on the first object. The position is expressed in world space.
    pub world1: P,

    /// Position of the contact on the second object. The position is expressed in world space.
    pub world2: P,

    /// Contact normal
    pub normal: P::Vect,

    /// Penetration depth
    pub depth:  <P::Vect as Vect>::Scalar
}

impl<P: Point> Contact<P> {
    /// Creates a new contact.
    #[inline]
    pub fn new(world1: P, world2: P, normal: P::Vect, depth: <P::Vect as Vect>::Scalar) -> Contact<P> {
        Contact {
            world1: world1,
            world2: world2,
            normal: normal,
            depth:  depth
        }
    }
}

impl<P: Point> Contact<P> {
    /// Reverts the contact normal and swaps `world1` and `world2`.
    #[inline]
    pub fn flip(&mut self) {
        mem::swap(&mut self.world1, &mut self.world2);
        self.normal = -self.normal.clone();
    }
}
