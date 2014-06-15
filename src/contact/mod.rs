//! Main data structure for contacts.

use std::mem;
use math::{Scalar, Vect};

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
