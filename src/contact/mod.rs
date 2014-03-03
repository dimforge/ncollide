//! Main data structure for contacts.

use std::mem;
use math::{Scalar, Vector};

/// Geometric description of a contact.
#[deriving(Show, Eq, Clone, DeepClone, Encodable, Decodable)]
pub struct Contact {
    /// Position of the contact on the first object. The position is expressed in world space.
    world1: Vector,

    /// Position of the contact on the second object. The position is expressed in world space.
    world2: Vector,

    /// Contact normal
    normal: Vector,

    /// Penetration depth
    depth:  Scalar
}

impl Contact {
    /// Creates a new contact.
    #[inline]
    pub fn new(world1: Vector, world2: Vector, normal: Vector, depth: Scalar) -> Contact {
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
    pub fn flip(&mut self) {
        mem::swap(&mut self.world1, &mut self.world2);
        self.normal = -self.normal;
    }
}
