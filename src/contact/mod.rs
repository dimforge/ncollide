//! Main data structure for contacts.

use std::util;
use math::{N, V};

/// Geometric description of a contact.
#[deriving(ToStr, Eq, Clone, DeepClone, Encodable, Decodable)]
pub struct Contact {
    /// Position of the contact on the first object. The position is expressed in world space.
    world1: V,

    /// Position of the contact on the second object. The position is expressed in world space.
    world2: V,

    /// Contact normal
    normal: V,

    /// Penetration depth
    depth:  N
}

impl Contact {
    /// Creates a new contact.
    #[inline]
    pub fn new(world1: V, world2: V, normal: V, depth: N) -> Contact {
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
        util::swap(&mut self.world1, &mut self.world2);
        self.normal = -self.normal;
    }
}
