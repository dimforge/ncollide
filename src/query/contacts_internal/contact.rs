use std::mem;
use utils::GenerationalId;
use na::{self, Real, Unit};
use query::ContactKinematic;
use math::{Point, Vector};

/// Geometric description of a contact.
#[derive(Debug, PartialEq, Clone)]
pub struct Contact<N: Real> {
    /// Position of the contact on the first object. The position is expressed in world space.
    pub world1: Point<N>,

    /// Position of the contact on the second object. The position is expressed in world space.
    pub world2: Point<N>,

    /// Contact normal
    pub normal: Unit<Vector<N>>,

    /// Penetration depth
    pub depth: N,
}

impl<N: Real> Contact<N> {
    /// Creates a new contact.
    #[inline]
    pub fn new(world1: Point<N>, world2: Point<N>, normal: Unit<Vector<N>>, depth: N) -> Self {
        Contact {
            world1,
            world2,
            normal,
            depth,
        }
    }

    /// Creates a new contact, computing automatically the penetration depth.
    #[inline]
    pub fn new_wo_depth(world1: Point<N>, world2: Point<N>, normal: Unit<Vector<N>>) -> Contact<N> {
        let depth = -na::dot(normal.as_ref(), &(world2 - world1));
        Self::new(world1, world2, normal, depth)
    }
}

impl<N: Real> Contact<N> {
    /// Reverts the contact normal and swaps `world1` and `world2`.
    #[inline]
    pub fn flip(&mut self) {
        mem::swap(&mut self.world1, &mut self.world2);
        self.normal = -self.normal;
    }
}

#[derive(Clone, Debug)]
pub struct TrackedContact<N: Real> {
    pub contact: Contact<N>,
    pub kinematic: ContactKinematic<N>,
    pub id: GenerationalId,
}

impl<N: Real> TrackedContact<N> {
    pub fn new(contact: Contact<N>, kinematic: ContactKinematic<N>, id: GenerationalId) -> Self {
        TrackedContact {
            contact,
            kinematic,
            id,
        }
    }
}

/// The prediction parameters for contact determination.n
pub struct ContactPrediction<N: Real> {
    /// The linear prediction.
    pub linear: N,
    /// The angular regularization for the first solid.
    pub angular1: N,
    /// The angular regularization for the second solid.
    pub angular2: N,
}
impl<N: Real> ContactPrediction<N> {
    /// Initialize prediction parameters.
    pub fn new(linear: N, angular1: N, angular2: N) -> Self {
        ContactPrediction {
            linear,
            angular1,
            angular2,
        }
    }
}
