use std::mem;
use utils::{GenerationalId, IdAllocator};
use shape::FeatureId;
use bounding_volume::PolyhedralCone;
use query::ContactKinematic;
use math::{Isometry, Point};

use na::{self, Real, Unit};

/// Geometric description of a contact.
#[derive(Debug, PartialEq, Clone)]
pub struct Contact<P: Point> {
    /// Position of the contact on the first object. The position is expressed in world space.
    pub world1: P,

    /// Position of the contact on the second object. The position is expressed in world space.
    pub world2: P,

    /// Contact normal
    pub normal: Unit<P::Vector>,

    /// Penetration depth
    pub depth: P::Real,
}

impl<P: Point> Contact<P> {
    /// Creates a new contact.
    #[inline]
    pub fn new(world1: P, world2: P, normal: Unit<P::Vector>, depth: P::Real) -> Self {
        Contact {
            world1,
            world2,
            normal,
            depth,
        }
    }

    /// Creates a new contact, computing automatically the penetration depth.
    #[inline]
    pub fn new_wo_depth(world1: P, world2: P, normal: Unit<P::Vector>) -> Contact<P> {
        let depth = -na::dot(normal.as_ref(), &(world2 - world1));
        Self::new(world1, world2, normal, depth)
    }
}

impl<P: Point> Contact<P> {
    /// Reverts the contact normal and swaps `world1` and `world2`.
    #[inline]
    pub fn flip(&mut self) {
        mem::swap(&mut self.world1, &mut self.world2);
        self.normal = -self.normal;
    }
}

#[derive(Clone, Debug)]
pub struct TrackedContact<P: Point> {
    pub contact: Contact<P>,
    pub kinematic: ContactKinematic<P>,
    pub id: GenerationalId,
}

impl<P: Point> TrackedContact<P> {
    pub fn new(contact: Contact<P>, kinematic: ContactKinematic<P>, id: GenerationalId) -> Self {
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
