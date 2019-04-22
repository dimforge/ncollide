use crate::math::{Point, Vector};
use na::{self, RealField, Unit};
use crate::query::ContactKinematic;
use std::mem;
use crate::utils::GenerationalId;

/// Geometric description of a contact.
#[derive(Debug, PartialEq, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Contact<N: RealField> {
    /// Position of the contact on the first object. The position is expressed in world space.
    pub world1: Point<N>,

    /// Position of the contact on the second object. The position is expressed in world space.
    pub world2: Point<N>,

    /// Contact normal
    pub normal: Unit<Vector<N>>,

    /// Penetration depth
    pub depth: N,
}

impl<N: RealField> Contact<N> {
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
        let depth = -normal.dot(&(world2 - world1));
        Self::new(world1, world2, normal, depth)
    }
}

impl<N: RealField> Contact<N> {
    /// Reverts the contact normal and swaps `world1` and `world2`.
    #[inline]
    pub fn flip(&mut self) {
        mem::swap(&mut self.world1, &mut self.world2);
        self.normal = -self.normal;
    }
}

/// A contact combined with contact kinematic information as well as a persistant identifier.
///
/// When ncollide is used to compute contact points between moving solids, it will attempt to
/// match contact points found at successive frames. Two contact points are said to "match" if
/// they can be seen as the same contact point that moved in-between frames. Two matching
/// contact points are given the same `id` here.
#[derive(Clone, Debug)]
pub struct TrackedContact<N: RealField> {
    /// The geometric contact information.
    pub contact: Contact<N>,
    /// The local contact kinematic.
    pub kinematic: ContactKinematic<N>,
    /// The identifier of this contact.
    pub id: GenerationalId,
}

impl<N: RealField> TrackedContact<N> {
    /// Creates a new tracked contact.
    pub fn new(contact: Contact<N>, kinematic: ContactKinematic<N>, id: GenerationalId) -> Self {
        TrackedContact {
            contact,
            kinematic,
            id,
        }
    }
}

/// The prediction parameters for contact determination.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactPrediction<N: RealField> {
    linear: N,
    angular1: N,
    angular2: N,
    cos_angular1: N,
    cos_angular2: N,
    sin_angular1: N,
    sin_angular2: N,
}

impl<N: RealField> ContactPrediction<N> {
    /// Initialize prediction parameters.
    pub fn new(linear: N, angular1: N, angular2: N) -> Self {
        ContactPrediction {
            linear,
            angular1,
            angular2,
            cos_angular1: angular1.cos(),
            cos_angular2: angular2.cos(),
            sin_angular1: angular1.sin(),
            sin_angular2: angular2.sin(),
        }
    }

    /// The linear prediction.
    #[inline]
    pub fn linear(&self) -> N {
        self.linear
    }


    /// Sets linear prediction.
    #[inline]
    pub fn set_linear(&mut self, val: N) {
        self.linear = val
    }

    /// The angular regularization for the first solid.
    #[inline]
    pub fn angular1(&self) -> N {
        self.angular1
    }

    /// The angular regularization for the second solid.
    #[inline]
    pub fn angular2(&self) -> N {
        self.angular2
    }

    /// The cosine of angular regularization for the first solid.
    #[inline]
    pub fn cos_angular1(&self) -> N {
        self.cos_angular1
    }

    /// The cosine angular regularization for the second solid.
    #[inline]
    pub fn cos_angular2(&self) -> N {
        self.cos_angular2
    }

    /// The sine of angular regularization for the first solid.
    #[inline]
    pub fn sin_angular1(&self) -> N {
        self.sin_angular1
    }

    /// The sine angular regularization for the second solid.
    #[inline]
    pub fn sin_angular2(&self) -> N {
        self.sin_angular2
    }
}
