use std::num::Bounded;
use std::rc::Rc;
use std::cell::RefCell;
use na::Translation;
use na;
use bounding_volume::{HasBoundingVolume, BoundingVolume, LooseBoundingVolume};
use math::{Scalar, Vect, Matrix};

/// Trait of objects that can be bounded by an AABB.
pub trait HasAABB {
    /// The object’s AABB.
    fn aabb(&self, &Matrix) -> AABB;
}

/// An Axis Aligned Bounding Box.
///
/// # Parameter:
///   * `Vect` - type of the points of the bounding box. It determines the AABB dimension.
///   * `Scalar` - type of the one components of the aabb points.
#[deriving(Show, PartialEq, Clone, Encodable, Decodable)]
pub struct AABB {
    mins: Vect,
    maxs: Vect
}

impl AABB {
    /// Creates a new AABB.
    ///
    /// # Arguments:
    ///   * `mins` - position of the point with the smallest coordinates.
    ///   * `maxs` - position of the point with the highest coordinates. Each component of `mins`
    ///   must be smaller than the related components of `maxs`.
    pub fn new(mins: Vect, maxs: Vect) -> AABB {
        assert!(na::partial_le(&mins, &maxs));

        AABB {
            mins: mins,
            maxs: maxs
        }
    }

    /// Creates an invalid AABB with:
    /// * `mins = Bounded::max_value()`
    /// * `maxs = Bounded::max_value()`.
    /// This is useful to build aabb using merges.
    pub fn new_invalid() -> AABB {
        let _max: Vect = Bounded::max_value();
        AABB {
            mins: Bounded::max_value(),
            maxs: -_max,
        }
    }

    /// Reference to the AABB point with the smallest components along each axis.
    #[inline]
    pub fn mins<'a>(&'a self) -> &'a Vect {
        &self.mins
    }

    /// Reference to the AABB point with the biggest components along each axis.
    #[inline]
    pub fn maxs<'a>(&'a self) -> &'a Vect {
        &self.maxs
    }

}

impl BoundingVolume for AABB {
    #[inline]
    fn intersects(&self, other: &AABB) -> bool {
        na::partial_le(&self.mins, &other.maxs) &&
        na::partial_ge(&self.maxs, &other.mins)
    }

    #[inline]
    fn contains(&self, other: &AABB) -> bool {
        na::partial_le(&self.mins, &other.mins) &&
        na::partial_ge(&self.maxs, &other.maxs)
    }

    #[inline]
    fn merge(&mut self, other: &AABB) {
        self.mins = na::inf(&self.mins, &other.mins);
        self.maxs = na::sup(&self.maxs, &other.maxs);
    }

    #[inline]
    fn merged(&self, other: &AABB) -> AABB {
        AABB {
            mins: na::inf(&self.mins, &other.mins),
            maxs: na::sup(&self.maxs, &other.maxs)
        }
    }
}

impl LooseBoundingVolume for AABB {
    #[inline]
    fn loosen(&mut self, amount: Scalar) {
        self.mins = self.mins - amount;
        self.maxs = self.maxs + amount;
    }

    #[inline]
    fn loosened(&self, amount: Scalar) -> AABB {
        AABB {
            mins: self.mins - amount,
            maxs: self.maxs + amount
        }
    }
}

impl Translation<Vect> for AABB
{
    #[inline]
    fn translation(&self) -> Vect {
        (self.mins + self.maxs) * na::cast::<f64, Scalar>(0.5)
    }

    #[inline]
    fn inv_translation(&self) -> Vect {
        -self.translation()
    }

    #[inline]
    fn append_translation(&mut self, dv: &Vect) {
        self.mins = self.mins + *dv;
        self.maxs = self.maxs + *dv;
    }

    #[inline]
    fn append_translation_cpy(aabb: &AABB, dv: &Vect) -> AABB {
        AABB::new(aabb.mins + *dv, aabb.maxs + *dv)
    }

    #[inline]
    fn prepend_translation(&mut self, dv: &Vect) {
        self.append_translation(dv)
    }

    #[inline]
    fn prepend_translation_cpy(aabb: &AABB, dv: &Vect) -> AABB {
        Translation::append_translation_cpy(aabb, dv)
    }

    #[inline]
    fn set_translation(&mut self, v: Vect) {
        let center = self.translation();

        self.mins = self.mins - center + v;
        self.maxs = self.maxs - center + v;
    }
}

// FIXME: remove that to use `Transformed` istead?
/// Wrapper which implements `HasBoundingVolume<AABB>` for objects implementing `HasAABB`.
#[doc(hidden)]
#[deriving(Clone, PartialEq)]
pub struct WithAABB<A>(pub Matrix, pub A);

impl<A> WithAABB<A> {
    /// The transformation matrix of this shape.
    #[inline]
    pub fn m<'a>(&'a self) -> &'a Matrix {
        let WithAABB(ref t, _) = *self;

        t
    }

    /// The local-space geometry of this shape.
    #[inline]
    pub fn g<'a>(&'a self) -> &'a A {
        let WithAABB(_, ref g) = *self;

        g
    }
}

impl<A: HasAABB> HasBoundingVolume<AABB> for WithAABB<A> {
    #[inline]
    fn bounding_volume(&self) -> AABB {
        let WithAABB(ref t, ref g) = *self;

        g.aabb(t)
    }
}

impl<A: HasAABB>
HasBoundingVolume<AABB> for Rc<WithAABB<A>> {
    #[inline]
    fn bounding_volume(&self) -> AABB {
        self.g().aabb(self.m())
    }
}

impl<A: HasAABB>
HasBoundingVolume<AABB> for Rc<RefCell<WithAABB<A>>> {
    #[inline]
    fn bounding_volume(&self) -> AABB {
        let bself = self.borrow();

        bself.g().aabb(bself.m())
    }
}

#[cfg(all(dim3, f64, test))]
mod test {
    use super::AABB;
    use bounding_volume::BoundingVolume;
    use na::Vec3;

    #[test]
    fn test_merge() {
        let a = AABB::new(Vec3::new(10f64, 20.0, 30.0), Vec3::new(20.0, 30.0, 40.0));
        let b = AABB::new(Vec3::new(-10f64, -20.0, 35.0), Vec3::new(30.0, 50.0, 36.0));

        let merge = AABB::new(Vec3::new(-10.0f64, -20.0, 30.0), Vec3::new(30.0, 50.0, 40.0));

        assert!(a.merged(&b) == merge)
    }
}
