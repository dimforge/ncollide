use std::num::Bounded;
use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::{Translation, Indexable};
use nalgebra::na;
use implicit::Implicit;
use bounding_volume::{HasBoundingVolume, BoundingVolume, LooseBoundingVolume};
use math::{Scalar, Vect, Matrix};

/// Traits of objects that can be approximated by an AABB.
pub trait HasAABB {
    /// The object’s AABB.
    fn aabb(&self, &Matrix) -> AABB;
}

/// An Axis Aligned Bounding Box.
///
/// # Parameter:
///   * `Vect` - type of the points of the bounding box. It determines the AABB dimension.
///   * `Scalar` - type of the one components of the aabb points.
#[deriving(Show, Eq, Clone, Encodable, Decodable)]
pub struct AABB {
    mins: Vect,
    maxs: Vect
}

impl AABB {
    /// Reference to the AABB point with the smallest components along each axis.
    #[inline]
    pub fn mins<'r>(&'r self) -> &'r Vect {
        &'r self.mins
    }

    /// Reference to the AABB point with the biggest components along each axis.
    #[inline]
    pub fn maxs<'r>(&'r self) -> &'r Vect {
        &'r self.maxs
    }
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
        let _M: Vect = Bounded::max_value();
        AABB {
            mins: Bounded::max_value(),
            maxs: -_M,
        }
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

impl Translation<Vect> for AABB
{
    fn translation(&self) -> Vect {
        (self.mins + self.maxs) * na::cast::<f64, Scalar>(0.5)
    }

    fn inv_translation(&self) -> Vect {
        -self.translation()
    }

    fn append_translation(&mut self, dv: &Vect) {
        self.mins = self.mins + *dv;
        self.maxs = self.maxs + *dv;
    }

    fn append_translation_cpy(aabb: &AABB, dv: &Vect) -> AABB {
        AABB::new(aabb.mins + *dv, aabb.maxs + *dv)
    }

    fn prepend_translation(&mut self, dv: &Vect) {
        self.mins = self.mins + *dv;
        self.maxs = self.maxs + *dv;
    }

    fn prepend_translation_cpy(aabb: &AABB, dv: &Vect) -> AABB {
        AABB::new(aabb.mins + *dv, aabb.maxs + *dv)
    }

    fn set_translation(&mut self, v: Vect) {
        let center = self.translation();

        self.mins = self.mins - center + v;
        self.maxs = self.maxs - center + v;
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

/// Builds the AABB of an implicit shape.
pub fn implicit_shape_aabb<I: Implicit<Vect, Matrix>>(m: &Matrix, i: &I) -> AABB {
        let mut resm:  Vect = na::zero();
        let mut resM:  Vect = na::zero();
        let mut basis: Vect = na::zero();

        for d in range(0, na::dim::<Vect>()) {
            // FIXME: this could be further improved iterating on `m`'s columns, and passing
            // Identity as the transformation matrix.
            basis.set(d, na::one());
            resM.set(d, i.support_point_without_margin(m, &basis).at(d));

            basis.set(d, -na::one::<Scalar>());
            resm.set(d, i.support_point_without_margin(m, &basis).at(d));

            basis.set(d, na::zero());
        }

        let margin = i.margin();
        AABB::new(resm - margin, resM + margin)
}

// FIXME: remove that to use `Transformed` istead?
/// Wrapper which implements `HasBoundingVolume<Matrix, AABB>` for objects implementing `HasAABB`.
#[deriving(Clone, Eq)]
pub struct WithAABB<A>(Matrix, A);

impl<A> WithAABB<A> {
    /// The transformation matrix of this shape.
    pub fn m<'a>(&'a self) -> &'a Matrix {
        let WithAABB(ref t, _) = *self;

        t
    }

    /// The local-space geometry of this shape.
    pub fn g<'a>(&'a self) -> &'a A {
        let WithAABB(_, ref g) = *self;

        g
    }
}

impl<A: HasAABB> HasBoundingVolume<AABB> for WithAABB<A> {
    fn bounding_volume(&self) -> AABB {
        let WithAABB(ref t, ref g) = *self;

        g.aabb(t)
    }
}

impl<A: HasAABB>
HasBoundingVolume<AABB> for Rc<WithAABB<A>> {
    fn bounding_volume(&self) -> AABB {
        self.g().aabb(self.m())
    }
}

impl<A: HasAABB>
HasBoundingVolume<AABB> for Rc<RefCell<WithAABB<A>>> {
    fn bounding_volume(&self) -> AABB {
        let bself = self.borrow();

        bself.g().aabb(bself.m())
    }
}

#[cfg(dim3, f64, test)]
mod test {
    use super::AABB;
    use bounding_volume::BoundingVolume;
    use nalgebra::na::Vec3;

    #[test]
    fn test_merge() {
        let a = AABB::new(Vec3::new(10f64, 20.0, 30.0), Vec3::new(20.0, 30.0, 40.0));
        let b = AABB::new(Vec3::new(-10f64, -20.0, 35.0), Vec3::new(30.0, 50.0, 36.0));

        let merge = AABB::new(Vec3::new(-10.0f64, -20.0, 30.0), Vec3::new(30.0, 50.0, 40.0));

        assert!(a.merged(&b) == merge)
    }
}
