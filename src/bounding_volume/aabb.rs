use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::{Translation, Indexable};
use nalgebra::na;
use implicit::Implicit;
use bounding_volume::{HasBoundingVolume, BoundingVolume, LooseBoundingVolume};
use math::{N, V, M};

/// Traits of objects approximable by an AABB.
pub trait HasAABB {
    /// The object’s AABB.
    fn aabb(&self, &M) -> AABB;
}

/// An Axis Aligned Bounding Box.
///
/// # Parameter:
///   * `V` - type of the points of the bounding box. It determines the AABB dimension.
///   * `N` - type of the one components of the aabb points.
#[deriving(ToStr, Eq, Clone, Encodable, Decodable)]
pub struct AABB {
    priv mins: V,
    priv maxs: V
}

impl AABB {
    /// Reference to the AABB point with the smallest components along each axis.
    #[inline]
    pub fn mins<'r>(&'r self) -> &'r V {
        &'r self.mins
    }

    /// Reference to the AABB point with the bigest components along each axis.
    #[inline]
    pub fn maxs<'r>(&'r self) -> &'r V {
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
    pub fn new(mins: V, maxs: V) -> AABB {
        assert!(mins <= maxs);

        AABB {
            mins: mins,
            maxs: maxs
        }
    }

    /// Creates an invalid AABB with:
    ///     * `mins = Bounded::max_value()`
    ///     * `maxs = Bounded::max_value()`.
    /// This is useful to build aabb using merges.
    pub fn new_invalid() -> AABB {
        let _M: V = Bounded::max_value();
        AABB {
            mins: Bounded::max_value(),
            maxs: -_M,
        }
    }
}

impl BoundingVolume for AABB {
    #[inline]
    fn intersects(&self, other: &AABB) -> bool {
        self.mins <= other.maxs && self.maxs >= other.mins
    }

    #[inline]
    fn contains(&self, other: &AABB) -> bool {
        self.mins <= other.mins && self.maxs >= other.maxs
    }

    #[inline]
    fn merge(&mut self, other: &AABB) {
        self.mins = self.mins.min(&other.mins);
        self.maxs = self.maxs.max(&other.maxs);
    }

    #[inline]
    fn merged(&self, other: &AABB) -> AABB {
        AABB {
            mins: self.mins.min(&other.mins),
            maxs: self.maxs.max(&other.maxs)
        }
    }
}

impl Translation<V> for AABB
{
    fn translation(&self) -> V {
        (self.mins + self.maxs) * na::cast::<f64, N>(0.5)
    }

    fn inv_translation(&self) -> V {
        -self.translation()
    }

    fn append_translation(&mut self, dv: &V) {
        self.mins = self.mins + *dv;
        self.maxs = self.maxs + *dv;
    }

    fn append_translation_cpy(aabb: &AABB, dv: &V) -> AABB {
        AABB::new(aabb.mins + *dv, aabb.maxs + *dv)
    }

    fn prepend_translation(&mut self, dv: &V) {
        self.mins = self.mins + *dv;
        self.maxs = self.maxs + *dv;
    }

    fn prepend_translation_cpy(aabb: &AABB, dv: &V) -> AABB {
        AABB::new(aabb.mins + *dv, aabb.maxs + *dv)
    }

    fn set_translation(&mut self, v: V) {
        let center = self.translation();

        self.mins = self.mins - center + v;
        self.maxs = self.maxs - center + v;
    }
}

impl LooseBoundingVolume for AABB {
    #[inline]
    fn loosen(&mut self, amount: N) {
        self.mins = self.mins - amount;
        self.maxs = self.maxs + amount;
    }

    #[inline]
    fn loosened(&self, amount: N) -> AABB {
        AABB {
            mins: self.mins - amount,
            maxs: self.maxs + amount
        }
    }
}

/// Builds the AABB of an implicit shape.
pub fn implicit_shape_aabb<I: Implicit<V, M>>(m: &M, i: &I) -> AABB {
        let mut resm:  V = na::zero();
        let mut resM:  V = na::zero();
        let mut basis: V = na::zero();

        for d in range(0, na::dim::<V>()) {
            // FIXME: this could be further improved iterating on `m`'s columns, and passing
            // Identity as the transformation matrix.
            basis.set(d, na::one());
            resM.set(d, i.support_point_without_margin(m, &basis).at(d));

            basis.set(d, -na::one::<N>());
            resm.set(d, i.support_point_without_margin(m, &basis).at(d));

            basis.set(d, na::zero());
        }

        let margin = i.margin();
        AABB::new(resm - margin, resM + margin)
}

// FIXME: remove that to use `Transformed` istead?
/// Wrapper which implements `HasBoundingVolume<M, AABB>` for objects implementing `HasAABB`.
#[deriving(Clone, Eq, DeepClone)]
pub struct WithAABB<A>(M, A);

impl<A> WithAABB<A> {
    /// The transformation matrix of this shape.
    pub fn m<'a>(&'a self) -> &'a M {
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
        let bself = self.borrow();

        bself.g().aabb(bself.m())
    }
}

impl<A: HasAABB>
HasBoundingVolume<AABB> for Rc<RefCell<WithAABB<A>>> {
    fn bounding_volume(&self) -> AABB {
        let bself = self.borrow().borrow();

        bself.get().g().aabb(bself.get().m())
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
