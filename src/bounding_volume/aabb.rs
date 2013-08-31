use std::num::Zero;
use nalgebra::vec::{Vec, VecExt, AlgebraicVecExt, Basis, ScalarAdd, ScalarSub};
use nalgebra::mat::Translation;
use geom::Implicit;
use bounding_volume::{HasBoundingVolume, BoundingVolume, LooseBoundingVolume};

/// Traits of objects approximable by an AABB.
pub trait HasAABB<N, V, M> {
    /// The object’s AABB.
    fn aabb(&self, &M) -> AABB<N, V>;
}

/// An Axis Aligned Bounding Box.
///
/// # Parameter:
///   * `V` - type of the points of the bounding box. It determines the AABB dimension.
///   * `N` - type of the one components of the aabb points.
#[deriving(ToStr, Eq, Clone)]
pub struct AABB<N, V> {
    priv mins: V,
    priv maxs: V
}

impl<N, V> AABB<N, V> {
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

impl<V: VecExt<N>, N> AABB<N, V> {
    /// Creates a new AABB.
    ///
    /// # Arguments:
    ///   * `mins` - position of the point with the smallest coordinates.
    ///   * `maxs` - position of the point with the highest coordinates. Each component of `mins`
    ///   must be smaller than the related components of `maxs`.
    pub fn new(mins: V, maxs: V) -> AABB<N, V> {
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
    pub fn new_invalid() -> AABB<N, V> {
        let _M: V = Bounded::max_value();
        AABB {
            mins: Bounded::max_value(),
            maxs: -_M,
        }
    }
}

impl<N: Primitive + Orderable + ToStr,
     V: VecExt<N> + ToStr>
BoundingVolume for AABB<N, V> {
    #[inline]
    fn intersects(&self, other: &AABB<N, V>) -> bool {
        self.mins <= other.maxs && self.maxs >= other.mins
    }

    #[inline]
    fn contains(&self, other: &AABB<N, V>) -> bool {
        self.mins <= other.mins && self.maxs >= other.maxs
    }

    #[inline]
    fn merge(&mut self, other: &AABB<N, V>) {
        self.mins = self.mins.min(&other.mins);
        self.maxs = self.maxs.max(&other.maxs);
    }

    #[inline]
    fn merged(&self, other: &AABB<N, V>) -> AABB<N, V> {
        AABB {
            mins: self.mins.min(&other.mins),
            maxs: self.maxs.max(&other.maxs)
        }
    }
}

impl<V: VecExt<N>, N: NumCast> Translation<V> for AABB<N, V>
{
    fn translation(&self) -> V {
        (self.mins + self.maxs) / NumCast::from(2.0f64)
    }

    fn inv_translation(&self) -> V {
        -self.translation()
    }

    fn translate_by(&mut self, dv: &V) {
        self.mins = self.mins + *dv;
        self.maxs = self.maxs + *dv;
    }

    fn translated(&self, dv: &V) -> AABB<N, V> {
        AABB::new(self.mins + *dv, self.maxs + *dv)
    }
}

impl<N: Primitive + Orderable + ToStr,
     V: VecExt<N> + ToStr>
LooseBoundingVolume<N> for AABB<N, V> {
    #[inline]
    fn loosen(&mut self, amount: N) {
        self.mins.scalar_sub_inplace(&amount);
        self.maxs.scalar_add_inplace(&amount);
    }

    #[inline]
    fn loosened(&self, amount: N) -> AABB<N, V> {
        AABB {
            mins: self.mins.scalar_sub(&amount),
            maxs: self.maxs.scalar_add(&amount)
        }
    }
}

/// Builds the AABB of an implicit shape.
pub fn implicit_shape_aabb<N: Algebraic,
                           V: AlgebraicVecExt<N>,
                           M,
                           I: Implicit<N, V, M>>(
                           m: &M,
                           i: &I)
                           -> AABB<N, V> {
        let mut resm: V = Zero::zero();
        let mut resM: V = Zero::zero();

        // FIXME: optimize using Indexable?
        do Basis::canonical_basis() |basis: V| {
            resm = resm + basis * basis.dot(&i.support_point(m, &-basis));
            resM = resM + basis * basis.dot(&i.support_point(m, &basis));

            true
        }

        let res = AABB::new(resm, resM);

        res
}

// FIXME: remove that to use `Transformed` istead?
/// Wrapper which implements `HasBoundingVolume<M, AABB<N, V>>` for objects implementing `HasAABB<N, V, M>`.
#[deriving(Clone, Eq, DeepClone)]
pub struct WithAABB<M, A>(M, A);

impl<N: Primitive + Orderable + ToStr,
     V: VecExt<N> + Clone + ToStr,
     M,
     A: HasAABB<N, V, M>>
HasBoundingVolume<AABB<N, V>> for WithAABB<M, A> {
    fn bounding_volume(&self) -> AABB<N, V> {
        let WithAABB(ref t, ref g) = *self;

        g.aabb(t)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use nalgebra::vec::Vec3;

    #[test]
    fn test_merge() {
        let a = AABB::new(Vec3::new(10f64, 20.0, 30.0), Vec3::new(20.0, 30.0, 40.0));
        let b = AABB::new(Vec3::new(-10f64, -20.0, 35.0), Vec3::new(30.0, 50.0, 36.0));

        let merge = AABB::new(Vec3::new(-10.0f64, -20.0, 30.0), Vec3::new(30.0, 50.0, 40.0));

        assert!(a.merged(&b) == merge)
    }
}
