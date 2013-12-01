use std::num::{Zero, One};
use nalgebra::na::{Cast, VecExt, AlgebraicVecExt, ScalarAdd, ScalarSub, Translation};
use nalgebra::na;
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
#[deriving(ToStr, Eq, Clone, Encodable, Decodable)]
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

impl<N: Primitive + Orderable, V: VecExt<N>> BoundingVolume for AABB<N, V> {
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

impl<V: VecExt<N>, N: Cast<f32>> Translation<V> for AABB<N, V>
{
    fn translation(&self) -> V {
        (self.mins + self.maxs) / Cast::from(2.0)
    }

    fn inv_translation(&self) -> V {
        -self.translation()
    }

    fn append_translation(&mut self, dv: &V) {
        self.mins = self.mins + *dv;
        self.maxs = self.maxs + *dv;
    }

    fn append_translation_cpy(aabb: &AABB<N, V>, dv: &V) -> AABB<N, V> {
        AABB::new(aabb.mins + *dv, aabb.maxs + *dv)
    }

    fn prepend_translation(&mut self, dv: &V) {
        self.mins = self.mins + *dv;
        self.maxs = self.maxs + *dv;
    }

    fn prepend_translation_cpy(aabb: &AABB<N, V>, dv: &V) -> AABB<N, V> {
        AABB::new(aabb.mins + *dv, aabb.maxs + *dv)
    }

    fn set_translation(&mut self, v: V) {
        let center = self.translation();

        self.mins = self.mins - center + v;
        self.maxs = self.maxs - center + v;
    }
}

impl<N: Primitive + Orderable,
     V: VecExt<N>>
LooseBoundingVolume<N> for AABB<N, V> {
    #[inline]
    fn loosen(&mut self, amount: N) {
        self.mins = self.mins.sub_s(&amount);
        self.maxs = self.maxs.add_s(&amount);
    }

    #[inline]
    fn loosened(&self, amount: N) -> AABB<N, V> {
        AABB {
            mins: self.mins.sub_s(&amount),
            maxs: self.maxs.add_s(&amount)
        }
    }
}

/// Builds the AABB of an implicit shape.
pub fn implicit_shape_aabb<N: Algebraic + One + Zero + Neg<N>,
                           V: AlgebraicVecExt<N>,
                           M,
                           I: Implicit<N, V, M>>(
                           m: &M,
                           i: &I)
                           -> AABB<N, V> {
        let mut resm: V  = na::zero();
        let mut resM: V  = na::zero();
        let mut basis: V = na::zero();

        for d in range(0, na::dim::<V>()) {
            // FIXME: this could be further improved iterating on `m`'s columns, and passing
            // Identity as the transformation matrix.
            basis.set(d, na::one());
            resM.set(d, i.support_point(m, &basis).at(d));

            basis.set(d, -na::one::<N>());
            resm.set(d, i.support_point(m, &basis).at(d));

            basis.set(d, na::zero());
        }

        let res = AABB::new(resm, resM);

        res
}

// FIXME: remove that to use `Transformed` istead?
/// Wrapper which implements `HasBoundingVolume<M, AABB<N, V>>` for objects implementing `HasAABB<N, V, M>`.
#[deriving(Clone, Eq, DeepClone)]
pub struct WithAABB<M, A>(M, A);

impl<N: Primitive + Orderable,
     V: VecExt<N> + Clone,
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
    use super::AABB;
    use nalgebra::na::Vec3;

    #[test]
    fn test_merge() {
        let a = AABB::new(Vec3::new(10f64, 20.0, 30.0), Vec3::new(20.0, 30.0, 40.0));
        let b = AABB::new(Vec3::new(-10f64, -20.0, 35.0), Vec3::new(30.0, 50.0, 36.0));

        let merge = AABB::new(Vec3::new(-10.0f64, -20.0, 30.0), Vec3::new(30.0, 50.0, 40.0));

        assert!(a.merged(&b) == merge)
    }
}
