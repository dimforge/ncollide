use nalgebra::traits::scalar_op::{ScalarAdd, ScalarSub, ScalarDiv};
use nalgebra::traits::translation::Translation;
use bounding_volume::bounding_volume::{HasBoundingVolume, BoundingVolume, LooseBoundingVolume};

/// Traits of objects approximable by an AABB.
pub trait HasAABB<N, V> {
    /// The object’s AABB.
    fn aabb(&self) -> AABB<N, V>;
}

/// An Axis Aligned Bounding Box.
///
/// # Parameter:
///   * `V` - type of the points of the bounding box. It determines the AABB dimension.
#[deriving(ToStr, Eq, Clone)]
pub struct AABB<N, V> {
    priv mins: V,
    priv maxs: V
}

impl<V: Ord + ScalarDiv<N>, N> AABB<N, V> {
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
}

impl<V: Ord + Orderable + Clone, N> BoundingVolume for AABB<N, V> {
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

impl<V: Add<V, V> + Neg<V> + ScalarDiv<N>, N: NumCast> Translation<V> for AABB<N, V>
{
    fn translation(&self) -> V {
        (self.mins + self.maxs).scalar_div(&NumCast::from(2.0f64))
    }

    fn inv_translation(&self) -> V {
        -self.translation()
    }

    fn translate_by(&mut self, dv: &V) {
        self.mins = self.mins + *dv;
        self.maxs = self.maxs + *dv;
    }
}

impl<V: Clone + Ord + Orderable + ScalarAdd<N> + ScalarSub<N>, N>
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

/// Wrapper which implements `HasBoundingVolume<AABB<N, V>>` for objects implementing `HasAABB<N, V>`.
#[deriving(Clone, Eq, DeepClone)]
pub struct WithAABB<A>(A);

impl<A: HasAABB<N, V>, V: Clone + Ord + Orderable + ScalarAdd<N> + ScalarSub<N>, N>
HasBoundingVolume<AABB<N, V>> for WithAABB<A> {
    fn bounding_volume(&self) -> AABB<N, V> {
        self.aabb()
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
