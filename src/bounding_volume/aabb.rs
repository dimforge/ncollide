use nalgebra::traits::scalar_op::{ScalarAdd, ScalarSub};
//FIXME: use nalgebra::traits::basis::Basis;
use bounding_volume::bounding_volume::{HasBoundingVolume, BoundingVolume, LooseBoundingVolume};

/// Traits of objects approximable by an AABB.
pub trait HasAABB<V>
{
    /// The object’s AABB.
    fn aabb(&self) -> AABB<V>;
}

/// An Axis Aligned Bounding Box.
///
/// # Parameter:
///   * `V` - type of the points of the bounding box. It determines the AABB dimension.
#[deriving(ToStr, Eq, Clone)]
pub struct AABB<V>
{
    priv mins: V,
    priv maxs: V
}

impl<V: Ord> AABB<V>
{
    /// Creates a new AABB.
    ///
    /// # Arguments:
    ///   * `mins` - position of the point with the smallest coordinates.
    ///   * `maxs` - position of the point with the highest coordinates. Each component of `mins`
    ///   must be smaller than the related components of `maxs`.
    pub fn new(mins: V, maxs: V) -> AABB<V>
    {
        assert!(mins <= maxs);

        AABB {
            mins: mins,
            maxs: maxs
        }
    }
}

impl<V: Ord + Orderable + Clone> BoundingVolume for AABB<V>
{
    #[inline]
    fn intersects(&self, other: &AABB<V>) -> bool
    { self.mins <= other.maxs && self.maxs >= other.mins }

    #[inline]
    fn contains(&self, other: &AABB<V>) -> bool
    { self.mins <= other.mins && self.maxs >= other.maxs }

    #[inline]
    fn merge(&mut self, other: &AABB<V>)
    {
        self.mins = self.mins.min(&other.mins);
        self.maxs = self.maxs.max(&other.maxs);
    }

    #[inline]
    fn merged(&self, other: &AABB<V>) -> AABB<V>
    {
        AABB {
            mins: self.mins.min(&other.mins),
            maxs: self.maxs.max(&other.maxs)
        }
    }
}

impl<V: Clone + Ord + Orderable + ScalarAdd<N> + ScalarSub<N>, N>
LooseBoundingVolume<N> for AABB<V>
{
    #[inline]
    fn loosen(&mut self, amount: N)
    {
        self.mins.scalar_sub_inplace(&amount);
        self.maxs.scalar_add_inplace(&amount);
    }

    #[inline]
    fn loosened(&self, amount: N) -> AABB<V>
    {
        AABB {
            mins: self.mins.scalar_sub(&amount),
            maxs: self.maxs.scalar_add(&amount)
        }
    }
}

/// Wrapper which implements `HasBoundingVolume<AABB<V>>` for objects implementing `HasAABB<V>`.
#[deriving(Clone, Eq, DeepClone)]
pub struct WithAABB<A>(A);

impl<A: HasAABB<V>, V: Clone + Ord + Orderable + ScalarAdd<N> + ScalarSub<N>, N>
HasBoundingVolume<AABB<V>> for WithAABB<A>
{
    fn bounding_volume(&self) -> AABB<V>
    { self.aabb() }
}
