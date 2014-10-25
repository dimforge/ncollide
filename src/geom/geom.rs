use std::raw::TraitObject;
use std::intrinsics::TypeId;
use std::mem;
use std::any::{Any, AnyRefExt};
use ray::{Ray, RayCast};
use bounding_volume::{HasBoundingSphere, HasAABB, AABB};

/// Trait (that should be) implemented by every geometry.
pub trait Geom<N, P, V, M>: HasAABB<P, M>           +
                         HasBoundingSphere<N, P, M> +
                         RayCast<N, P, V, M>     +
                         Any {
    /// Duplicates (clones) this geometry.
    fn duplicate(&self) -> Box<Geom<N, P, V, M> + Send>;
}

/// Trait implemented by concave, composite geometries.
///
/// A composite geometry is composed of several `Geom`. Typically, it is a convex decomposition of
/// a concave geometry.
pub trait ConcaveGeom<N, P, V, M> : Geom<N, P, V, M> {
    /// Applies a function to each sub-geometry of this concave geometry.
    fn map_part_at<T>(&self, uint, |&M, &Geom<N, P, V, M>| -> T) -> T;
    /// Applies a transformation matrix and a function to each sub-geometry of this concave
    /// geometry.
    fn map_transformed_part_at<T>(&self, m: &M, uint, |&M, &Geom<N, P, V, M>| -> T) -> T;

    // FIXME: replace those by a visitor?
    /// Computes the indices of every sub-geometry which might intersect a given AABB.
    fn approx_interferences_with_aabb(&self, &AABB<P>, &mut Vec<uint>);
    /// Computes the indices of every sub-geometry which might intersect a given Ray.
    fn approx_interferences_with_ray(&self, &Ray<P, V>, &mut Vec<uint>);
    // FIXME: kind of ad-hoc…
    /// Gets the AABB of the geometry identified by the index `i`.
    fn aabb_at(&self, i: uint) -> &AABB<P>;
}

impl<N, P, V, M, T> Geom<N, P, V, M> for T
    where T: 'static + Send + Clone + HasAABB<P, M> + HasBoundingSphere<N, P, M> + RayCast<N, P, V, M> + Any {
    #[inline]
    fn duplicate(&self) -> Box<Geom<N, P, V, M> + Send> {
        (box self.clone()) as Box<Geom<N, P, V, M> + Send>
    }
}
// FIXME: we need to implement that since AnyRefExt is only implemented for Any, and it does not
// seem possible to convert a &Geom to a &Any…
impl<'a, N, P, V, M> AnyRefExt<'a> for &'a Geom<N, P, V, M> + 'a {
    #[inline]
    fn is<T: 'static>(self) -> bool {
        // Get TypeId of the type this function is instantiated with
        let t = TypeId::of::<T>();

        // Get TypeId of the type in the trait object
        let boxed = self.get_type_id();

        // Compare both TypeIds on equality
        t == boxed
    }

    #[inline]
    fn downcast_ref<T: 'static>(self) -> Option<&'a T> {
        if self.is::<T>() {
            unsafe {
                let to: TraitObject = mem::transmute_copy(&self);

                Some(mem::transmute(to.data))
            }
        } else {
            None
        }
    }
}
