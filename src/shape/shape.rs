use std::raw::TraitObject;
use std::intrinsics::TypeId;
use std::mem;
use std::any::{Any, AnyRefExt};
use ray::{Ray, RayCast};
use bounding_volume::{HasBoundingSphere, HasAABB, AABB};

/// Trait (that should be) implemented by every shape.
pub trait Shape<N, P, V, M>: HasAABB<P, M>              +
                             HasBoundingSphere<N, P, M> +
                             RayCast<N, P, V, M>        +
                             Any {
    /// Duplicates (clones) this shape.
    fn duplicate(&self) -> Box<Shape<N, P, V, M> + Send + Sync>;
    /// Tells whether `Self` is a trait-object or an exact type.
    fn is_exact(&self) -> bool {
        true
    }
}

/// Trait implemented by concave, composite geometries.
///
/// A composite shape is composed of several `Shape`. Typically, it is a convex decomposition of
/// a concave shape.
pub trait ConcaveShape<N, P, V, M> : Shape<N, P, V, M> {
    /// Applies a function to each sub-shape of this concave shape.
    fn map_part_at<T>(&self, uint, |&M, &Shape<N, P, V, M>| -> T) -> T;
    /// Applies a transformation matrix and a function to each sub-shape of this concave
    /// shape.
    fn map_transformed_part_at<T>(&self, m: &M, uint, |&M, &Shape<N, P, V, M>| -> T) -> T;

    // FIXME: replace those by a visitor?
    /// Computes the indices of every sub-shape which might intersect a given AABB.
    fn approx_interferences_with_aabb(&self, &AABB<P>, &mut Vec<uint>);
    /// Computes the indices of every sub-shape which might intersect a given Ray.
    fn approx_interferences_with_ray(&self, &Ray<P, V>, &mut Vec<uint>);
    // FIXME: kind of ad-hoc…
    /// Gets the AABB of the shape identified by the index `i`.
    fn aabb_at(&self, i: uint) -> &AABB<P>;
}

impl<N, P, V, M, T> Shape<N, P, V, M> for T
    where T: 'static + Send + Sync + Clone + HasAABB<P, M> + HasBoundingSphere<N, P, M> + RayCast<N, P, V, M> + Any {
    #[inline]
    fn duplicate(&self) -> Box<Shape<N, P, V, M> + Send + Sync> {
        (box self.clone()) as Box<Shape<N, P, V, M> + Send + Sync>
    }
}
// FIXME: we need to implement that since AnyRefExt is only implemented for Any, and it does not
// seem possible to convert a &Shape to a &Any…
impl<'a, N, P, V, M> AnyRefExt<'a> for &'a Shape<N, P, V, M> + 'a {
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
