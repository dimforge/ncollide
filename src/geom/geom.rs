//! Enum grouping the most useful geometric shapes.

use std::unstable::intrinsics::TypeId;
use std::cast;
use ray::{RayCastWithTransform, Ray};
use volumetric::Volumetric;
use bounding_volume::{HasAABB, AABB};

pub trait Geom<N, V, M, II> : Volumetric<N, V, II>          +
                              HasAABB<N, V, M>              +
                              RayCastWithTransform<N, V, M> +
                              Any {
    fn duplicate(&self) -> ~Geom<N, V, M, II>;
}

pub trait ConcaveGeom<N, V, M, II> : Geom<N, V, M, II> {
    fn map_part_at(&self, uint, |&M, &Geom<N, V, M, II>| -> ());
    fn map_transformed_part_at(&self, m: &M, uint, |&M, &Geom<N, V, M, II>| -> ());

    // FIXME: replace those by a visitor?
    fn approx_interferences_with_aabb(&self, &AABB<N, V>, &mut ~[uint]);
    fn approx_interferences_with_ray(&self, &Ray<V>, &mut ~[uint]);
    // FIXME: kind of ad-hoc…
    fn aabb_at<'a>(&'a self, uint) -> &'a AABB<N, V>;
}

impl<T: 'static              +
        Send                 +
        Clone                +
        Volumetric<N, V, II> +
        HasAABB<N, V, M>     +
        RayCastWithTransform<N, V, M> +
        Any,
     N, V, M, II>
Geom<N, V, M, II> for T {
    fn duplicate(&self) -> ~Geom<N, V, M, II> {
        ~self.clone() as ~Geom<N, V, M, II>
    }
}
// FIXME: we need to implement that since AnyRefExt is only implemented for Any, and it does not
// seem possible to convert a &Geom to a &Any…
impl<'a, N, V, M, II> AnyRefExt<'a> for &'a Geom<N, V, M, II> {
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
    fn as_ref<T: 'static>(self) -> Option<&'a T> {
        if self.is::<T>() {
            Some(unsafe { cast::transmute(self.as_void_ptr()) })
        } else {
            None
        }
    }
}
