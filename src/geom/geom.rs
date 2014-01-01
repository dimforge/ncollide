//! Enum grouping the most useful geometric shapes.

use std::unstable::intrinsics::TypeId;
use std::cast;
use ray::{RayCastWithTransform, Ray};
use volumetric::Volumetric;
use bounding_volume::{HasAABB, AABB};
use math::M;

pub trait Geom : Volumetric           +
                 HasAABB              +
                 RayCastWithTransform +
                 Any {
    fn duplicate(&self) -> ~Geom;
}

pub trait ConcaveGeom : Geom {
    fn map_part_at(&self, uint, |&M, &Geom| -> ());
    fn map_transformed_part_at(&self, m: &M, uint, |&M, &Geom| -> ());

    // FIXME: replace those by a visitor?
    fn approx_interferences_with_aabb(&self, &AABB, &mut ~[uint]);
    fn approx_interferences_with_ray(&self, &Ray, &mut ~[uint]);
    // FIXME: kind of ad-hoc…
    fn aabb_at<'a>(&'a self, uint) -> &'a AABB;
}

impl<T: 'static + Send + Clone + Volumetric + HasAABB + RayCastWithTransform + Any>
Geom for T {
    fn duplicate(&self) -> ~Geom {
        ~self.clone() as ~Geom
    }
}
// FIXME: we need to implement that since AnyRefExt is only implemented for Any, and it does not
// seem possible to convert a &Geom to a &Any…
impl<'a> AnyRefExt<'a> for &'a Geom {
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
