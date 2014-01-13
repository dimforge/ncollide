//! Enum grouping the most useful geometric shapes.

use std::unstable::intrinsics::TypeId;
use std::cast;
use ray::{RayCastWithTransform, Ray};
use volumetric::Volumetric;
use bounding_volume::{HasAABB, AABB};
use math::M;

/// Trait implemented by each geometry suported by `ncollide`.
pub trait Geom : Volumetric           +
                 HasAABB              +
                 RayCastWithTransform +
                 Any {
    /// Duplicates (clones) this geometry.
    fn duplicate(&self) -> ~Geom;
}

/// Trait implemented by concave, composite geometries suported by `ncollide`.
///
/// A composite geometry is composed of several `Geom`. Typically, it is a convex decomposition of
/// a concave geometry.
pub trait ConcaveGeom : Geom {
    /// Applies a function to each sub-geometry of this concave geometry.
    fn map_part_at(&self, uint, |&M, &Geom| -> ());
    /// Applies a transformation matrix and a function to each sub-geometry of this concave
    /// geometry.
    fn map_transformed_part_at(&self, m: &M, uint, |&M, &Geom| -> ());

    // FIXME: replace those by a visitor?
    /// Computes the indices of every sub-geometry which might intersect a given AABB.
    fn approx_interferences_with_aabb(&self, &AABB, &mut ~[uint]);
    /// Computes the indices of every sub-geometry which might intersect a given Ray.
    fn approx_interferences_with_ray(&self, &Ray, &mut ~[uint]);
    // FIXME: kind of ad-hoc…
    /// Gets the AABB a the geometry identified by the index `i`.
    fn aabb_at<'a>(&'a self, i: uint) -> &'a AABB;
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
