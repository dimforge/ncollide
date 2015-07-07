use math::{Scalar, Point, Vect, Isometry};
use entities::shape::{Ball, Capsule, Compound, Cone, Convex, Cuboid, Cylinder, TriMesh, Polyline, Plane,
                      Segment, Triangle};
use entities::inspection::Repr;
use ray::{RayCast, Ray, RayIntersection};

macro_rules! dispatch(
    ($sself: ident.$name: ident($($argN: ident),*)) => {
        {
            let repr = $sself.repr();

            if let Some(b) = repr.downcast_ref::<Ball<<P::Vect as Vect>::Scalar>>() {
                b.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Capsule<<P::Vect as Vect>::Scalar>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Compound<P, M>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Cone<<P::Vect as Vect>::Scalar>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Convex<P>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Cuboid<P::Vect>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Cylinder<<P::Vect as Vect>::Scalar>>() {
                c.$name($($argN,)*)
            }
            else if let Some(t) = repr.downcast_ref::<TriMesh<P>>() {
                t.$name($($argN,)*)
            }
            else if let Some(p) = repr.downcast_ref::<Polyline<P>>() {
                p.$name($($argN,)*)
            }
            else if let Some(p) = repr.downcast_ref::<Plane<P::Vect>>() {
                p.$name($($argN,)*)
            }
            else if let Some(s) = repr.downcast_ref::<Segment<P>>() {
                s.$name($($argN,)*)
            }
            else if let Some(t) = repr.downcast_ref::<Triangle<P>>() {
                t.$name($($argN,)*)
            }
            else {
                /*
                 * XXX: dispatch by custom type.
                 */
                unimplemented!()
            }
        }
    }
);

impl<P, M> RayCast<P, M> for Repr<P, M>
    where P: Point,
          M: Isometry<P, P::Vect> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<<P::Vect as Vect>::Scalar> {
        dispatch!(self.toi_with_ray(m, ray, solid))
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        dispatch!(self.toi_and_normal_with_ray(m, ray, solid))
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        dispatch!(self.toi_and_normal_and_uv_with_ray(m, ray, solid))
    }

    #[inline]
    fn intersects_ray(&self, m: &M, ray: &Ray<P>) -> bool {
        dispatch!(self.intersects_ray(m, ray))
    }
}
