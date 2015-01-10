use na::Translate;
use math::{Scalar, Point, Vect, Isometry};
use entities::shape::{Ball, Capsule, Compound, Cone, Convex, Cuboid, Cylinder, TriMesh, Polyline, Plane,
                      Segment, Triangle};
use entities::inspection::Repr;
use ray::{LocalRayCast, RayCast, Ray, RayIntersection};

macro_rules! dispatch(
    ($sself: ident.$name: ident($($argN: ident),*)) => {
        {
            let repr = $sself.repr();

            if let Some(b) = repr.downcast_ref::<Ball<N>>() {
                b.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Capsule<N>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Compound<N, P, V, M>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Cone<N>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Convex<P>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Cuboid<V>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Cylinder<N>>() {
                c.$name($($argN,)*)
            }
            else if let Some(t) = repr.downcast_ref::<TriMesh<N, P, V>>() {
                t.$name($($argN,)*)
            }
            else if let Some(p) = repr.downcast_ref::<Polyline<N, P, V>>() {
                p.$name($($argN,)*)
            }
            else if let Some(p) = repr.downcast_ref::<Plane<V>>() {
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

impl<'a, N, P, V, M> LocalRayCast<N, P, V> for Repr<N, P, V, M> + 'a
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<N> {
        dispatch!(self.toi_with_ray(ray, solid))
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        dispatch!(self.toi_and_normal_with_ray(ray, solid))
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        dispatch!(self.toi_and_normal_and_uv_with_ray(ray, solid))
    }

    #[inline]
    fn intersects_ray(&self, ray: &Ray<P, V>) -> bool {
        dispatch!(self.intersects_ray(ray))
    }
}

impl<'a, N, P, V, M> RayCast<N, P, V, M> for Repr<N, P, V, M> + 'a
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
}
