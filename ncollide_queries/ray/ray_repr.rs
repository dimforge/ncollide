use math::{Point, Vector, Isometry};
use entities::shape::{Ball, Capsule, Compound, Cone, ConvexHull, Cuboid, Cylinder, TriMesh, Polyline, Plane,
                      Segment, Triangle};
use entities::inspection::Shape;
use ray::{RayCast, Ray, RayIntersection};

macro_rules! dispatch(
    ($sself: ident.$name: ident($($argN: ident),*)) => {
        {
            let repr = $sself.desc();

            if let Some(b) = repr.as_shape::<Ball<<P::Vect as Vector>::Scalar>>() {
                b.$name($($argN,)*)
            }
            else if let Some(c) = repr.as_shape::<Capsule<<P::Vect as Vector>::Scalar>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.as_shape::<Compound<P, M>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.as_shape::<Cone<<P::Vect as Vector>::Scalar>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.as_shape::<ConvexHull<P>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.as_shape::<Cuboid<P::Vect>>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.as_shape::<Cylinder<<P::Vect as Vector>::Scalar>>() {
                c.$name($($argN,)*)
            }
            else if let Some(t) = repr.as_shape::<TriMesh<P>>() {
                t.$name($($argN,)*)
            }
            else if let Some(p) = repr.as_shape::<Polyline<P>>() {
                p.$name($($argN,)*)
            }
            else if let Some(p) = repr.as_shape::<Plane<P::Vect>>() {
                p.$name($($argN,)*)
            }
            else if let Some(s) = repr.as_shape::<Segment<P>>() {
                s.$name($($argN,)*)
            }
            else if let Some(t) = repr.as_shape::<Triangle<P>>() {
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

impl<P, M> RayCast<P, M> for Shape<P, M>
    where P: Point,
          M: Isometry<P, P::Vect> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<<P::Vect as Vector>::Scalar> {
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
