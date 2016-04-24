use na::Translation;
use math::{Point, Vector, Isometry};
use entities::shape::{Ball, Capsule, Compound, Cone, ConvexHull, Cuboid, Cylinder, TriMesh, Polyline, Plane,
                      Segment, Triangle};
use entities::inspection::Shape;
use point::{PointQuery, PointProjection};

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

impl<P, M> PointQuery<P, M> for Shape<P, M>
    where P: Point,
          M: Isometry<P, P::Vect> + Translation<P::Vect> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        dispatch!(self.project_point(m, pt, solid))
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> <P::Vect as Vector>::Scalar {
        dispatch!(self.distance_to_point(m, pt, solid))
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        dispatch!(self.contains_point(m, pt))
    }
}
