use na::Translate;
use math::{Scalar, Point, Vect, Isometry};
use entities::shape::{Ball, Capsule, Compound, Cone, Convex, Cuboid, Cylinder, TriMesh, Polyline, Plane,
                      Segment, Triangle};
use entities::inspection::Repr;
use point::{LocalPointQuery, PointQuery};

macro_rules! dispatch(
    ($sself: ident.$name: ident($arg0: ident $(, $argN: ident)*)) => {
        {
            let repr = $sself.repr();

            if let Some(b) = repr.downcast_ref::<Ball<N>>() {
                b.$name($arg0 $(,$argN)*)
            }
            else if let Some(c) = repr.downcast_ref::<Capsule<N>>() {
                c.$name($arg0 $(, $argN)*)
            }
            else if let Some(c) = repr.downcast_ref::<Compound<N, P, V, M>>() {
                c.$name($arg0 $(, $argN)*)
            }
            else if let Some(c) = repr.downcast_ref::<Cone<N>>() {
                c.$name($arg0 $(, $argN)*)
            }
            else if let Some(c) = repr.downcast_ref::<Convex<P>>() {
                c.$name($arg0 $(, $argN)*)
            }
            else if let Some(c) = repr.downcast_ref::<Cuboid<V>>() {
                c.$name($arg0 $(, $argN)*)
            }
            else if let Some(c) = repr.downcast_ref::<Cylinder<N>>() {
                c.$name($arg0 $(, $argN)*)
            }
            else if let Some(t) = repr.downcast_ref::<TriMesh<N, P, V>>() {
                t.$name($arg0 $(, $argN)*)
            }
            else if let Some(p) = repr.downcast_ref::<Polyline<N, P, V>>() {
                p.$name($arg0 $(, $argN)*)
            }
            else if let Some(p) = repr.downcast_ref::<Plane<V>>() {
                p.$name($arg0 $(, $argN)*)
            }
            else if let Some(s) = repr.downcast_ref::<Segment<P>>() {
                s.$name($arg0 $(, $argN)*)
            }
            else if let Some(t) = repr.downcast_ref::<Triangle<P>>() {
                t.$name($arg0 $(, $argN)*)
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

impl<'a, N, P, V, M> LocalPointQuery<N, P> for Repr<N, P, V, M> + 'a
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
    #[inline]
    fn project_point(&self, pt: &P, solid: bool) -> P {
        dispatch!(self.project_point(pt, solid))
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> N {
        dispatch!(self.distance_to_point(pt))
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        dispatch!(self.contains_point(pt))
    }
}

impl<'a, N, P, V, M> PointQuery<N, P, M> for Repr<N, P, V, M> + 'a
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
}
