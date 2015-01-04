use na::Translate;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use math::{Scalar, Point, Vect, Isometry};
use shape::{Ball, Capsule, Compound, Cone, Convex, Cuboid, Cylinder, TriMesh, Polyline, Plane,
            Segment, Triangle};
use inspection::Repr;


impl<'a, N, P, V, M> HasBoundingSphere<N, P, M> for Repr<N, P, V, M> + 'a
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<N, P> {
        let repr = self.repr();

        if let Some(b) = repr.downcast_ref::<Ball<N>>() {
            b.bounding_sphere(m)
        }
        else if let Some(c) = repr.downcast_ref::<Capsule<N>>() {
            c.bounding_sphere(m)
        }
        else if let Some(c) = repr.downcast_ref::<Compound<N, P, V, M>>() {
            c.bounding_sphere(m)
        }
        else if let Some(c) = repr.downcast_ref::<Cone<N>>() {
            c.bounding_sphere(m)
        }
        else if let Some(c) = repr.downcast_ref::<Convex<P>>() {
            c.bounding_sphere(m)
        }
        else if let Some(c) = repr.downcast_ref::<Cuboid<V>>() {
            c.bounding_sphere(m)
        }
        else if let Some(c) = repr.downcast_ref::<Cylinder<N>>() {
            c.bounding_sphere(m)
        }
        else if let Some(t) = repr.downcast_ref::<TriMesh<N, P, V>>() {
            t.bounding_sphere(m)
        }
        else if let Some(p) = repr.downcast_ref::<Polyline<N, P, V>>() {
            p.bounding_sphere(m)
        }
        else if let Some(p) = repr.downcast_ref::<Plane<V>>() {
            p.bounding_sphere(m)
        }
        else if let Some(s) = repr.downcast_ref::<Segment<P>>() {
            s.bounding_sphere(m)
        }
        else if let Some(t) = repr.downcast_ref::<Triangle<P>>() {
            t.bounding_sphere(m)
        }
        else {
            /*
             * XXX: dispatch by custom type.
             */
            unimplemented!()
        }
    }
}
