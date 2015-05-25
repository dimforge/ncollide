use na::Translate;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use math::{Scalar, Point, Vect, Isometry};
use shape::{Ball, Capsule, Compound, Cone, Convex, Cuboid, Cylinder, TriMesh, Polyline, Plane,
            Segment, Triangle};
use inspection::Repr;


impl<P, M> HasBoundingSphere<P, M> for Repr<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<P> {
        let repr = self.repr();

        if let Some(b) = repr.downcast_ref::<Ball<<P::Vect as Vect>::Scalar>>() {
            b.bounding_sphere(m)
        }
        else if let Some(c) = repr.downcast_ref::<Capsule<<P::Vect as Vect>::Scalar>>() {
            c.bounding_sphere(m)
        }
        else if let Some(c) = repr.downcast_ref::<Compound<P, M>>() {
            c.bounding_sphere(m)
        }
        else if let Some(c) = repr.downcast_ref::<Cone<<P::Vect as Vect>::Scalar>>() {
            c.bounding_sphere(m)
        }
        else if let Some(c) = repr.downcast_ref::<Convex<P>>() {
            c.bounding_sphere(m)
        }
        else if let Some(c) = repr.downcast_ref::<Cuboid<P::Vect>>() {
            c.bounding_sphere(m)
        }
        else if let Some(c) = repr.downcast_ref::<Cylinder<<P::Vect as Vect>::Scalar>>() {
            c.bounding_sphere(m)
        }
        else if let Some(t) = repr.downcast_ref::<TriMesh<P>>() {
            t.bounding_sphere(m)
        }
        else if let Some(p) = repr.downcast_ref::<Polyline<P>>() {
            p.bounding_sphere(m)
        }
        else if let Some(p) = repr.downcast_ref::<Plane<P::Vect>>() {
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
