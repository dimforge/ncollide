use na::Translate;
use bounding_volume::{HasBoundingVolume, BoundingSphere};
use math::{Point, Vector, Isometry};
use shape::{Ball, Capsule, Compound, Cone, ConvexHull, Cuboid, Cylinder, TriMesh, Polyline, Plane,
            Segment, Triangle};
use inspection::Repr;


impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for Repr<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let repr = self.repr();

        if let Some(b) = repr.downcast_ref::<Ball<<P::Vect as Vector>::Scalar>>() {
            b.bounding_volume(m)
        }
        else if let Some(c) = repr.downcast_ref::<Capsule<<P::Vect as Vector>::Scalar>>() {
            c.bounding_volume(m)
        }
        else if let Some(c) = repr.downcast_ref::<Compound<P, M>>() {
            c.bounding_volume(m)
        }
        else if let Some(c) = repr.downcast_ref::<Cone<<P::Vect as Vector>::Scalar>>() {
            c.bounding_volume(m)
        }
        else if let Some(c) = repr.downcast_ref::<ConvexHull<P>>() {
            c.bounding_volume(m)
        }
        else if let Some(c) = repr.downcast_ref::<Cuboid<P::Vect>>() {
            c.bounding_volume(m)
        }
        else if let Some(c) = repr.downcast_ref::<Cylinder<<P::Vect as Vector>::Scalar>>() {
            c.bounding_volume(m)
        }
        else if let Some(t) = repr.downcast_ref::<TriMesh<P>>() {
            t.bounding_volume(m)
        }
        else if let Some(p) = repr.downcast_ref::<Polyline<P>>() {
            p.bounding_volume(m)
        }
        else if let Some(p) = repr.downcast_ref::<Plane<P::Vect>>() {
            p.bounding_volume(m)
        }
        else if let Some(s) = repr.downcast_ref::<Segment<P>>() {
            s.bounding_volume(m)
        }
        else if let Some(t) = repr.downcast_ref::<Triangle<P>>() {
            t.bounding_volume(m)
        }
        else {
            /*
             * XXX: dispatch by custom type.
             */
            unimplemented!()
        }
    }
}
