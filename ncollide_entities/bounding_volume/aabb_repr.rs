use na::Translate;
use bounding_volume::{self, HasBoundingVolume, AABB};
use math::{Point, Vect, Isometry};
use shape::{Ball, Capsule, Compound, Cone, ConvexHull, Cuboid, Cylinder, TriMesh, Polyline, Plane,
            Segment, Triangle};
use inspection::Repr;

impl<P, M> HasBoundingVolume<M, AABB<P>> for Repr<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        let repr = self.repr();

        if let Some(b) = repr.downcast_ref::<Ball<<P::Vect as Vect>::Scalar>>() {
            bounding_volume::aabb(b, m)
        }
        else if let Some(c) = repr.downcast_ref::<Capsule<<P::Vect as Vect>::Scalar>>() {
            bounding_volume::aabb(c, m)
        }
        else if let Some(c) = repr.downcast_ref::<Compound<P, M>>() {
            bounding_volume::aabb(c, m)
        }
        else if let Some(c) = repr.downcast_ref::<Cone<<P::Vect as Vect>::Scalar>>() {
            bounding_volume::aabb(c, m)
        }
        else if let Some(c) = repr.downcast_ref::<ConvexHull<P>>() {
            bounding_volume::aabb(c, m)
        }
        else if let Some(c) = repr.downcast_ref::<Cuboid<P::Vect>>() {
            bounding_volume::aabb(c, m)
        }
        else if let Some(c) = repr.downcast_ref::<Cylinder<<P::Vect as Vect>::Scalar>>() {
            bounding_volume::aabb(c, m)
        }
        else if let Some(t) = repr.downcast_ref::<TriMesh<P>>() {
            bounding_volume::aabb(t, m)
        }
        else if let Some(p) = repr.downcast_ref::<Polyline<P>>() {
            bounding_volume::aabb(p, m)
        }
        else if let Some(p) = repr.downcast_ref::<Plane<P::Vect>>() {
            bounding_volume::aabb(p, m)
        }
        else if let Some(s) = repr.downcast_ref::<Segment<P>>() {
            bounding_volume::aabb(s, m)
        }
        else if let Some(t) = repr.downcast_ref::<Triangle<P>>() {
            bounding_volume::aabb(t, m)
        }
        else {
            /*
             * XXX: dispatch by custom type.
             */
            unimplemented!()
        }
    }
}
