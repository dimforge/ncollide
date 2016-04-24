use na::Translate;
use bounding_volume::{self, HasBoundingVolume, AABB};
use math::{Point, Vector, Isometry};
use shape::{Ball, Capsule, Compound, Cone, ConvexHull, Cuboid, Cylinder, TriMesh, Polyline, Plane,
            Segment, Triangle};
use inspection::Shape;

impl<P, M> HasBoundingVolume<M, AABB<P>> for Shape<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        let repr = self.desc();

        if let Some(b) = repr.as_shape::<Ball<<P::Vect as Vector>::Scalar>>() {
            bounding_volume::aabb(b, m)
        }
        else if let Some(c) = repr.as_shape::<Capsule<<P::Vect as Vector>::Scalar>>() {
            bounding_volume::aabb(c, m)
        }
        else if let Some(c) = repr.as_shape::<Compound<P, M>>() {
            bounding_volume::aabb(c, m)
        }
        else if let Some(c) = repr.as_shape::<Cone<<P::Vect as Vector>::Scalar>>() {
            bounding_volume::aabb(c, m)
        }
        else if let Some(c) = repr.as_shape::<ConvexHull<P>>() {
            bounding_volume::aabb(c, m)
        }
        else if let Some(c) = repr.as_shape::<Cuboid<P::Vect>>() {
            bounding_volume::aabb(c, m)
        }
        else if let Some(c) = repr.as_shape::<Cylinder<<P::Vect as Vector>::Scalar>>() {
            bounding_volume::aabb(c, m)
        }
        else if let Some(t) = repr.as_shape::<TriMesh<P>>() {
            bounding_volume::aabb(t, m)
        }
        else if let Some(p) = repr.as_shape::<Polyline<P>>() {
            bounding_volume::aabb(p, m)
        }
        else if let Some(p) = repr.as_shape::<Plane<P::Vect>>() {
            bounding_volume::aabb(p, m)
        }
        else if let Some(s) = repr.as_shape::<Segment<P>>() {
            bounding_volume::aabb(s, m)
        }
        else if let Some(t) = repr.as_shape::<Triangle<P>>() {
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
