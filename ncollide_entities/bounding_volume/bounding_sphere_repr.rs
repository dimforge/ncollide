use na::Translate;
use bounding_volume::{HasBoundingVolume, BoundingSphere};
use math::{Point, Vector, Isometry};
use shape::{Ball, Capsule, Compound, Cone, ConvexHull, Cuboid, Cylinder, TriMesh, Polyline, Plane,
            Segment, Triangle};
use inspection::Shape;


impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for Shape<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let repr = self.desc();

        if let Some(b) = repr.as_shape::<Ball<<P::Vect as Vector>::Scalar>>() {
            b.bounding_volume(m)
        }
        else if let Some(c) = repr.as_shape::<Capsule<<P::Vect as Vector>::Scalar>>() {
            c.bounding_volume(m)
        }
        else if let Some(c) = repr.as_shape::<Compound<P, M>>() {
            c.bounding_volume(m)
        }
        else if let Some(c) = repr.as_shape::<Cone<<P::Vect as Vector>::Scalar>>() {
            c.bounding_volume(m)
        }
        else if let Some(c) = repr.as_shape::<ConvexHull<P>>() {
            c.bounding_volume(m)
        }
        else if let Some(c) = repr.as_shape::<Cuboid<P::Vect>>() {
            c.bounding_volume(m)
        }
        else if let Some(c) = repr.as_shape::<Cylinder<<P::Vect as Vector>::Scalar>>() {
            c.bounding_volume(m)
        }
        else if let Some(t) = repr.as_shape::<TriMesh<P>>() {
            t.bounding_volume(m)
        }
        else if let Some(p) = repr.as_shape::<Polyline<P>>() {
            p.bounding_volume(m)
        }
        else if let Some(p) = repr.as_shape::<Plane<P::Vect>>() {
            p.bounding_volume(m)
        }
        else if let Some(s) = repr.as_shape::<Segment<P>>() {
            s.bounding_volume(m)
        }
        else if let Some(t) = repr.as_shape::<Triangle<P>>() {
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
