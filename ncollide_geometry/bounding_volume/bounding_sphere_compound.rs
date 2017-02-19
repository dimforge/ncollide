use bounding_volume::{BoundingVolume, BoundingSphere, HasBoundingVolume};
use shape::Compound;
use math::{Point, Isometry};


impl<P, M, M2> HasBoundingVolume<M2, BoundingSphere<P>> for Compound<P, M>
    where P:  Point,
          M:  Isometry<P>,
          M2: Isometry<P> {
    #[inline]
    fn bounding_volume(&self, m: &M2) -> BoundingSphere<P> {
        let shapes = self.shapes();

        let mut res = shapes[0].1.bounding_sphere(&shapes[0].0);

        for &(ref t, ref s) in shapes[1 ..].iter() {
            res.merge(&s.bounding_sphere(t));
        }

        BoundingSphere::new(m.transform_point(res.center()), res.radius())
    }
}
