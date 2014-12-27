use na::{Transform, Translate};
use bounding_volume::BoundingVolume;
use bounding_volume::{BoundingSphere, HasBoundingSphere};
use shape::Compound;
use math::{Scalar, Point, Vect};


impl<N, P, V, M, M2> HasBoundingSphere<N, P, M2> for Compound<N, P, V, M>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M2: Transform<P> {
    #[inline]
    fn bounding_sphere(&self, m: &M2) -> BoundingSphere<N, P> {
        let shapes = self.shapes();

        let mut res = shapes[0].1.bounding_sphere(&shapes[0].0);

        for &(ref t, ref s) in shapes.slice_from(1).iter() {
            res.merge(&s.bounding_sphere(t));
        }

        BoundingSphere::new(m.transform(res.center()), res.radius())
    }
}
