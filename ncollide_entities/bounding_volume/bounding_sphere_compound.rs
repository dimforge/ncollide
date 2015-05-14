use na::{Transform, Translate};
use bounding_volume::{BoundingVolume, BoundingSphere, HasBoundingSphere};
use shape::Compound;
use math::{Point, Vect, Isometry};


impl<P, M, M2> HasBoundingSphere<P, M2> for Compound<P, M>
    where P:  Point,
          P::Vect: Translate<P>,
          M:  Isometry<P, P::Vect>,
          M2: Transform<P> + Translate<P> {
    #[inline]
    fn bounding_sphere(&self, m: &M2) -> BoundingSphere<P> {
        let shapes = self.shapes();

        let mut res = shapes[0].1.bounding_sphere(&shapes[0].0);

        for &(ref t, ref s) in shapes[1 ..].iter() {
            res.merge(&s.bounding_sphere(t));
        }

        BoundingSphere::new(m.transform(res.center()), res.radius())
    }
}
