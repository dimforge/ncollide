use na::{Transform, Translate};
use bounding_volume::{self, BoundingVolume, BoundingSphere, HasBoundingVolume};
use shape::Compound;
use math::{Point, Isometry};


impl<P, M, M2> HasBoundingVolume<M2, BoundingSphere<P>> for Compound<P, M>
    where P:  Point,
          P::Vect: Translate<P>,
          M:  Isometry<P, P::Vect>,
          M2: Transform<P> + Translate<P> {
    #[inline]
    fn bounding_volume(&self, m: &M2) -> BoundingSphere<P> {
        let shapes = self.shapes();

        let mut res = bounding_volume::bounding_sphere(&**shapes[0].1, &shapes[0].0);

        for &(ref t, ref s) in shapes[1 ..].iter() {
            res.merge(&bounding_volume::bounding_sphere(&***s, t));
        }

        BoundingSphere::new(m.transform(res.center()), res.radius())
    }
}
