use na::Real;
use bounding_volume::{BoundingSphere, BoundingVolume, HasBoundingVolume};
use shape::Compound;
use math::Isometry;

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Compound<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let shapes = self.shapes();

        let mut res = shapes[0].1.bounding_sphere(&shapes[0].0);

        for &(ref t, ref s) in shapes[1..].iter() {
            res.merge(&s.bounding_sphere(t));
        }

        BoundingSphere::new(m * res.center(), res.radius())
    }
}
