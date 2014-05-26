use nalgebra::na::Transform;
use bounding_volume::{BoundingSphere, HasBoundingSphere, BoundingVolume};
use geom::Compound;
use math::Matrix;

impl HasBoundingSphere for Compound {
    #[inline]
    fn bounding_sphere(&self, m: &Matrix) -> BoundingSphere {
        let shapes = self.shapes();

        let mut res = shapes[0].ref1().bounding_sphere(shapes[0].ref0());

        for &(ref t, ref s) in shapes.slice_from(1).iter() {
            res.merge(&s.bounding_sphere(t));
        }

        BoundingSphere::new(m.transform(res.center()), res.radius())
    }
}
