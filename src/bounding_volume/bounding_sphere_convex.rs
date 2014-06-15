use nalgebra::na::Transform;
use bounding_volume::{BoundingSphere, HasBoundingSphere};
use bounding_volume;
use geom::Convex;
use math::Matrix;

impl HasBoundingSphere for Convex {
    #[inline]
    fn bounding_sphere(&self, m: &Matrix) -> BoundingSphere {
        let (center, radius) = bounding_volume::bounding_sphere(self.pts().as_slice());

        BoundingSphere::new(m.transform(&center), radius + self.margin())
    }
}
