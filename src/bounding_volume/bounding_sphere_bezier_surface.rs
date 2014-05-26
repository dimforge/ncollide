use nalgebra::na::Transform;
use bounding_volume::{BoundingSphere, HasBoundingSphere};
use bounding_volume;
use geom::BezierSurface;
use math::Matrix;

impl HasBoundingSphere for BezierSurface {
    fn bounding_sphere(&self, m: &Matrix) -> BoundingSphere {
        let (center, radius) = bounding_volume::bounding_sphere(self.control_points());

        BoundingSphere::new(m.transform(&center), radius)
    }
}
