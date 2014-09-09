use nalgebra::na::Transform;
use bounding_volume::{BoundingSphere, HasBoundingSphere};
use bounding_volume;
use geom::Segment;
use math::Matrix;

impl HasBoundingSphere for Segment {
    #[inline]
    fn bounding_sphere(&self, m: &Matrix) -> BoundingSphere {
        let pts = [ self.a().clone(), self.b().clone() ];
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(pts.as_slice());

        BoundingSphere::new(m.transform(&center), radius)
    }
}
