use na::Transform;
use bounding_volume::{BoundingSphere, HasBoundingSphere};
use bounding_volume;
use geom::Mesh;
use math::Matrix;

impl HasBoundingSphere for Mesh {
    #[inline]
    fn bounding_sphere(&self, m: &Matrix) -> BoundingSphere {
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(self.vertices().as_slice());

        BoundingSphere::new(m.transform(&center), radius)
    }
}
