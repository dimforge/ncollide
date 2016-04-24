extern crate nalgebra as na;
extern crate ncollide;

use std::mem;
use std::any::TypeId;
use na::{Vector2, Point2, Isometry2, Rotate};
use ncollide::inspection::{Shape, ShapeDesc2};
use ncollide::support_map::SupportMap;
use ncollide::geometry::{self, Proximity};
use ncollide::shape::Cuboid;
use ncollide::bounding_volume::{self, HasBoundingVolume, AABB2};

struct Ellipse {
    a: f32, // The first radius.
    b: f32  // The second radius.
}

impl SupportMap<Point2<f32>, Isometry2<f32>> for Ellipse {
    #[inline]
    fn support_point(&self, transform: &Isometry2<f32>, dir: &Vector2<f32>) -> Point2<f32> {
        // Bring `dir` into the ellipse's local frame.
        let local_dir = transform.inverse_rotate(&dir);

        // Compute the denominator.
        let denom = f32::sqrt(local_dir.x * local_dir.x * self.a * self.a +
                              local_dir.y * local_dir.y * self.b * self.b);

        // Compute the support point into the ellipse's local frame.
        let local_support_point = Point2::new(self.a * self.a * local_dir.x / denom,
                                              self.b * self.b * local_dir.y / denom);

        // Return the support point transformed back into the global frame.
        *transform * local_support_point
    }
}

impl Shape<Point2<f32>, Isometry2<f32>> for Ellipse {
    #[inline(always)]
    fn desc(&self) -> ShapeDesc2<f32> {
        unsafe {
            ShapeDesc2::new(
                // Dynamic type identifier for an ellipse.
                TypeId::of::<Ellipse>(),
                // Dynamic type identifier for a support-mapped object.
                TypeId::of::<&SupportMap<Point2<f32>, Isometry2<f32>>>(),
                // Informations for dynamic method dispatch of the SupportMap trait-object.
                mem::transmute(self as &SupportMap<Point2<f32>, Isometry2<f32>>)
            )
        }
    }
}

impl HasBoundingVolume<Isometry2<f32>, AABB2<f32>> for Ellipse {
    fn bounding_volume(&self, m: &Isometry2<f32>) -> AABB2<f32> {
        // Generic method to compute the aabb of a support-mapped shape.
        bounding_volume::support_map_aabb(m, self)
    }
}

fn main() {
    let ellipse = Ellipse { a: 2.0f32, b: 1.0 };
    let cuboid  = Cuboid::new(Vector2::new(1.0, 1.0));

    let ellipse_pos = na::one();
    let cuboid_pos  = Isometry2::new(Vector2::new(4.0, 0.0), na::zero());

    let dist = geometry::distance(&ellipse_pos, &ellipse, &cuboid_pos, &cuboid);
    let prox = geometry::proximity(&ellipse_pos, &ellipse, &cuboid_pos, &cuboid, 0.0);
    let ctct = geometry::contact(&ellipse_pos, &ellipse, &cuboid_pos, &cuboid, 0.0);

    assert!(na::approx_eq(&dist, &1.0));
    assert_eq!(prox, Proximity::Disjoint);
    assert!(ctct.is_none());
}
