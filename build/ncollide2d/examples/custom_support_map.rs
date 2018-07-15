extern crate alga;
#[macro_use]
extern crate approx;
extern crate nalgebra as na;
extern crate ncollide2d;

use alga::linear::ProjectiveTransformation;
use na::{Isometry2, Point2, Vector2};
use ncollide2d::bounding_volume::{self, AABB};
use ncollide2d::query::{self, Proximity};
use ncollide2d::shape::SupportMap;
use ncollide2d::shape::{Cuboid, Shape};

struct Ellipse {
    a: f32, // The first radius.
    b: f32, // The second radius.
}

impl SupportMap<f32> for Ellipse {
    fn local_support_point(&self, dir: &Vector2<f32>) -> Point2<f32> {
        // Compute the denominator.
        let denom = f32::sqrt(dir.x * dir.x * self.a * self.a + dir.y * dir.y * self.b * self.b);

        // Compute the support point into the ellipse's local frame.
        Point2::new(
            self.a * self.a * dir.x / denom,
            self.b * self.b * dir.y / denom,
        )
    }
}

impl Shape<f32> for Ellipse {
    fn aabb(&self, m: &Isometry2<f32>) -> AABB<f32> {
        // Generic method to compute the aabb of a support-mapped shape.
        bounding_volume::support_map_aabb(m, self)
    }

    fn as_support_map(&self) -> Option<&SupportMap<f32>> {
        Some(self)
    }
}

fn main() {
    let ellipse = Ellipse { a: 2.0f32, b: 1.0 };
    let cuboid = Cuboid::new(Vector2::new(1.0, 1.0));

    let ellipse_pos = na::one();
    let cuboid_pos = Isometry2::new(Vector2::new(4.0, 0.0), na::zero());

    let dist = query::distance(&ellipse_pos, &ellipse, &cuboid_pos, &cuboid);
    let prox = query::proximity(&ellipse_pos, &ellipse, &cuboid_pos, &cuboid, 0.0);
    let ctct = query::contact(&ellipse_pos, &ellipse, &cuboid_pos, &cuboid, 0.0);

    assert!(relative_eq!(dist, 1.0, epsilon = 1.0e-6));
    assert_eq!(prox, Proximity::Disjoint);
    assert!(ctct.is_none());
}
