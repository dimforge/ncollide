#[macro_use]
extern crate approx;
extern crate nalgebra as na;
extern crate ncollide2d;

use na::{Isometry2, Point2, Translation2, Vector2};
use ncollide2d::query::{self, Proximity};
use ncollide2d::shape::{CompositeShape, CompositeShape, Cuboid, Shape, Shape};
use ncollide2d::partitioning::BVT;
use ncollide2d::bounding_volume::AABB;

struct CrossedCuboids {
    bvt: BVT<usize, AABB<f32>>,
}

impl CrossedCuboids {
    pub fn new() -> CrossedCuboids {
        // The shape indices paired with their corresponding AABBs.
        // Nedded to initialize the acceleration structure.
        let aabbs = vec![
            (0, CrossedCuboids::generate_aabb(0)),
            (1, CrossedCuboids::generate_aabb(1)),
        ];

        CrossedCuboids {
            bvt: BVT::new_balanced(aabbs),
        }
    }

    // Helper function to generate the AABB bounding the i-th cuboid.
    fn generate_aabb(i: usize) -> AABB<f32> {
        if i == 0 {
            // The AABB for the horizontal cuboid.
            AABB::new(Point2::new(-1.0, 0.0), Point2::new(3.0, 2.0))
        } else {
            // The AABB for the vertical cuboid.
            AABB::new(Point2::new(0.0, -1.0), Point2::new(2.0, 3.0))
        }
    }

    // Helper function to generate the i-th cuboid.
    fn generate_cuboid(i: usize) -> Cuboid<f32> {
        if i == 0 {
            // Create a 4x2 cuboid. Remember that we must provide the
            // half-lengths.
            Cuboid::new(Vector2::new(2.0, 1.0))
        } else {
            // Create a 2x4 cuboid. Remember that we must provide the
            // half-lengths.
            Cuboid::new(Vector2::new(1.0, 2.0))
        }
    }
}

impl CompositeShape<Point2<f32>, Isometry2<f32>> for CrossedCuboids {
    fn map_part_at(&self, i: usize, f: &mut FnMut(&Isometry2<f32>, &Shape<f32>)) {
        // The translation needed to center the cuboid at the point (1, 1).
        let transform = Isometry2::new(Vector2::new(1.0, 1.0), na::zero());

        // Create the cuboid on-the-fly.
        let cuboid = CrossedCuboids::generate_cuboid(i);

        // Call the function.
        f(&transform, &cuboid)
    }

    fn map_transformed_part_at(
        &self,
        i: usize,
        m: &Isometry2<f32>,
        f: &mut FnMut(&Isometry2<f32>, &Shape<f32>),
    ) {
        // Prepend the translation needed to center the cuboid at the point (1, 1).
        let transform = m * Translation2::new(1.0, 1.0);

        // Create the cuboid on-the-fly.
        let cuboid = CrossedCuboids::generate_cuboid(i);

        // Call the function.
        f(&transform, &cuboid)
    }

    fn aabb_at(&self, i: usize) -> AABB<f32> {
        // Compute the i-th AABB.
        CrossedCuboids::generate_aabb(i)
    }

    fn bvt(&self) -> &BVT<usize, AABB<f32>> {
        // Reference to the acceleration structure.
        &self.bvt
    }
}

impl Shape<Point2<f32>, Isometry2<f32>> for CrossedCuboids {
    fn aabb(&self, m: &Isometry2<f32>) -> AABB<f32> {
        // This is far from an optimal AABB.
        AABB::new(
            m.translation * Point2::new(-10.0, -10.0),
            m.translation * Point2::new(10.0, 10.0),
        )
    }

    fn as_composite_shape(&self) -> Option<&CompositeShape<f32>> {
        Some(self)
    }
}

fn main() {
    let cross = CrossedCuboids::new();
    let cuboid = Cuboid::new(Vector2::new(1.0, 1.0));

    let cross_pos = na::one();
    let cuboid_pos = Isometry2::new(Vector2::new(6.0, 0.0), na::zero());

    let dist = query::distance(&cross_pos, &cross, &cuboid_pos, &cuboid);
    let prox = query::proximity(&cross_pos, &cross, &cuboid_pos, &cuboid, 0.0);
    let ctct = query::contact(&cross_pos, &cross, &cuboid_pos, &cuboid, 0.0);

    assert!(relative_eq!(dist, 2.0));
    assert_eq!(prox, Proximity::Disjoint);
    assert!(ctct.is_none());
}
