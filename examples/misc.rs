#![allow(unused_variables)]

extern crate nalgebra as na;
extern crate ncollide;

use na::{Point3, Isometry3};
use ncollide::inspection::Repr;
use ncollide::shape::{Ball, Cylinder, Cone};
use ncollide::narrow_phase::{CollisionDispatcher, DefaultCollisionDispatcher, PlaneSupportMapCollisionDetector,
                             IncrementalContactManifoldGenerator, OneShotContactManifoldGenerator};


fn main() {
    let dispatcher: DefaultCollisionDispatcher<Point3<f64>, Isometry3<f64>> = DefaultCollisionDispatcher::new();
    let shape1 = Ball::new(0.5);
    let shape2 = Cylinder::new(0.5, 1.0);
    let shape3 = Cone::new(0.5, 1.0);

    let ball_vs_cylinder_detector = dispatcher.get_collision_algorithm(&shape1.repr(), &shape2.repr());
    let ball_vs_cone_detector     = dispatcher.get_collision_algorithm(&shape1.repr(), &shape3.repr());
    let cylinder_vs_cone_detector = dispatcher.get_collision_algorithm(&shape2.repr(), &shape3.repr());


    let plane_vs_ball: PlaneSupportMapCollisionDetector<Point3<f32>, Isometry3<f32>> =
        PlaneSupportMapCollisionDetector::new();
    let full_manifold_plane_vs_ball = IncrementalContactManifoldGenerator::new(plane_vs_ball);


    let plane_vs_ball: PlaneSupportMapCollisionDetector<Point3<f32>, Isometry3<f32>> =
        PlaneSupportMapCollisionDetector::new();
    let full_manifold_plane_vs_ball = OneShotContactManifoldGenerator::new(plane_vs_ball);
}
