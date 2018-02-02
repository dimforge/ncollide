#![allow(unused_variables)]

extern crate nalgebra as na;
extern crate ncollide;

use na::{Isometry3, Point3};
use ncollide::shape::{Ball, Cone, Cylinder};
use ncollide::narrow_phase::{ContactDispatcher, DefaultContactDispatcher,
                             IncrementalContactManifoldGenerator, OneShotContactManifoldGenerator,
                             PlaneSupportMapContactGenerator};

fn main() {
    let dispatcher: DefaultContactDispatcher<Point3<f64>, Isometry3<f64>> =
        DefaultContactDispatcher::new();
    let shape1 = Ball::new(0.5);
    let shape2 = Cylinder::new(0.5, 1.0);
    let shape3 = Cone::new(0.5, 1.0);

    let ball_vs_cylinder_detector = dispatcher.get_contact_algorithm(&shape1, &shape2);
    let ball_vs_cone_detector = dispatcher.get_contact_algorithm(&shape1, &shape3);
    let cylinder_vs_cone_detector = dispatcher.get_contact_algorithm(&shape2, &shape3);

    let plane_vs_ball: PlaneSupportMapContactGenerator<Point3<f32>, Isometry3<f32>> =
        PlaneSupportMapContactGenerator::new();
    let full_manifold_plane_vs_ball = IncrementalContactManifoldGenerator::new(plane_vs_ball);

    let plane_vs_ball: PlaneSupportMapContactGenerator<Point3<f32>, Isometry3<f32>> =
        PlaneSupportMapContactGenerator::new();
    let full_manifold_plane_vs_ball = OneShotContactManifoldGenerator::new(plane_vs_ball);
}
