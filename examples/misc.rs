#![allow(unused_variables)]

extern crate nalgebra as na;
extern crate ncollide;

use na::{Pnt3, Iso3};
use ncollide::inspection::Repr;
use ncollide::shape::{Ball, Cylinder, Cone};
use ncollide::narrow_phase::{CollisionDispatcher, BasicCollisionDispatcher, PlaneSupportMap,
                             IncrementalContactManifoldGenerator, OneShotContactManifoldGenerator};


fn main() {
    let dispatcher: BasicCollisionDispatcher<Pnt3<f64>, Iso3<f64>> = BasicCollisionDispatcher::new(0.10f64);
    let shape1 = Ball::new(0.5);
    let shape2 = Cylinder::new(0.5, 1.0);
    let shape3 = Cone::new(0.5, 1.0);

    let ball_vs_cylinder_detector = dispatcher.get_collision_algorithm(&shape1.repr(), &shape2.repr());
    let ball_vs_cone_detector     = dispatcher.get_collision_algorithm(&shape1.repr(), &shape3.repr());
    let cylinder_vs_cone_detector = dispatcher.get_collision_algorithm(&shape2.repr(), &shape3.repr());


    let plane_vs_ball: PlaneSupportMap<Pnt3<f32>, Iso3<f32>> = PlaneSupportMap::new(0.04);
    let full_manifold_plane_vs_ball = IncrementalContactManifoldGenerator::new(0.04, plane_vs_ball);


    let plane_vs_ball: PlaneSupportMap<Pnt3<f32>, Iso3<f32>> = PlaneSupportMap::new(0.04);
    let full_manifold_plane_vs_ball = OneShotContactManifoldGenerator::new(0.04, plane_vs_ball);
}
