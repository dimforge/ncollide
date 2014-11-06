#![allow(unused_variables)]

extern crate "nalgebra" as na;
extern crate ncollide;

use ncollide::shape::{Ball, Cylinder, Cone};
use ncollide::narrow_phase::{ShapeShapeDispatcher, ShapeShapeDispatcher3, PlaneSupportMap,
                             PlaneSupportMap3, IncrementalContactManifoldGenerator,
                             OneShotContactManifoldGenerator};


fn main() {
    let dispatcher: ShapeShapeDispatcher3 = ShapeShapeDispatcher::new(0.10f32);
    let geom1 = Ball::new(0.5f32);
    let geom2 = Cylinder::new(0.5f32, 1.0);
    let geom3 = Cone::new(0.5f32, 1.0);

    let ball_vs_cylinder_detector = dispatcher.dispatch(&geom1, &geom2);
    let ball_vs_cone_detector     = dispatcher.dispatch(&geom1, &geom3);
    let cylinder_vs_cone_detector = dispatcher.dispatch(&geom2, &geom3);


    let plane_vs_ball: PlaneSupportMap3<Ball<f32>> = PlaneSupportMap::new(0.04);
    let full_manifold_plane_vs_ball = IncrementalContactManifoldGenerator::new(0.04, plane_vs_ball);


    let plane_vs_ball: PlaneSupportMap3<Ball<f32>> = PlaneSupportMap::new(0.04);
    let full_manifold_plane_vs_ball = OneShotContactManifoldGenerator::new(0.04, plane_vs_ball);
}
